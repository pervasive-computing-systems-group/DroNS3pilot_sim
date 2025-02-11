import abc
from dronekit import VehicleMode
from pymavlink import mavutil
import math
import time
import subprocess as sb
from threading import Thread, Event
import defines
import signal
from energy_budget import EnergyBudget
from pymavlink import mavutil
import subprocess
import sys
import os
import generate_data
import random

'''
TODO: Overall, refactor command init() function (not the basic __init__() function) to a different name. Very confusing currently, 
make naming convention more descriptive (Maybe start() is a better name). 
We should create a telemetry class to collect and retrieve data from Misisons and Commands that can be passed as an optional argument. 
We should try to remove all global variables.
We should try to make all filepaths passed rather than hardcoded - or in the defines file for any library. 
'''

# ph = ProcessHandler(debug=defines.debug)

# def signal_handler(signum, frame):
# 		ph.signal_handler(signum, frame)
# 		exit(1)


# signal.signal(signal.SIGINT, signal_handler)

def setHolder(holder_t):
	global holder 
	holder = holder_t

def get_xy(passed_vehicle):
	return [passed_vehicle.location.local_frame.east, passed_vehicle.location.local_frame.north]


class Command(object):
	__metaclass__ = abc.ABCMeta

	def __init__(self):
		pass

	@abc.abstractmethod
	def begin(self):
		pass

	def update(self):
		time.sleep(0.1)

	@abc.abstractmethod
	def is_done(self):
		pass

#TODO: Research vehicle.simple_takeoff and other takeoff techniques
class GainAlt(Command):
	def __init__(self, target_altitude, passed_vehicle):
		self.target_altitude = target_altitude
		self.vehicle = passed_vehicle
		

	def begin(self):
		self.vehicle.mode = VehicleMode('GUIDED')

		if self.vehicle.location.global_relative_frame.alt < 1:
			while not self.vehicle.armed:
				self.vehicle.armed = True
				time.sleep(0.5)
			self.vehicle.simple_takeoff(self.target_altitude)
		else:
			goto_position_target_local_enu(0, 0, self.target_altitude, self.vehicle)

	def is_done(self):
		diff = abs(self.vehicle.location.global_relative_frame.alt - self.target_altitude)
		return diff < 0.5

class Wait(Command):
	def __init__(self, wait_time, passed_vehicle, debug = False):

		self.wait_time = wait_time
		self.start_time = 0
		self.vehicle = passed_vehicle
		self.time_elapsed = 0
		self.debug = debug

	def begin(self):
		self.start_time = time.time()
		if self.debug:
			print("Waiting " + str(self.wait_time) + " seconds")
	
	def is_done(self):
		self.time_elapsed = time.time() - self.start_time

		if self.time_elapsed >= self.wait_time:
			return True
		return False
	


#TODO: Refactor data_collected - also what does this even do? Consider if this is even needed or if there is a better way to do this.
class StartTimer(Command):
	def __init__(self):
		self.data_collected = 0

	def begin(self):
		pass

	def is_done(self):
		return True

class StopTimer(Command):
	def __init__(self):
		pass

	def begin(self):
		pass

	def is_done(self):
		return True


#Command for client server connection in experiment
#TODO: update .pln file with correct waypoints
#TODO: maybe add energy budget stuff modeled for Connect
class Connect(Command):
	def __init__(self, passed_vehicle, first, bytes_sent): 
		self.vehicle = passed_vehicle 
		self.bytes_sent = bytes_sent
		self.data = ''
		self.success = False
		self.done = False
		self.first = first
	
	def begin(self):
		self.start_time = time.time()
		generate_data.generate_data(self.bytes_sent, "send.txt")
		self.connect()

	#TODO: sending has no output, either need to fix that or make receiving have correct number of bytes
	def connect(self):
		try: #try catch to continue program if server and client can't connect
			if self.first:
				os.chdir("Server_Client")
				subprocess.run(["make"], shell = True)
				with open("../connection_data.txt", "w"):
					pass #clear file before new run
			result = subprocess.check_output(["./Client/client " + defines.IP_ADDRESS + " 8080 S send.txt"], shell = True,  text= True)
			#print("CLIENT STDOUT:")
			#print(result)
			self.data = "Connected"
			self.success = True
			self.done = True
		except subprocess.CalledProcessError as exc:
			print("Error in subprocess: ", exc.returncode, exc.output)
			self.data = "ERROR: unable to connect "
			self.success = False
			self.done = True
		#else:

		#Get output of client and output to file along with distance from pi
		self.total_time = time.time() - self.start_time
		with open("../connection_data.txt", "a") as output_file:
			output_file.write("Distance: " + str(self.distance_finder()) + ", Data: " + self.data + ", Number of bytes sent: " + str(self.bytes_sent) +  ", Time: " + str(self.total_time) + "\n") 
	
	def distance_finder(self):
		return abs(math.sqrt(
			(self.vehicle.location.local_frame.north) ** 2 + 
			(self.vehicle.location.local_frame.east) ** 2 + 
			(self.vehicle.location.local_frame.down) ** 2))
	
	def connection_success(self):
		return self.success
	
	def is_done(self):
		return self.done
		

class MoveToWaypoint(Command): #used to be WaypointDist
	def __init__(self, east, north, up, passed_vehicle, tolerance = 0.5, debug = False):
		self.east = east
		self.north = north
		self.up = up
		self.vehicle = passed_vehicle
		self.tolerance = tolerance
		self.debug = debug

	def begin(self):
		self.cost = self.distance_finder()
		self.vehicle.mode = VehicleMode('GUIDED')
		if self.debug:
			print("Moving to " + str(self.east) + ", " + str(self.north) + ", " + str(self.up))
		goto_position_target_local_enu(self.east, self.north, self.up, self.vehicle)

	def distance_finder(self):
		return abs(math.sqrt(
					(self.vehicle.location.local_frame.north - self.north) ** 2 + 
					(self.vehicle.location.local_frame.east - self.east) ** 2 + 
					(self.vehicle.location.local_frame.down) ** 2))
	
	def is_done(self):
		target_dist = abs(math.sqrt(
			(self.vehicle.location.local_frame.north - self.north) ** 2 + 
			(self.vehicle.location.local_frame.east - self.east) ** 2 + 
			(self.vehicle.location.local_frame.down + self.up) ** 2))
		return target_dist < self.tolerance

'''
A waypoint with high-tolerance. The command ends once the drone is within 3 meters of the waypoint. The high-tolerance allows the drone to change direction and pursue the next waypoint without stopping. If now follow-on command is sent, then the drone will stop at the given position.
'''
class PassWaypoint(Command):
	def __init__(self, east, north, up, passed_vehicle, debug = False):
		self.east = east
		self.north = north
		self.up = up
		self.vehicle = passed_vehicle
		self.debug = debug

	def begin(self):
		self.cost = self.distance_finder()
		self.vehicle.mode = VehicleMode('GUIDED')
		if self.debug:
			print("Moving to " + str(self.east) + ", " + str(self.north) + ", " + str(self.up))
		goto_position_target_local_enu(self.east, self.north, self.up, self.vehicle)

	def distance_finder(self):
		return abs(math.sqrt(
					(self.vehicle.location.local_frame.north - self.north) ** 2 + 
					(self.vehicle.location.local_frame.east - self.east) ** 2 + 
					(self.vehicle.location.local_frame.down) ** 2))
	
	def is_done(self):
		target_dist = abs(math.sqrt(
			(self.vehicle.location.local_frame.north - self.north) ** 2 + 
			(self.vehicle.location.local_frame.east - self.east) ** 2 + 
			(self.vehicle.location.local_frame.down + self.up) ** 2))
		return target_dist < 7.5

#TODO: More accurate commenting and naming. This should be similar to the previous one, 
# just based off of time instead of a tolerance. Probably not the most useful.
class WaypointTime(Command):
	def __init__(self, east, north, up, seconds, passed_vehicle):
		self.east = east
		self.north = north
		self.up = up
		self.seconds = seconds
		self.vehicle = passed_vehicle

	def begin(self):
		self.vehicle.mode = VehicleMode('GUIDED')
		goto_position_target_local_enu(self.east, self.north, self.up)
		self.time_start = time.time()

	def is_done(self):
		seconds_elapsed = time.time() - self.time_start 
		return seconds_elapsed > self.seconds

#TODO: Potentially refactor this into a more comprehensive initialization command
class Idle(Command):
	def __init__(self, mode, passed_vehicle):
		self.mode = mode
		self.can_idle = True
		self.is_idle = False	
		self.vehicle = passed_vehicle

	def begin(self):
		self.vehicle.mode = VehicleMode('GUIDED')

	def update(self):
		if self.can_idle and not self.is_idle:
			self.vehicle.mode = VehicleMode(self.mode)
			self.is_idle = True
		time.sleep(0.1)

	def is_done(self):
		return False

#TODO: Refactor to use code from WaypointDist command - no reason to have another one.
class ReturnHome(Command):
	def __init__(self, alt, passed_vehicle, debug = False):
		self.east = 0
		self.north = 0
		self.up = alt
		self.vehicle = passed_vehicle
		self.cost = self.distance_finder()
		self.debug = debug

	def begin(self):
		if self.debug:
			print("Returning to " + str(self.east) + ", " + str(self.north) + ", " + str(self.up))
		self.vehicle.mode = VehicleMode('GUIDED')
		goto_position_target_local_enu(self.east, self.north, self.up, self.vehicle)

	def distance_finder(self):
		return abs(math.sqrt(
					(self.vehicle.location.local_frame.north - self.north) ** 2 + 
					(self.vehicle.location.local_frame.east - self.east) ** 2 + 
					(self.vehicle.location.local_frame.down) ** 2))
	

	def is_done(self):
		target_dist = abs(math.sqrt(
			(self.vehicle.location.local_frame.north - self.north) ** 2 + 
			(self.vehicle.location.local_frame.east - self.east) ** 2 + 
			(self.vehicle.location.local_frame.down + self.up) ** 2))
		return target_dist < 0.5

#TODO: change land_in_place function - we can have the code inside this object and anything that needs it can access it that way.
class Land(Command):
	def __init__(self, passed_vehicle, debug = False):
		self.vehicle = passed_vehicle
		self.debug = debug

	def begin(self):
		if self.debug:
			print("Landing . . .")
		land_in_place(self.vehicle)

	def is_done(self):
		return self.vehicle.location.global_relative_frame.alt < 0.1

class CollectDataGeneral(Command):

	def __init__(self, node_east, node_north, node_up, power, vehicle, telem, sim = False, payload = 500000, delay = True, collect_time = 5000):
		self.data = telem
		self.node_east = node_east
		self.node_north = node_north
		self.node_up = node_up
		self.power = power
		self.east = 0
		self.north = 0
		self.thread = None
		self.node_hostname = None
		self.node_collect_time = collect_time
		self.running_sim = sim
		self.vehicle = vehicle
		self.communication_path = defines.Comms_Path

		self.payload = payload
		self.delay = delay


		self.collect_success = Event()
		self.collect_success.clear()
		self.collect_complete = Event()
		self.collect_complete.clear()

	def begin(self):
		# Create thread for comms process
		if self.node_collect_time is not None:
			self.thread = Thread(target=self.launchCollection)
			self.thread.start()
			holder.add_thread(self.thread)
		else:
			print("ERROR: failed to find node info")
			# Set complete-flag
			self.collect_complete.set()
	
	def launchCollection(self):
		# Start comms process, wait for response
		print("Starting data collection process")
	
		# Are we running the simulation?
		if self.running_sim:
			dist_to_node = abs(math.sqrt(
				(self.vehicle.location.local_frame.north - self.node_north) ** 2 + 
				(self.vehicle.location.local_frame.east - self.node_east) ** 2 + 
				(self.vehicle.location.local_frame.down + self.node_up) ** 2))
			# Collect data using NS3
			child = sb.Popen([defines.NS_3_PATH, "run", "scratch/drone-to-sensor", "--", "--distance="+str(dist_to_node), 
		     	"--payload="+str(self.payload), "--txpower="+str(self.power), "--delay=" + "true" if self.delay else "false"],  stdout=sb.DEVNULL)
			holder.add_process(child)
			child.communicate()[0]
			rc = child.returncode
		else:
			if self.communication_path is not None:
			# Collect data using collect_data executable
				try:
					child = sb.Popen([self.communication_path, str(self.node_ID), 
						str(self.node_hostname), str(self.node_collect_time)], stdout=sb.DEVNULL)
					holder.add_process(child)
					child.communicate()[0]
					rc = child.returncode
				except Exception:
					print("ERROR: Subprocess failed to open comminication protocol. Please verify file integrity.")
					rc = 1
				
			else:
				print("ERROR: No communication protocol set. Please override default comm_path value when initializing.")
				rc = 1

		# If return on comms process was successful, set success-flag
		if rc == 0:
			#global data_collected
			if self.data.data_collected is None:
				self.data.data_collected = 0
			print("Successfully collected " + str(self.payload) + " from node", self.node_east, self.node_north, self.node_up)
			self.data.data_collected += self.payload
			self.collect_success.set()
		# else, leave success-flag unset
		else:
			print("failed to collect from node " + str(self.node_ID))
		
		# Set complete-flag, rejoin
		self.collect_complete.set()

	def is_done(self):
		if self.collect_complete.is_set():
			if self.thread is not None:
				self.thread.join()
			return True
		else:
			return False
		
	def collection_success(self):
		return self.collect_success.is_set()

#TODO: Potentially refactor Collect Data to include MoveAndCollectData
class CollectData(Command):
	# Collect data from node, with node communication range node_range (for simulation)
	def __init__(self, node, power,  vehicle, mpath, telem, sim = False, comm_path = None):
		self.data = telem
		self.node_ID = node
		self.power = power
		self.east = 0
		self.north = 0
		self.thread = None
		self.node_hostname = None
		self.node_collect_time = None
		self.running_sim = sim
		self.vehicle = vehicle
		self.node_path = mpath
		self.communication_path = comm_path
		
		# Find data about this node
		file1 = open( self.node_path)
		for aline in file1:
			values = aline.split()
			if int(values[0]) == self.node_ID:
				self.node_hostname = values[1]
				self.node_collect_time = int(values[2])
				self.east = float(values[3])
				self.north = float(values[4])
				break
		file1.close()
		# Thread events for collection success/complete
		self.collect_success = Event()
		self.collect_success.clear()
		self.collect_complete = Event()
		self.collect_complete.clear()

	def begin(self):
		self.cost = self.distance_finder()
		# Create thread for comms process
		if self.node_collect_time is not None:
			self.thread = Thread(target=self.launchCollection)
			self.thread.start()
			holder.add_thread(self.thread)
		else:
			print("ERROR: failed to find node " + str(self.node_ID) + " info")
			# Set complete-flag
			self.collect_complete.set()

	def launchCollection(self):
		# Start comms process, wait for response
		#print("Starting data collection process")
		self.start =  time.time()
		# Are we running the simulation?
		if self.running_sim:
			dist_to_node = abs(math.sqrt(
				(self.vehicle.location.local_frame.north - self.north) ** 2 + 
				(self.vehicle.location.local_frame.east - self.east) ** 2 + 
				(self.vehicle.location.local_frame.down) ** 2))
			# Collect data using NS3
			child = sb.Popen([defines.NS_3_PATH, "run", "scratch/drone-to-sensor", "--", "--distance="+str(dist_to_node), 
		     	"--payload=5000000", "--txpower="+str(self.power), "--delay=true"],  stdout=sb.DEVNULL)
			holder.add_process(child)
			child.communicate()[0]
			rc = child.returncode
		else:
			if self.communication_path is not None:
			# Collect data using collect_data executable
				try:
					child = sb.Popen([self.communication_path, str(self.node_ID), 
						str(self.node_hostname), str(self.node_collect_time)], stdout=sb.DEVNULL)
					holder.add_process(child)
					child.communicate()[0]
					rc = child.returncode
				except Exception:
					print("ERROR: Subprocess failed to open comminication protocol. Please verify file integrity.")
				
			else:
				print("ERROR: No communication protocol set. Please override default comm_path value when initializing.")

		# If return on comms process was successful, set success-flag
		if rc == 0:
			#global data_collected
			if self.data.data_collected is None:
				self.data.data_collected = 0
			print("Successfully collected 5000000 from node " + str(self.node_ID))
			self.data.data_collected += 5000000
			self.collect_success.set()
		# else, leave success-flag unset
		else:
			print("failed to collect from node " + str(self.node_ID))
		
		self.end = time.time()
		self.time = self.end-self.start
		# Set complete-flag, rejoin
		self.collect_complete.set()
	
	def distance_finder(self):
		return abs(math.sqrt(
					(self.vehicle.location.local_frame.north - self.north) ** 2 + 
					(self.vehicle.location.local_frame.east - self.east) ** 2 + 
					(self.vehicle.location.local_frame.down) ** 2))

	def is_done(self):
		if self.collect_complete.is_set():
			if self.thread is not None:
				self.thread.join()
			return True
		else:
			return False
		
	def collection_success(self):
		return self.collect_success.is_set()



# Run data collection script
class CollectWSNData(Command):
	# Collect data from node, with node communication range node_range (for simulation)
	def __init__(self, vehicle, node, sim = False, node_data_path = 'data/node_data.dat', comm_path = './Networking/Client/collect_data', node_data = None):
		# Data about the node we are connecting to:
		self.node_ID = int(node)
		self.east = 0
		self.north = 0
		self.altitude = 0
		self.safeAGL = 0
		self.node_type = 0
		self.byte_to_collect = 0
		self.node_hostname = None
		# Other stuff that we need
		self.vehicle = vehicle
		self.thread = None
		self.running_sim = sim
		self.node_path = node_data_path
		self.communication_path = comm_path
		# Fill in node data
		if node_data == None:
			# Read node info from a file...
			file1 = open(self.node_path)
			# Find this node in the file
			for aline in file1:
				values = aline.split()
				if int(values[0]) == self.node_ID:
					self.node_hostname = values[1]
					self.east = float(values[2])
					self.north = float(values[3])
					self.altitude = float(values[4])
					self.byte_to_collect = 1000000
					break
			file1.close()
		else:
			# Get node data from node_data argument
			# node_data = [x, y, z, z_s, Q, type, ip]
			self.east = node_data[0]
			self.north = node_data[1]
			self.altitude = node_data[2]
			self.safeAGL = node_data[3]
			self.byte_to_collect = node_data[4]
			self.node_type = node_data[5]
			self.node_hostname = node_data[6]
		
		# Thread events for collection success/complete
		self.collect_success = Event()
		self.collect_success.clear()
		self.collect_complete = Event()
		self.collect_complete.clear()

	def begin(self):
		# Create thread for comms process
		if self.node_hostname is not None:
			self.thread = Thread(target=self.launchCollection)
			self.thread.start()
			holder.add_thread(self.thread)
		else:
			print("ERROR:CollectWSNData: failed to find node " + str(self.node_ID) + " info")
			# Set complete-flag
			self.collect_complete.set()

	def launchCollection(self):
		# Start comms process, wait for response
		print(f'Collecting data from {self.node_ID}')
		self.start =  time.time()
		# Are we running the simulation?
		if self.running_sim:
			print(f"Simulating data TX, expecting {self.byte_to_collect} bytes")
			dist_to_node = abs(math.sqrt(
				(self.vehicle.location.local_frame.north - self.north) ** 2 + 
				(self.vehicle.location.local_frame.east - self.east) ** 2 + 
				(self.vehicle.location.local_frame.down - self.altitude) ** 2))
			# Collect data using SimpleNetSim
			child = sb.Popen([defines.SNS_PATH, str(dist_to_node), str(self.byte_to_collect)],  stdout=sb.DEVNULL)
			holder.add_process(child)
			child.communicate()[0]
			rc = child.returncode
		else:
			if self.communication_path is not None:
				# Collect data using collect_data executable
				dist_to_node = abs(math.sqrt(
					(self.vehicle.location.local_frame.north - self.north) ** 2 + 
					(self.vehicle.location.local_frame.east - self.east) ** 2 + 
					(self.vehicle.location.local_frame.down - self.altitude) ** 2))
				try:
					child = sb.Popen([self.communication_path, str(self.node_hostname), str(dist_to_node)], stdout=sb.DEVNULL)
					holder.add_process(child)
					child.communicate()[0]
					rc = child.returncode
				except Exception:
					print("ERROR: Subprocess failed to open comminication protocol. Please verify file integrity.")
				
			else:
				print("ERROR: No communication protocol set. Please override default comm_path value when initializing.")

		# If return on comms process was successful, set success-flag
		if rc == 0:
			self.collect_success.set()
		# else, leave success-flag unset
		else:
			print("failed to collect from node " + str(self.node_ID))
		
		self.end = time.time()
		self.time = self.end-self.start
		# Set complete-flag, rejoin
		self.collect_complete.set()
	
	def distance_finder(self):
		return abs(math.sqrt(
					(self.vehicle.location.local_frame.north - self.north) ** 2 + 
					(self.vehicle.location.local_frame.east - self.east) ** 2 + 
					(self.vehicle.location.local_frame.down) ** 2))

	def is_done(self):
		if self.collect_complete.is_set():
			if self.thread is not None:
				self.thread.join()
			return True
		else:
			return False
		
	def collection_success(self):
		return self.collect_success.is_set()



# Run data collection script
class CollectTXData(Command):
	# Collect data from node, arguents: vehicle, sim?, node-ip, distance, bytes-(for simulation), comms exe path
	def __init__(self, vehicle, sim = False, node_ip='localhost', distance=0, bytes=1000, comm_path = './Networking/Client/collect_data'):
		# Data about the node we are connecting to:
		self.distance = distance
		self.node_type = 0
		self.byte_to_collect = bytes
		self.node_hostname = node_ip
		# Other stuff that we need
		self.vehicle = vehicle
		self.thread = None
		self.running_sim = sim
		self.communication_path = comm_path
		
		# Thread events for collection success/complete
		self.collect_success = Event()
		self.collect_success.clear()
		self.collect_complete = Event()
		self.collect_complete.clear()

	def begin(self):
		# Create thread for comms process
		if self.node_hostname is not None:
			self.thread = Thread(target=self.launchCollection)
			self.thread.start()
			holder.add_thread(self.thread)
		else:
			print("ERROR:CollectWSNData: no hostname")
			# Set complete-flag
			self.collect_complete.set()

	def launchCollection(self):
		# Start comms process, wait for response
		print(f'Collecting data @ {self.node_hostname}')
		self.start =  time.time()
		# Are we running the simulation?
		if self.running_sim:
			print(f"Simulating data TX, expecting {self.byte_to_collect} bytes")
			# Collect data using SimpleNetSim
			child = sb.Popen([defines.SNS_PATH, str(self.distance), str(self.byte_to_collect)],  stdout=sb.DEVNULL)
			holder.add_process(child)
			child.communicate()[0]
			rc = child.returncode
		else:
			if self.communication_path is not None:
				# Collect data using collect_data executable
				try:
					child = sb.Popen([self.communication_path, str(self.node_hostname), str(self.distance)], stdout=sb.DEVNULL)
					holder.add_process(child)
					child.communicate()[0]
					rc = child.returncode
				except Exception:
					print("ERROR: Subprocess failed to open comminication protocol. Please verify file integrity.")
				
			else:
				print("ERROR: No communication protocol set. Please override default comm_path value when initializing.")

		# If return on comms process was successful, set success-flag
		if rc == 0:
			self.collect_success.set()
		# else, leave success-flag unset
		else:
			print("failed to collect at " + str(self.node_hostname))
		
		self.end = time.time()
		self.time = self.end-self.start
		# Set complete-flag, rejoin
		self.collect_complete.set()
	
	def distance_finder(self):
		return abs(math.sqrt(
					(self.vehicle.location.local_frame.north - self.north) ** 2 + 
					(self.vehicle.location.local_frame.east - self.east) ** 2 + 
					(self.vehicle.location.local_frame.down) ** 2))

	def is_done(self):
		if self.collect_complete.is_set():
			if self.thread is not None:
				self.thread.join()
			return True
		else:
			return False
		
	def collection_success(self):
		return self.collect_success.is_set()


#TODO: Refactor both MoveAndCollectData to contain both.
class MoveAndCollectData(Command):
	# Attempt to collect data from node, while moving towards the node 
	# at altitude alt, with node communication range node_range (for simulation)
	'''
		Attempt to collect data from node, while moving towards the node
		at altitude alt, with node connection power 'power' (for simulation)

		Available algorithms: 
			DEFAULT: Default configuration, stops when it recieves a signal from the node
			NAIVE: No early stopping, always stops at the known location of the node

		node_path: path to node info file. Default value is "node_info.txt"
			The node info file lists the nodes in the WSN. Each line contains one node. Coordinates are relative to home locations.
				[node number] [IP address] [sensor data size] [relative x] [relative y]
	'''
	def __init__(self, node, alt, power, vehicle, telem, algorithm = "DEFAULT", node_path = "sample_wsn_mission/node_info.txt", sim = False):

		self.node_ID = node
		self.power = power
		self.east = 0
		self.north = 0
		self.up = alt
		self.node_hostname = None
		self.node_collect_time = None
    
		self.data = telem
		self.vehicle = vehicle
		self.running_sim = sim
		self.algorithm = algorithm
		self.node_path = node_path

		# Find data about this node
    
		file1 = open(self.node_path,"r+")
		for aline in file1:
			values = aline.split()
			if int(values[0]) == self.node_ID:
				self.node_hostname = values[1]
				self.node_collect_time = int(values[2])
				self.east = float(values[3])
				self.north = float(values[4])
				break
		file1.close()
    
		self.collect_success = False
		self.stopped_at_point = False
		# Thread event for collection complete
		self.collect_complete = Event()
		self.collect_complete.clear()
		self.thread = None

	def begin(self):
		self.cost = self.distance_finder()
		print("Starting move-collect command for node ", self.node_ID)
		# Move towards the node
		self.vehicle.mode = VehicleMode('GUIDED')
		goto_position_target_local_enu(self.east, self.north, self.up, self.vehicle)

	def update(self):
		# Check if we found data on this node
		if self.node_collect_time is not None:
			#Get distance to node
			target_dist = abs(math.sqrt(
					(self.vehicle.location.local_frame.north - self.north) ** 2 + 
					(self.vehicle.location.local_frame.east - self.east) ** 2 + 
					(self.vehicle.location.local_frame.down + self.up) ** 2))
			# Attempt to communicate with this node
			if not self.collect_success and self.launchCollection():
				print("Connected to node, stopping for data collection")
				# Made contact, stop and collect data
				if self.algorithm == "DEFAULT":
					send_stop()
				self.collect_success = True
				self.thread = Thread(target=self.collection_thread)
				self.thread.start()
				holder.add_thread(self.thread)
			elif self.algorithm == "NAIVE" and self.collect_success and target_dist < 0.5 and not self.stopped_at_point:
				send_stop()
				print("Stopping at node for data collection")
				self.stopped_at_point = True
			else: 
				#  Check if we reached the node
				if target_dist < 0.5:
					# Reached node, could not connect...
					self.collect_complete.set()
		else:
			print("ERROR: failed to find node " + str(self.node_ID) + " info")
			# Set complete-flag
			self.collect_complete.set()
			



	def launchCollection(self):
		# Start comms process with 0 wait time, wait for response
		self.start = time.time()
		# Are we running the simulation?
		if self.running_sim:
			dist_to_node = abs(math.sqrt(
				(self.vehicle.location.local_frame.north - self.north) ** 2 + 
				(self.vehicle.location.local_frame.east - self.east) ** 2 + 
				(self.vehicle.location.local_frame.down) ** 2))
			# Attempt to contact node using NS3, disable delay, set data to 1 byte
			child = sb.Popen([defines.NS_3_PATH, "run", "scratch/drone-to-sensor", "--", "--distance="+str(dist_to_node), 
		     	"--payload=5000000", "--txpower="+str(self.power), "--delay=false"], stdout=sb.DEVNULL)
			holder.add_process(child)
			child.communicate()[0]
			rc = child.returncode
		else:
			# Attempt to contact node using collect_data executable with transmission time = 0
			child = sb.Popen(["/home/pi/MinLatencyWSN/MinLat_autopilot/Networking/Client/collect_data", str(self.node_ID), str(self.node_hostname), "0"], 
		    	stdout=sb.DEVNULL)
			holder.add_process(child)
			child.communicate()[0]
			rc = child.returncode

		# If return on comms process was successful, return true
		if rc == 0:
			print("Successfully collected 5000000 from node " + str(self.node_ID))
			#global data_collected
			self.data.data_collected += 5000000
			self.end = time.time()
			self.time =self.end - self.start
			return True
		# else, return false
		else:
			print("failed to collect from node " + str(self.node_ID))
			self.end = time.time()
			self.time =self.end - self.start
			return False


	def collection_thread(self):
		time.sleep(self.node_collect_time/1000.0)
		self.collect_complete.set()

	def distance_finder(self):
		return abs(math.sqrt(
					(self.vehicle.location.local_frame.north - self.north) ** 2 + 
					(self.vehicle.location.local_frame.east - self.east) ** 2 + 
					(self.vehicle.location.local_frame.down) ** 2))

	def is_done(self):
		if self.collect_complete.is_set():
			return True
		else:
			return False
		
	def collection_success(self):
		return self.collect_success
		
# 	def collection_success(self):
# 		return self.collect_success

class Sleep(Command):
	def __init__(self, time):
		self.time = time
		self.is_done = False
	def begin(self):
		self.sleep()
	def sleep(self):
		time.sleep(self.time)
		self.is_done = True
	def is_done(self):
		return self.is_done

def land_in_place(passed_vehicle):
	passed_vehicle.mode = VehicleMode("LAND")
	print("Landing")

	# Wait until the vehicle reaches the ground
	while True:
		# print(" Altitude: ", vehicle.location.global_relative_frame.alt)
		if passed_vehicle.location.global_relative_frame.alt < 0.1:
			print("Landed")
			break
		time.sleep(1)

	passed_vehicle.close()

def goto_position_target_local_enu(east, north, up, passed_vehicle):
	down = -up
	msg = passed_vehicle.message_factory.set_position_target_local_ned_encode(
			0,      # time_boot_ms (not used)
			0, 0,   # target system, target component
			mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
			0b0000111111111000, # type_mask (only positions enabled)
			north, east, down, # NED positions
			0, 0, 0, # NED velocities
			0, 0, 0, # NED accelerations (not supported),
			0, 0)    # yaw, yaw_rate (not supported)
	passed_vehicle.send_mavlink(msg)

def send_stop(passed_vehicle):
	"""
	Move vehicle in direction based on specified velocity vectors.
	"""
	msg = passed_vehicle.message_factory.set_position_target_global_int_encode(
		0,       # time_boot_ms (not used)
		0, 0,    # target system, target component
		mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
		0b0000111111000111, # type_mask (only speeds enabled)
		0, # lat_int - X Position in WGS84 frame in 1e7 * meters
		0, # lon_int - Y Position in WGS84 frame in 1e7 * meters
		0, # alt - Altitude in meters in AMSL altitude(not WGS84 if absolute or relative)
		# altitude above terrain if GLOBAL_TERRAIN_ALT_INT
		0, # X velocity in NED frame in m/s
		0, # Y velocity in NED frame in m/s
		0, # Z velocity in NED frame in m/s
		0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
		0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
	# Send message once
	passed_vehicle.send_mavlink(msg)

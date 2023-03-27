import abc
from dronekit import VehicleMode
import threading
import queue
import commands
import subprocess as sb
from collections import deque
import time
import numpy as np
import defines
import math
import signal
from solver import LKH_Solver

'''
TODO:

In general replace all global variables with arguments or objects.
For vehicle, if no argument is passed, initialize the vehicle (look at init() from simulation) in the parent Mission class
Make all file paths passed rather than hardcoded (maybe exception for LKH stuff or any other library)
Make general mission that uses a file with a series of commands in it.
Remove all NS3 calls.
Create a class that simulates the network communication stuff (The NS3 Stuff).
'''

# ph = ProcessHandler(debug=defines.debug)

# def signal_handler(signum, frame):
# 		ph.signal_handler(signum, frame)
# 		exit(1)


# signal.signal(signal.SIGINT, signal_handler)

#TODO: Remove reliance on global variables
def setSimulation(sim):
	global running_sim
	running_sim = sim
	# if running_sim:
	# 	import numpy as np

def setHolder(holder_t):
	global holder 
	holder = holder_t

def setSeed(vseed):
	global seed
	seed = vseed

def setPath(vpath):
	global path
	path = vpath
	commands.setPath(path)


#TODO: Refactor to be passed into the mission
def pass_vehicle(passed_vehicle):
	global vehicle
	vehicle = passed_vehicle


class Mission(object):
	__metaclass__ = abc.ABCMeta
	terminate = False
	name = "Name not set"
	thread = None
	q = deque()

	#TODO: Refactor to include optional global arguments - simulation + any others we want
	@abc.abstractmethod
	def __init__(self):
		pass

	def start(self):
		self.thread = threading.Thread(target=self.update_wrapper, name=self.name)
		self.thread.start()
		if running_sim:
			# Wait here for the thread to re-join
			self.thread.join()

	# Wrapper function for updating mission if not terminated
	def update_wrapper(self):
		self.command.begin()
		
		#This assumes we always go to LAND - we may need some other signal to trigger the mission to pause or stop
		#TODO: more robust failsafe
		while not self.terminate:
			if vehicle.mode == VehicleMode("GUIDED") and vehicle.system_status == "ACTIVE":
				self.update()
			else:
				time.sleep(0.1)
				print("Vehicle no longer in guided or in non-stable flight mode")

	# Periodically called to check command status/is-done
	def update(self):
		# Check current command
		if self.command.is_done():
			# Current command finished, check if there is another command
			if self.q:
				# Run next command
				self.command = self.q.popleft()
				self.command.begin()
			else:
				# deque is empty
				self.dispose()
		# Current command isn't complete, call update on command
		else:
			#  Command not complete, call update
			self.command.update()

	def dispose(self):
		self.terminate = True

	

#TODO: Rename? Name not intuitive
class Manual(Mission):
	name = "MANUAL"

	def __init__(self):
		self.is_loiter = False
		self.command_idle = commands.Idle('LOITER')
		self.command_idle.can_idle = False
		self.command = commands.GainAlt(2)
		self.q.append(self.command_idle)

	def update(self):
		if vehicle.channels['3'] > 1500 and not self.is_loiter:
			self.is_loiter = True
			self.command_idle.can_idle = True
		super(Manual, self).update()



#TODO: Refactor all CollectWSNData into single mission with an algorithm parameter
class CollectWSNData(Mission):

	'''
		Mission to collect data from a series of WSN nodes. 

		plan_path: path to drone plan file. Default value is "drone_plan.pln".
			The drone plan file utilizes a series of commands, with one command per line. The available commands are:
				0: Fly to waypoint relative to home location
					0 [relative x] [relative y] [relative altitude]
				1: Connect to WSN node
					1 [node number] [communication power]
				2: Set mission altitude
					2 [mission altitude]

		node_path: path to node info file. Default value is "node_info.txt"
			The node info file lists the nodes in the WSN. Each line contains one node. Coordinates are relative to home locations.
				[node number] [IP address] [sensor data size] [relative x] [relative y]

		output_path: path to directory where results will be stored. Default value is "" and will write to the working directory

		algorithm: Chooses which algorithm to use. Default value is "DEFAULT"
			Available algorithms:
				DEFAULT: The default configuration. Any time the drone cannot connect to a node, it moves toward the node until it connects. If there are
					multiple nodes that did not connect at a single stop, it will use the order in the drone plan file.
				NAIVE: Alters the DEFAULT algorithm by not stopping when it connects, but always stopping directly on top of the node.
				ONLINE: Alters the DEFAULT algorithm by ordering the subtours via the LKH TSP solver algorithm. (Currently not working ://)
				NO_SUB: Alters the DEFAULT algorithm by not executing subtours.

	'''


	name = "WSN_DATA"
	missed_q = deque()

	CMD_WAYPOINT = 0
	CMD_COLL_DATA = 1
	CMD_MSN_ALT = 2
	start_time = 0
	end_time = 0
	

	def __init__(self, plan_path = "sample_wsn_mission/drone_plan.pln", node_path = "sample_wsn_mission/node_info.txt", output_path = "./", algorithm = "DEFAULT"):

		self.plan_path = plan_path
		self.algorithm = algorithm
		self.output_path = output_path
		self.node_path = node_path

		commands.init_data_collected()
		# Set random seed for consistancy
		np.random.seed(seed)
		self.mission_alt = 50
		# Add take-off command
		self.q.append(commands.GainAlt(self.mission_alt))
		# Add Timer-Start command
		self.q.append(commands.StartTimer())
		# Get list of commands from file
		file1 = open(self.plan_path,"r+")
		# Node power-settings
		self.nPowers = {-1: 0}
		# Run through each command in list
		for aline in file1:
			values = aline.split()
			# If cmd = 0 (waypoint)
			if int(values[0]) == self.CMD_WAYPOINT:
				# Add waypoint movement to queue
				self.q.append(commands.WaypointDist(float(values[1]), float(values[2]), float(values[3])))
			# else if cmd = 1 (data collection)
			elif int(values[0]) == self.CMD_COLL_DATA:
				print(values)
				# TODO: Add a normal-distribution for the nodes range
				arr = np.random.normal(float(values[2]) - 1, 8, 1)
				self.nPowers[int(values[1])] = arr[0]
				# Add collect data command
				self.q.append(commands.CollectData(int(values[1]), arr[0]))
			elif int(values[0]) == self.CMD_MSN_ALT:
				# Set mission altitude
				self.mission_alt = float(values[1])
			else:
				print("ERROR: unexpected cmd in pln file")
		file1.close()
		# Add Return-To-Home command
		self.q.append(commands.ReturnHome(self.mission_alt))
		# Add Timer-Stop command
		self.q.append(commands.StopTimer())
		if running_sim:
			# Add land command
			self.q.append(commands.Land())
		# Add first command as current command
		self.command = self.q.popleft()
		print("Node power settings")
		print(self.nPowers)

	def update(self):
		if isinstance(self.command, commands.CollectData):
			# Check if we are done collecting data
			if self.command.is_done():
				# Finished, check is collection was successful
				if self.command.collection_success():
					print("Total collected data: " + str(commands.data_collected))
				else:
					print("Total collected data: " + str(commands.data_collected))
					# print("Failed collected data from " + str(self.command.node_ID))
					# Add this node to missed nodes queue
					self.missed_q.append(self.command.node_ID)
					# Check if we are done collecting data at the hovering location
				if not isinstance(self.q[0], commands.CollectData):
					if self.algorithm == "ONLINE":
						hpp_points = []
						hpp_points_id = []
						while self.missed_q:
							n = self.missed_q.pop()
							hpp_points_id.append(n)
							file1 = open(self.node_path,"r+")
							for aline in file1:
								values = aline.split()
								if int(values[0]) == n:
									east = float(values[3])
									north = float(values[4])
							file1.close()
							# Add moving-collecting from this node to the command queue
							hpp_points.append([east,north])
						if isinstance(self.q[0], commands.WaypointDist):
							hpp_points.append([self.q[0].east, self.q[0].north])
							hpp_points_id.append(-1)
							hpp_points.append(commands.get_xy())
							hpp_points_id.append(-1)
							
						else:
							hpp_points.append([self.q[0].east, self.q[0].north])
							hpp_points_id.append(-1)

						print(hpp_points)

						if len(hpp_points) < 3:
							for p, i in zip(hpp_points, hpp_points_id):
								if i == -1:
									self.q.appendleft(commands.WaypointDist(p[0], p[1], self.mission_alt))
								else:
									self.q.appendleft(commands.MoveAndCollectData(i, self.mission_alt, self.nPowers[i], node_path = self.node_path))
						else:
							lkh = LKH_Solver([hpp_points, len(hpp_points) - 2, len(hpp_points) - 1, holder])
							LKH_path = lkh.solve()
							for p in LKH_path:
								if hpp_points_id[p] == -1:
									self.q.appendleft(commands.WaypointDist(hpp_points[p][0], hpp_points[p][1], self.mission_alt))
								else:
									self.q.appendleft(commands.MoveAndCollectData(hpp_points_id[p], self.mission_alt, self.nPowers[p], node_path = self.node_path))

					# Done collecting data at this point, start recovery
					while self.missed_q:
						n = self.missed_q.pop()
						if self.algorithm != "NO_SUB":
							print("Added move-collect command")
							# Add moving-collecting from this node to the command queue
							self.q.appendleft(commands.MoveAndCollectData(n, self.mission_alt, self.nPowers[n], algorithm = "NAIVE" if self.algorithm == "NAIVE" else "Default", node_path = self.node_path))
		# Check to see if we just finished a move-collect command
		if isinstance(self.command, commands.MoveAndCollectData):
			# Check if this was the last move-collect command
			if not isinstance(self.q[0], commands.MoveAndCollectData):
				# TODO: Update the next waypoint
				pass
		if isinstance(self.command, commands.StartTimer):
			self.start_time = time.time()
			print("Starting Timer")

		if isinstance(self.command, commands.StopTimer):
			self.end_time = time.time()
			print("Stopping Timer")
			lapsed = self.end_time - self.start_time
			print(lapsed)
			if self.output_path[-1] != "/":
				self.output_path += "/"

			with open(self.output_path + "flight-time.dat", "a") as tfile:
				tfile.write("0 " + str(lapsed) + "\n")

			with open(self.output_path + "data_collected.dat", "a") as dfile:
				dfile.write("0 " + str(commands.data_collected) + "\n")

			print("Data Collected:", commands.data_collected)
			commands.data_collected = 0
			self.terminate = True

		super(CollectWSNData, self).update()

		#Prints LKH parameters to a the required file
# def PrintLKHFile(self, hpp_points, start, end):
# 	with open("FixedHPP.par", "w") as parFile:
# 		parFile.write("PROBLEM_FILE = FixedHPP.tsp\n")
# 		parFile.write("COMMENT Fixed Hamiltonian Path Problem\n")
# 		parFile.write("TOUR_FILE = LKH_output.dat\n")

# 	with open("FixedHPP.tsp", "w") as dataFile:
# 		dataFile.write("NAME : FixedHPP \n")
# 		dataFile.write("COMMENT : Fixed Hamiltonian Path Problem \n")
# 		dataFile.write("TYPE : TSP \n")
# 		dataFile.write("DIMENSION : " + str(len(hpp_points)) + "\n")
# 		dataFile.write("EDGE_WEIGHT_TYPE : EXPLICIT \n")
# 		dataFile.write("EDGE_WEIGHT_FORMAT : FULL_MATRIX\n")

# 		dataFile.write("EDGE_WEIGHT_SECTION\n")
# 		for i in range(len(hpp_points)):
# 			for j in range(len(hpp_points)):
# 				if(i == start and j == end) or (i == end and j == start):
# 					dataFile.write("0.0\t")
# 				else:
# 					if i == start or i == end or j == start or j == end:
# 						dataFile.write(str(self.FindNodeDistance(hpp_points[i], hpp_points[j])*1000) + "\t")
# 					else:
# 						dataFile.write(str(self.FindNodeDistance(hpp_points[i], hpp_points[j])) + "\t")
# 			dataFile.write("\n")
# 		dataFile.write("EOF\n")
# 	#Finds the distance between two points
# def FindNodeDistance(self, a, b):
# 	return math.sqrt(((a[0] - b[0])**2 + (a[1] - b[1])**2))

# #Runs the LKH solver and returns the solution
# def getLKHSolution(self, start, end):
# 	lkh_process = sb.Popen([defines.LKH_PATH, "FixedHPP.par"])
# 	holder.add_process(lkh_process)


# 	# process_num = ph.add_process(lkh_process)
# 	lkh_process.communicate()[0]
# 	# ph.remove_process(process_num)

# 	with open("LKH_output.dat") as outputFile:
# 		lines = outputFile.readlines()
# 	lkh_path = []
	
# 	for i in range(6, len(lines) - 1):
# 		if int(lines[i]) != -1:
# 			lkh_path.append(int(lines[i]) -1)


# 	print(lkh_path)
# 	if lkh_path[0] != start or lkh_path[-1] != end:
		
# 		if lkh_path[1] == end and lkh_path[0] == start:
# 			lkh_path.append(lkh_path[0])
# 			lkh_path.pop(0)
# 			lkh_path.reverse()
		
# 		while lkh_path[0] != start:
# 			print("Rotating list...")
# 			lkh_path.append(lkh_path[0])
# 			lkh_path.pop(0)

# 		if lkh_path[1] == end:
# 			lkh_path.append(lkh_path[0])
# 			lkh_path.pop(0)
# 			lkh_path.reverse()

# 		if lkh_path[-1] != end:
# 			raise(Exception("Temp exception 2 for TSP failing to function as HPP"))
	

# 	return lkh_path





CC=g++

HEADER = include/defines.h
CFLAGS = -Wall -Iinclude -pthread

build: Client/collect_data.cpp Server/server.cpp $(HEADER)
	$(CC) $(CFLAGS) -o Client/collect_data Client/collect_data.cpp
	$(CC) $(CFLAGS) -o Server/server Server/server.cpp

clean: 
	rm -rf Client/*o Server/*o Client/collect_data Server/server

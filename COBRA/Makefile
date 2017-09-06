all: main.cpp Agent.cpp Endpoint.cpp Graph.cpp Node.cpp Simulation.cpp
	gcc -o cobra \
	main.cpp \
	Agent.cpp Endpoint.cpp Graph.cpp \
	Node.cpp Simulation.cpp \
	-I . \
	-I /usr/include/c++/7.1.1/ \
	-lboost_graph \
	-lstdc++ \
	-fpermissive 

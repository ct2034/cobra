#pragma once
#include <vector>
#include <iostream>
#include <string>
#include <sstream>
#include <cstring>
#include <cassert>
#include <fstream>
#include <stdlib.h>
#include <stdio.h>
#include <climits>

using namespace std;


#include<boost/tokenizer.hpp>
#include <dlib/optimization/max_cost_assignment.h>

#include "Endpoint.h"
#include "Agent.h"

#include <iostream>
#include <cstdlib>
#include <ctime>

class Simulation
{
public:

	Simulation(int num_agents, int num_tasks, int tasks_per_timestep);
	~Simulation();

	void Kiva(int podRows, int podCols, int podLength, int numAgents, int num_tasks, int tasks_per_timestep);
	
	//save
	void ShowTask();
	void SavePath(string fname);
	void SaveTask(const string &fname);
	void minCost();

private:
	// initialize
	void LoadMap(string fname);
	void LoadTask(string fname);
	
private:
	int row, col;
	vector<bool> my_map;
	vector<bool> DeliverGoal; //goals of DELIVER tasks
	double focal_w;
	//task
	vector<list<Task>> tasks_total;
	list<Task*> tasks_assign;
	list<Task*> tasks_deliver;

	vector<Endpoint> endpoints;
	//agent
	vector<Agent> agents;

	unsigned int maxtime;//map_size * num_agents
	unsigned int timestep;
	
	
	int workpoint_num;
	int t_task;//timestep of last task

	vector<int> endpoint_hashtable;//loc->endpointID
};


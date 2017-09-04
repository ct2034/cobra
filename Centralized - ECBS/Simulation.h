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

#include "ecbs_search.h"

#include <iostream>
#include <cstdlib>
#include <ctime>

class Simulation
{
public:

	Simulation(string map_name, string task_name);
	~Simulation();
	

	//run
	void run( double focal_w);
	
	//save
	void ShowTask();
	void SavePath(string fname);
	void SaveTask(const string &fname, const string &instance_name);
	void SaveThroughput(const string &fname);
	void minCost();

	double computation_time;
	int num_computations;

private:
	// initialize
	void LoadMap(string fname);
	void LoadTask(string fname);

	void AssignTasks(vector<Agent*> &agents, const vector<vector<int> > &cons_paths);
	bool PathFinding(vector<Agent*> &agents, const vector<vector<int> > &cons_paths);
	bool TestConstraints();
	
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


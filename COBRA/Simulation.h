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
//#include <float.h>

#include<boost/tokenizer.hpp> //use to read file

#include "Endpoint.h"
#include "Agent.h"
using namespace std;


class Simulation
{
public:

	Simulation(string map_name, string task_name);
	~Simulation();
	

	//run
	void run_TOTP();
	void run_TPTR();

	//save
	void ShowTask();
	void SavePath(const string &fname);
	void SaveTask(const string &fname, const string &instance_name);
	void SaveThroughput(const string &fname);

	double computation_time;
	int num_computations;

private:
	// initialize
	void LoadMap(string fname);
	void LoadTask(string fname);
	// test 
	bool TestConstraints();
private:
	int row, col;
	Token token;
	vector<list<Task>> tasks;
	vector<Endpoint> endpoints;
	vector<Agent> agents;

	unsigned int maxtime;
	
	int workpoint_num; //number of endpoints that may have tasks on. Other endpoints are home endpoints
	int t_task;//timestep that last task appears

};


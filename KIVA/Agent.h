#pragma once
#include <vector>
#include <list>

#include <string>
#include <functional>  // for std::hash (c++11 and above)
#include <map>

#include "Node.h"
#include "Endpoint.h"

using namespace std;

class Task;

class Agent
{
public:
	Agent() {};
	Agent(int loc, int col,int row, int id, int maxtime);
	Agent(const Agent &ag);
	~Agent();
	void Set(int loc, int col, int row, int id, int maxtime);
	void reset(const Agent &ag);

	vector<int> path;  // a path that takes the agent from initial to start to goal location satisfing all constraints
	//Endpoint* home;
	int loc;
	Endpoint* next_ep;
	
	int id;
	unsigned int maxtime;
	unsigned int timestep;//current timestep
	unsigned int arrive_time;
	Task *task;
	int row;
	int col;
	bool delivering;

};


class Task
{
public:
	Task(Endpoint *start, Endpoint *goal, int start_time, int goal_time)
		:start(start), goal(goal), start_time(start_time), goal_time(goal_time), delivering(false) {}
	~Task() {}

	Endpoint *start;
	Endpoint *goal;

	Agent* ag;
	unsigned int ag_arrive_start;
	unsigned int ag_arrive_goal;
	int start_time; //min time agent stay at start point
	int goal_time; //min time agent stay at goal point
	bool delivering;

};


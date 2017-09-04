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
class Token;

class Agent
{
public:
	Agent() {};
	Agent(int loc, int col,int row, int id, int maxtime);
	Agent(const Agent &ag);
	~Agent();
	void Set(int loc, int col, int row, int id, unsigned int maxtime);
	void reset(const Agent &ag);
	bool TOTP(Token &token);//time ordered token passing 
	bool TPTR(Token &token);//token passing and task robbing
	
public:
	vector<unsigned int> path;
	int loc;
	int id;
	unsigned int maxtime;
	unsigned int finish_time; //time that the robot finishs the current task
	Task *task;
	int row;
	int col;
	
private:
	int AStar(int start, int begin_time, const Endpoint &goal, const Token &token, int ag_hide); //return timestep or -1
	void updatePath(const Node &goal);
	inline void releaseClosedListNodes(map<unsigned int, Node*> &allNodes_table);
	inline bool isConstrained(int curr_id, int next_id, int next_timestep, const Token &token, int ag_hide);
	bool Move2EP(Token &token); // move to empty endpoint
};

typedef enum { WAIT, TAKEN } TaskState;

class Task
{
public:
	Task(Endpoint *start, Endpoint *goal, int start_time, int goal_time)
		:start(start), goal(goal), start_time(start_time), goal_time(goal_time), state(WAIT) {}
	~Task() {}

	Endpoint *start;
	Endpoint *goal;

	Agent* ag;
	unsigned int ag_arrive_start;
	unsigned int ag_arrive_goal;
	int start_time; //min time agent need to spend at start point
	int goal_time; //min time agent need to spend at goal point
	TaskState state;

};

class Token
{
public:
	Token() { timestep = 0; }
	Token(const Token &token);
	~Token() {}
	void reset(const Token &token);
	
	vector<bool> my_map;
	vector<bool> my_endpoints;
	list<Task*> tasks;
	vector<Agent*> agents;
	
	vector<vector<unsigned int> > path;//path[agent][time] = loc
	unsigned int timestep;
};

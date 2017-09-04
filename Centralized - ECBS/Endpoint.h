#pragma once
#include <vector>
#include "Node.h"


using namespace std;

typedef enum { START, GOAL, VACANT } EPState; //Endpoint State

class Endpoint
{
public:

	Endpoint() {};
	Endpoint(int loc) :loc(loc),hold(false) {};
	
	Endpoint(int loc, const vector<bool> &map, int col, int pt = 0);
	~Endpoint();
	void SetHVal(const vector<bool> &map, int col) { h_val = BFS(map, col); }

	bool hold;
//private:
	int id;
	int loc;
	EPState state; //only used in random agents
	vector<int> h_val;
	int start_time;
	int processing_time;
private:
	vector<int> BFS(const vector<bool> &map, int col); //breadth first search, return h_val
	
};


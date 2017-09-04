#pragma once
#include <vector>


using namespace std;

class Endpoint
{
public:

	Endpoint() {};
	Endpoint(int loc) :loc(loc) {};
	~Endpoint();
	void SetHVal(const vector<bool> &map, int col) { h_val = BFS(map, col); }


	int id;//endpoint id
	int loc;
	vector<int> h_val;//heuristic map
private:
	vector<int> BFS(const vector<bool> &map, int col); //breadth first search, return h_val
	
};


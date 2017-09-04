#include "Endpoint.h"
#include <queue>


Endpoint::Endpoint(int loc, const vector<bool> &map, int col, int pt):loc(loc),processing_time(pt)
{
	hold = false;
	for (unsigned int i = 0; i < h_val.size(); i++)
	{
		h_val[i] = -1;
	}
	h_val = BFS(map, col);
}



Endpoint::~Endpoint()
{
}


vector<int> Endpoint::BFS(const vector<bool> &map, int col) 
{ 
	queue<int> Q;
	vector<bool> status(map.size(), false);//false means undicovered
	vector<int> h(map.size(), -1);
	int neighbor[4] = { 1,-1,col,-col };
	status[loc] = true; 
	h[loc] = 0;
	Q.push(loc);
	while (!Q.empty())
	{	
		int v = Q.front();
		Q.pop();
		for (int i = 0; i < 4; i++)
		{
			int u = v + neighbor[i];
			if (map[u])
			{
				if (!status[u]) // u is undiscovered
				{ 
					status[u] = true;
					h[u] = h[v] + 1;
					Q.push(u);
				}
			}
		}		
	}
	return h;
}
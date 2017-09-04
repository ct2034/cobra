#pragma once

#include <math.h>
#include <iostream>

#include <boost/heap/fibonacci_heap.hpp>

#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/trim_all.hpp>


#include <cassert>
#include <fstream>
#include <string>
#include <limits>

using namespace std;
using boost::heap::fibonacci_heap;
using boost::heap::compare;

class Node {
public:

	int loc;
	double g_val;
	double h_val = 0;
	Node* parent;
	int timestep = 0;
	int num_internal_conf = 0;  // used in ECBS
	bool in_openlist = false;

	///////////////////////////////////////////////////////////////////////////////
	// NOTE -- Normally, compare_node (lhs,rhs) suppose to return true if lhs<rhs.
	//         However, Heaps in STL and Boost are implemented as max-Heap.
	//         Hence, to achieve min-Head, we return true if lhs>rhs
	///////////////////////////////////////////////////////////////////////////////

	// the following is used to comapre nodes in the OPEN list
	struct compare_node {
		// returns true if n1 > n2 (note -- this gives us *min*-heap).
		bool operator()(const Node* n1, const Node* n2) const {
			if (n1->g_val + n1->h_val == n2->g_val + n2->h_val)
				return n1->g_val <= n2->g_val;  // break ties towards larger g_vals
			return n1->g_val + n1->h_val >= n2->g_val + n2->h_val;
		}
	};  // used by OPEN (heap) to compare nodes (top of the heap has min f-val, and then highest g-val)

		// the following is used to comapre nodes in the FOCAL list
	struct secondary_compare_node {
		// returns true if n1 > n2
		bool operator()(const Node* n1, const Node* n2) const {
			if (n1->num_internal_conf == n2->num_internal_conf) {
				if (n1->g_val + n1->h_val == n2->g_val + n2->h_val)
					return n1->g_val <= n2->g_val;  // break ties towards larger g_vals
				return n1->g_val + n1->h_val >= n2->g_val + n2->h_val;  // break ties towards smaller f_vals (prefer shorter solutions)
			}
			return n1->num_internal_conf >= n2->num_internal_conf;  // n1 > n2 if it has more conflicts
		}
	};  // used by FOCAL (heap) to compare nodes (top of the heap has min number-of-conflicts)

		// define a typedefs for handles to the heaps (allow up to quickly update a node in the heap)
	typedef boost::heap::fibonacci_heap< Node*, compare<compare_node> >::handle_type open_handle_t;
	typedef boost::heap::fibonacci_heap< Node*, compare<secondary_compare_node> >::handle_type focal_handle_t;

	open_handle_t open_handle;
	focal_handle_t focal_handle;

	Node(int loc, double g_val, double h_val, Node* parent, int timestep, int num_internal_conf = 0, bool in_openlist = false)
		:loc(loc), g_val(g_val), h_val(h_val), parent(parent), timestep(timestep),
		num_internal_conf(num_internal_conf), in_openlist(in_openlist) {}
	Node::Node() : loc(0), g_val(0), h_val(0), parent(NULL), timestep(0), num_internal_conf(0), in_openlist(false) {}
	Node(const Node& other);
	~Node();

	inline double getFVal() const { return g_val + h_val; }
	// The following is used by googledensehash for checking whether two nodes are equal
	// we say that two nodes, s1 and s2, are equal if
	// both are non-NULL and agree on the id and timestep
	struct eqnode {
		bool operator()(const Node* s1, const Node* s2) const {
			return (s1 == s2) || (s1 && s2 && s1->loc == s2->loc && s1->timestep == s2->timestep);
		}
	};

	// The following is used by googledensehash for generating the hash value of a nodes
	// /* TODO:  */his is needed because otherwise we'll have to define the specilized template inside std namespace
	struct NodeHasher {
		std::size_t operator()(const Node* n) const {
			// cout << "COMPUTE HASH: " << *n << " ; Hash=" << hash<int>()(n->id) << endl;
			// cout << "   Pointer Address: " << n << endl;
			size_t id_hash = std::hash<int>()(n->loc);
			size_t timestep_hash = std::hash<int>()(n->timestep);
			return (id_hash ^ (timestep_hash << 1));
		}
	};
};

std::ostream& operator<<(std::ostream& os, const Node& n);
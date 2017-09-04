#include "Node.h"


Node::Node(const Node& other) {
	loc = other.loc;
	g_val = other.g_val;
	h_val = other.h_val;
	parent = other.parent;
	timestep = other.timestep;
	in_openlist = other.in_openlist;
	open_handle = other.open_handle;
	focal_handle = other.focal_handle;
	num_internal_conf = other.num_internal_conf;
}

Node::~Node() {
}

std::ostream& operator<<(std::ostream& os, const Node& n) {
	if (n.parent != NULL)
		os << "ID=" << n.loc << " ; TIMESTEP=" << n.timestep << " ; GVAL=" << n.g_val << " ; HVAL=" << n.h_val
		<< " ; PARENT=" << (n.parent)->loc
		<< " ; IN_OPEN?" << std::boolalpha << n.in_openlist;
	else
		os << "ID=" << n.loc << " ; TIMESTEP=" << n.timestep << " ; GVAL=" << n.g_val << " ; HVAL=" << n.h_val
		<< " ; ROOT (NO PARENT)";
	return os;
}


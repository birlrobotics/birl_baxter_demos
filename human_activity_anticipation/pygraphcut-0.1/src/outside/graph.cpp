/* graph.cpp */


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "graph.h"
//#include "instances.inc"

//template <typename captype, typename tcaptype, typename flowtype> 
	/*Graph<captype, tcaptype, flowtype>*/ Graph::Graph(int node_num_max, int edge_num_max, void (*err_function)(char *))
	: node_num(0),
	  nodeptr_block(NULL),
	  error_function(err_function)
{
	if (node_num_max < 16) node_num_max = 16;
	if (edge_num_max < 16) edge_num_max = 16;

	nodes = (node*) malloc(node_num_max*sizeof(node));
	arcs = (arc*) malloc(2*edge_num_max*sizeof(arc));
	if (!nodes || !arcs) { if (error_function) (*error_function)("Not enough memory!"); exit(1); }

	node_last = nodes;
	node_max = nodes + node_num_max;
	arc_last = arcs;
	arc_max = arcs + 2*edge_num_max;

	maxflow_iteration = 0;
	flow = 0;
}

//template <typename captype, typename tcaptype, typename flowtype> 
	/*Graph<captype, tcaptype, flowtype>*/ Graph::~Graph()
{
	if (nodeptr_block) 
	{ 
		delete nodeptr_block; 
		nodeptr_block = NULL; 
	}
	free(nodes);
	free(arcs);
}

//template <typename captype, typename tcaptype, typename flowtype> 
	void /*Graph<captype, tcaptype, flowtype>*/ Graph::reset()
{
	node_last = nodes;
	arc_last = arcs;
	node_num = 0;

	if (nodeptr_block) 
	{ 
		delete nodeptr_block; 
		nodeptr_block = NULL; 
	}

	maxflow_iteration = 0;
	flow = 0;
}

//template <typename captype, typename tcaptype, typename flowtype> 
	void /*Graph<captype, tcaptype, flowtype>*/ Graph::reallocate_nodes(int num)
{
	int node_num_max = (int)(node_max - nodes);
	node* nodes_old = nodes;

	node_num_max += node_num_max / 2;
	if (node_num_max < node_num + num) node_num_max = node_num + num;
	nodes = (node*) realloc(nodes_old, node_num_max*sizeof(node));
	if (!nodes) { if (error_function) (*error_function)("Not enough memory!"); exit(1); }

	node_last = nodes + node_num;
	node_max = nodes + node_num_max;

	if (nodes != nodes_old)
	{
		arc* a;
		for (a=arcs; a<arc_last; a++)
		{
			a->head = (node*) ((char*)a->head + (((char*) nodes) - ((char*) nodes_old)));
		}
	}
}

//template <typename captype, typename tcaptype, typename flowtype> 
	void /*Graph<captype, tcaptype, flowtype>*/ Graph::reallocate_arcs()
{
	int arc_num_max = (int)(arc_max - arcs);
	int arc_num = (int)(arc_last - arcs);
	arc* arcs_old = arcs;

	arc_num_max += arc_num_max / 2; if (arc_num_max & 1) arc_num_max ++;
	arcs = (arc*) realloc(arcs_old, arc_num_max*sizeof(arc));
	if (!arcs) { if (error_function) (*error_function)("Not enough memory!"); exit(1); }

	arc_last = arcs + arc_num;
	arc_max = arcs + arc_num_max;

	if (arcs != arcs_old)
	{
		node* i;
		arc* a;
		for (i=nodes; i<node_last; i++)
		{
			if (i->first) i->first = (arc*) ((char*)i->first + (((char*) arcs) - ((char*) arcs_old)));
		}
		for (a=arcs; a<arc_last; a++)
		{
			if (a->next) a->next = (arc*) ((char*)a->next + (((char*) arcs) - ((char*) arcs_old)));
			a->sister = (arc*) ((char*)a->sister + (((char*) arcs) - ((char*) arcs_old)));
		}
	}
}

// MOVEMENT TO GRAPH.CPP

///////////////////////////////////////
// Implementation - inline functions //
///////////////////////////////////////



//template <typename captype, typename tcaptype, typename flowtype> 
	/*typename Graph<captype, tcaptype, flowtype>*/ Graph::node_id /*Graph<captype, tcaptype, flowtype>*/ Graph::add_node(int num)
{
	assert(num > 0);

	if (node_last + num > node_max) reallocate_nodes(num);

	if (num == 1)
	{
		node_last -> first = NULL;
		node_last -> tr_cap = 0;
		node_last -> is_marked = 0;
		node_last -> is_in_changed_list = 0;

		node_last ++;
		return node_num ++;
	}
	else
	{
		memset(node_last, 0, num*sizeof(node));

		node_id i = node_num;
		node_num += num;
		node_last += num;
		return i;
	}
}

//template <typename captype, typename tcaptype, typename flowtype> 
	void /*Graph<captype, tcaptype, flowtype>*/ Graph::add_tweights(node_id i, tcaptype cap_source, tcaptype cap_sink)
{
	assert(i >= 0 && i < node_num);

	tcaptype delta = nodes[i].tr_cap;
	if (delta > 0) cap_source += delta;
	else           cap_sink   -= delta;
	flow += (cap_source < cap_sink) ? cap_source : cap_sink;
	nodes[i].tr_cap = cap_source - cap_sink;
}

//template <typename captype, typename tcaptype, typename flowtype> 
	void /*Graph<captype, tcaptype, flowtype>*/ Graph::add_edge(node_id _i, node_id _j, captype cap, captype rev_cap)
{
	assert(_i >= 0 && _i < node_num);
	assert(_j >= 0 && _j < node_num);
	assert(_i != _j);
	assert(cap >= 0);
	assert(rev_cap >= 0);

	if (arc_last == arc_max) reallocate_arcs();

	arc *a = arc_last ++;
	arc *a_rev = arc_last ++;

	node* i = nodes + _i;
	node* j = nodes + _j;

	a -> sister = a_rev;
	a_rev -> sister = a;
	a -> next = i -> first;
	i -> first = a;
	a_rev -> next = j -> first;
	j -> first = a_rev;
	a -> head = j;
	a_rev -> head = i;
	a -> r_cap = cap;
	a_rev -> r_cap = rev_cap;
}

//template <typename captype, typename tcaptype, typename flowtype> 
	/*typename Graph<captype, tcaptype, flowtype>*/ Graph::arc* /*Graph<captype, tcaptype, flowtype>*/ Graph::get_first_arc()
{
	return arcs;
}

//template <typename captype, typename tcaptype, typename flowtype> 
	/*typename Graph<captype, tcaptype, flowtype>*/ Graph::arc* /*Graph<captype, tcaptype, flowtype>*/ Graph::get_next_arc(arc* a) 
{
	return a + 1; 
}

//template <typename captype, typename tcaptype, typename flowtype> 
	void /*Graph<captype, tcaptype, flowtype>*/ Graph::get_arc_ends(arc* a, node_id& i, node_id& j)
{
	assert(a >= arcs && a < arc_last);
	i = (node_id) (a->sister->head - nodes);
	j = (node_id) (a->head - nodes);
}

//template <typename captype, typename tcaptype, typename flowtype> 
	Graph::tcaptype /*Graph<captype, tcaptype, flowtype>*/ Graph::get_trcap(node_id i)
{
	assert(i>=0 && i<node_num);
	return nodes[i].tr_cap;
}

//template <typename captype, typename tcaptype, typename flowtype> 
	Graph::captype /*Graph<captype, tcaptype, flowtype>*/ Graph::get_rcap(arc* a)
{
	assert(a >= arcs && a < arc_last);
	return a->r_cap;
}

//template <typename captype, typename tcaptype, typename flowtype> 
	void /*Graph<captype, tcaptype, flowtype>*/ Graph::set_trcap(node_id i, tcaptype trcap)
{
	assert(i>=0 && i<node_num); 
	nodes[i].tr_cap = trcap;
}

//template <typename captype, typename tcaptype, typename flowtype> 
	void /*Graph<captype, tcaptype, flowtype>*/ Graph::set_rcap(arc* a, captype rcap)
{
	assert(a >= arcs && a < arc_last);
	a->r_cap = rcap;
}


//template <typename captype, typename tcaptype, typename flowtype> 
	/*typename Graph<captype, tcaptype, flowtype>*/ Graph::termtype /*Graph<captype, tcaptype, flowtype>*/ Graph::what_segment(node_id i, termtype default_segm)
{
	if (nodes[i].parent)
	{
		return (nodes[i].is_sink) ? SINK : SOURCE;
	}
	else
	{
		return default_segm;
	}
}

//template <typename captype, typename tcaptype, typename flowtype> 
	void /*Graph<captype, tcaptype, flowtype>*/ Graph::mark_node(node_id _i)
{
	node* i = nodes + _i;
	if (!i->next)
	{
		// it's not in the list yet
		if (queue_last[1]) queue_last[1] -> next = i;
		else               queue_first[1]        = i;
		queue_last[1] = i;
		i -> next = i;
	}
	i->is_marked = 1;
}


//template <typename captype, typename tcaptype, typename flowtype> 
	int /*Graph<captype, tcaptype, flowtype>*/ Graph::get_node_num() { return node_num; }
//template <typename captype, typename tcaptype, typename flowtype> 
	int /*Graph<captype, tcaptype, flowtype>*/ Graph::get_arc_num() { return (int)(arc_last - arcs); }

//template <typename captype, typename tcaptype, typename flowtype> 
	void /*Graph<captype, tcaptype, flowtype>*/ Graph::remove_from_changed_list(node_id i) 
	{ 
		assert(i>=0 && i<node_num && nodes[i].is_in_changed_list); 
		nodes[i].is_in_changed_list = 0;
	}

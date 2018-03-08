/* energy.h */
/* Vladimir Kolmogorov (vnk@cs.cornell.edu), 2003. */

/*
	This software minimizes certain energy functions of binary variables, as described in 

		What Energy Functions can be Minimized via Graph Cuts?
		Vladimir Kolmogorov and Ramin Zabih. 
		In IEEE Transactions on Pattern Analysis and Machine Intelligence (PAMI), February 2004. 

	It uses maxflow algorithm described in 

		An Experimental Comparison of Min-Cut/Max-Flow Algorithms for Energy Minimization in Vision
		Yuri Boykov and Vladimir Kolmogorov.
		In IEEE Transactions on Pattern Analysis and Machine Intelligence (PAMI), September 2004.

	More specifically, it computes the global minimum of a function E of binary
	variables x_1, ..., x_n which can be written as a sum of terms involving
	at most three variables at a time:

		E(x_1, ..., x_n) = \sum_{i}     E^{i}    (x_i)
						+ \sum_{i,j}   E^{i,j}  (x_i, x_j)
						+ \sum_{i,j,k} E^{i,j,k}(x_i, x_j, x_k)

	The method works only if each term is "submodular". Definitions of submodularity
	for terms E^{i}, E^{i,j}, E^{i,j,k} are given below as comments to functions
	add_term1(), add_term2(), add_term3(). 

	This software can be used only for research purposes. IF YOU USE THIS SOFTWARE,
	YOU SHOULD CITE THE AFOREMENTIONED PAPERS IN ANY RESULTING PUBLICATION.

	In order to use it, you will also need a MAXFLOW software which can be
	obtained from http://www.cs.cornell.edu/People/vnk/software.html

	NOTE: This software minimizes functions of BINARY variables only.
	However, it can also be used for minimizing certain functions of non-binary
	(multi-label) variables via a sequence of binary moves (alpha-expansion, 
	alpha-beta swap, k-jumps, etc.) as proposed in

		Efficient Approximate Energy Minimization via Graph Cuts 
		Yuri Boykov, Olga Veksler, Ramin Zabih, 
		IEEE transactions on PAMI, vol. 20, no. 12, p. 1222-1239, November 2001.

	IF YOU USE THIS SOFTWARE FOR IMPLEMENTING ALPHA-EXPANSION OR ALPHA-BETA SWAP
	ALGORITHM, YOU SHOULD CITE THIS PAPER IN ANY RESULTING PUBLICATION.

	Also note that an implementation of minimization techniques for non-binary variables
	can be downloaded from O. Veksler's homepage: http://www.csd.uwo.ca/faculty/olga/code.html .

	------------------------------------------------------------------------

	Example usage
	(Minimizes the following function of 3 binary variables:
	E(x, y, z) = x - 2*y + 3*(1-z) - 4*x*y + 5*|y-z|):

	///////////////////////////////////////////////////

	#include <stdio.h>
	#include "energy.h"

	void main()
	{
		// Minimize the following function of 3 binary variables:
		// E(x, y, z) = x - 2*y + 3*(1-z) - 4*x*y + 5*|y-z|
		   
		Energy::Var varx, vary, varz;
		Energy *e = new Energy();

		varx = e -> add_variable();
		vary = e -> add_variable();
		varz = e -> add_variable();

		e -> add_term1(varx, 0, 1);  // add term x 
		e -> add_term1(vary, 0, -2); // add term -2*y
		e -> add_term1(varz, 3, 0);  // add term 3*(1-z)

		e -> add_term2(x, y, 0, 0, 0, -4); // add term -4*x*y
		e -> add_term2(y, z, 0, 5, 5, 0); // add term 5*|y-z|

		Energy::TotalValue Emin = e -> minimize();
		
		printf("Minimum = %d\n", Emin);
		printf("Optimal solution:\n");
		printf("x = %d\n", e->get_var(varx));
		printf("y = %d\n", e->get_var(vary));
		printf("z = %d\n", e->get_var(varz));

		delete e;
	}

	///////////////////////////////////////////////////
*/

#include <assert.h>
#include "energy.h"

/***********************************************************************/
/************************  Implementation ******************************/
/***********************************************************************/

Energy::Energy(void (*err_function)(char *)) : Graph(0,0,err_function)
{
	Econst = 0;
	error_function = err_function;
}

Energy::~Energy() {}

Energy::Var Energy::add_variable() {	return add_node(); }

void Energy::add_constant(Value A) { Econst += A; }

void Energy::add_term1(Var x,
                              Value A, Value B)
{
	if (A!=0 || B!=0) add_tweights(x, B, A);
}

void Energy::add_term2(Var x, Var y,
                              Value A, Value B,
                              Value C, Value D)
{
	/* 
	   E = A A  +  0   B-A
	       D D     C-D 0
	   Add edges for the first term
	*/
	if (D!=0 || A!=0) add_tweights(x, D, A);
	B -= A; C -= D;
	Value BC = B+C;
	/* now need to represent
	   0 B
	   C 0
	*/

	assert(BC >= 0); /* check regularity */
	if (B < 0)
	{
		/* Write it as
		   B B  +  -B 0  +  0   0
		   0 0     -B 0     B+C 0
		*/
		add_tweights(x, 0, B); /* first term */
		add_tweights(y, 0, -B); /* second term */
		if (BC > 0) add_edge(x, y, 0, BC); /* third term */
	}
	else if (C < 0)
	{
		/* Write it as
		   -C -C  +  C 0  +  0 B+C
		    0  0     C 0     0 0
		*/
		add_tweights(x, 0, -C); /* first term */
		add_tweights(y, 0, C); /* second term */
		if (BC > 0) add_edge(x, y, B+C, 0); /* third term */
	}
	else /* B >= 0, C >= 0 */
	{
		if (B>0 || C>0) add_edge(x, y, B, C);
	}
}

void Energy::add_term3(Var x, Var y, Var z,
                              Value E000, Value E001,
                              Value E010, Value E011,
                              Value E100, Value E101,
                              Value E110, Value E111)
{
	register Value pi = (E000 + E011 + E101 + E110) - (E100 + E010 + E001 + E111);
	register Value delta;
	register Var u;

	if (pi >= 0)
	{
		Econst += E111 - (E011 + E101 + E110);

		add_tweights(x, E101, E001);
		add_tweights(y, E110, E100);
		add_tweights(z, E011, E010);

		delta = (E010 + E001) - (E000 + E011); /* -pi(E[x=0]) */
		assert(delta >= 0); /* check regularity */
		add_edge(y, z, delta, 0);

		delta = (E100 + E001) - (E000 + E101); /* -pi(E[y=0]) */
		assert(delta >= 0); /* check regularity */
		add_edge(z, x, delta, 0);

		delta = (E100 + E010) - (E000 + E110); /* -pi(E[z=0]) */
		assert(delta >= 0); /* check regularity */
		add_edge(x, y, delta, 0);

		if (pi > 0)
		{
			u = add_variable();
			add_edge(x, u, pi, 0);
			add_edge(y, u, pi, 0);
			add_edge(z, u, pi, 0);
			add_tweights(u, 0, pi);
		}
	}
	else
	{
		Econst += E000 - (E100 + E010 + E001);

		add_tweights(x, E110, E010);
		add_tweights(y, E011, E001);
		add_tweights(z, E101, E100);

		delta = (E110 + E101) - (E100 + E111); /* -pi(E[x=1]) */
		assert(delta >= 0); /* check regularity */
		add_edge(z, y, delta, 0);

		delta = (E110 + E011) - (E010 + E111); /* -pi(E[y=1]) */
		assert(delta >= 0); /* check regularity */
		add_edge(x, z, delta, 0);

		delta = (E101 + E011) - (E001 + E111); /* -pi(E[z=1]) */
		assert(delta >= 0); /* check regularity */
		add_edge(y, x, delta, 0);

		u = add_variable();
		add_edge(u, x, -pi, 0);
		add_edge(u, y, -pi, 0);
		add_edge(u, z, -pi, 0);
		add_tweights(u, -pi, 0);
	}
}

Energy::TotalValue Energy::minimize() { return Econst + maxflow(); }

int Energy::get_var(Var x) { return (int)what_segment(x); }

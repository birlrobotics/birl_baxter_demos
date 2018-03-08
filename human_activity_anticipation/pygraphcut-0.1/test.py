# Copyright 2007 Thomas Finley, tfinley@gmail.com

"""Testing module for PyGraphcut.

This testing module tests the functionality of the Graph and Energy
classes, making sure that what functions that should work do work, and
those use cases that should not work do not work.  This module is
intended for development use to ensure nothing too important became
broken, and also for end users to ensure their compilation completed
successfully.

To run the tests execute this module, e.g., 'python test.py'."""

from graphcut import *
import unittest

# These are for backward compatibility with old Python versions.
try:
    a = set()
except NameError:
    import sets
    set = sets.Set
try:
    sorted([1,2])
except NameError:
    def sorted(seq):
        s = list(seq)
        s.sort()
        return s

class GraphNodeCreationTestCase(unittest.TestCase):
    """Test cases involving only adding nodes."""
    def setUp(self):
        self.graph = Graph()
        self.num_to_add = [1, 1, 5, 3]

    def testEmptyNodeCount(self):
        """Ensure that the node count is initially 0."""
        self.assertEqual(self.graph.num_nodes, 0)

    def testEmptySegmentList(self):
        """Ensure that the segment list is initially empty."""
        self.assertEqual(self.graph.segments, [])

    def testSingleAddition(self):
        """Ensure that adding a single node appears to work."""
        self.graph.add_node()
        self.assertEqual(self.graph.num_nodes, 1)

    def testIncrementalAdditions(self):
        """Ensure that addition really changes the number of nodes."""
        for which, to_add in enumerate(self.num_to_add):
            self.graph.add_node(to_add)
            should_number = sum(self.num_to_add[:which+1])
            self.assertEqual(should_number, self.graph.num_nodes)
            self.assertEqual(should_number, len(self.graph.segments))

    def testIncrementalAdditionsOrder(self):
        """Test that indices are added in the expected order."""
        for which, to_add in enumerate(self.num_to_add):
            index = self.graph.add_node(to_add)
            should_number = sum(self.num_to_add[:which])
            self.assertEqual(should_number, index)

    def testIndicesDisjoint(self):
        """Ensure that the indices assigned to new nodes are disjoint."""
        current_nodes = set()
        for to_add in self.num_to_add:
            first_index = self.graph.add_node(to_add)
            new_nodes = set(xrange(first_index, first_index+to_add))
            self.failIf(new_nodes.intersection(current_nodes))
            current_nodes.update(new_nodes)
        self.assertEqual(self.graph.num_nodes, len(current_nodes))

    def testBadAdditions(self):
        """Ensure that non-positive additions throw ValueErrors."""
        bad_num_to_add = [-2, 0, -1000]
        for to_add in bad_num_to_add:
            self.assertRaises(ValueError, self.graph.add_node, to_add)

class GraphEdgeCreationTestCase(unittest.TestCase):
    """Test cases involving setting capacities."""
    def setUp(self):
        self.graph = Graph()

    def testAddEdge(self):
        """Check adding various capacities."""
        index = self.graph.add_node(3)
        self.graph.add_edge(index,   index+1, 2, 3)
        self.graph.add_edge(index,   index+1, 2, 3)
        self.graph.add_edge(index+1, index+2, 0, 0)
        self.graph.add_edge(index+2, index+1, 1, 4)

    def testAddSourceSinkEdge(self):
        """Check adding a source and a sink edge."""
        index = self.graph.add_node()
        self.graph.add_tweights(index, 0, 2)

    def testAddSelfEdge(self):
        """Check adding a self edge fails."""
        index = self.graph.add_node()
        self.assertRaises(ValueError, self.graph.add_edge, index, index, 1, 0)

    def testAddNegativeEdge(self):
        """Check adding a negative capacity edge fails."""
        index, ae = self.graph.add_node(2), self.graph.add_edge
        self.assertRaises(ValueError, ae, index, index+1, -3, 0)
        self.assertRaises(ValueError, ae, index, index+1, -5000.2, -.29392)

    def testAddNegativeSSEdge(self):
        """Check negative source/sink capacities fails."""
        index, at = self.graph.add_node(), self.graph.add_tweights
        self.assertRaises(ValueError, at, index, -2, 2)
        self.assertRaises(ValueError, at, index, 2, -2)
        self.assertRaises(ValueError, at, index, -2, -2)

    def testAddNonexistentEdge(self):
        """Check edges between non-existent nodes fails."""
        index, ae = self.graph.add_node(5), self.graph.add_edge
        self.assertRaises(ValueError, ae, index, index+5, 2, 3)
        self.assertRaises(ValueError, ae, index+5, index, 2, 3)
        self.assertRaises(ValueError, ae, index-1, index, 2, 3)
        self.assertRaises(ValueError, ae, index, index-1, 2, 3)
        self.assertRaises(ValueError, ae, index, index+5000, 2, 3)

    def testAddNonexistentSSEdge(self):
        """Check source/sink capacities on an absent node fails."""
        index, at = self.graph.add_node(), self.graph.add_tweights
        self.assertRaises(ValueError, at, index+1, 2, 2)
        self.assertRaises(ValueError, at, index-1, 2, 2)
        self.assertRaises(ValueError, at, index-1000, 2, 2)

class GraphSegmentAccessTestCase(unittest.TestCase):
    """Tests for exceptions upon getting the segment of absent nodes."""
    def setUp(self):
        self.graph = Graph()

    def testAccessBadSegmentEmptyGraph(self):
        """Check exception throwing for empty graph."""
        for n in [0, 4, -2]:
            self.assertRaises(ValueError, self.graph.segment, n)

    def testAccessBadSegmentNodeGraph(self):
        """Check exception throwing for non-empty graph."""
        to_add = 20
        first_node = self.graph.add_node(to_add)
        for n in xrange(first_node-to_add, first_node):
            self.assertRaises(ValueError, self.graph.segment, n)
        for n in xrange(first_node, first_node+to_add):
            self.graph.segment(n)
        for n in xrange(first_node+to_add, first_node+to_add+to_add):
            self.assertRaises(ValueError, self.graph.segment, n)

class GraphSingleNodeCutTestCase(unittest.TestCase):
    """Test cases involving graph cuts with one non-source/sink node."""
    def setUp(self):
        self.graph = Graph()
        self.index = self.graph.add_node()

    def testNoEdges(self):
        """Check zero maxflow on an empty graph."""
        self.assertAlmostEqual(self.graph.maxflow(), 0.0)

    def testSimpleSink(self):
        """Check node is in sink cut on simple graph."""
        self.graph.add_tweights(self.index, 1.2, 2.12345)
        self.assertAlmostEqual(self.graph.maxflow(), 1.2)
        self.assertEqual(self.graph.segment(self.index), True)

    def testSimpleSource(self):
        """Check node is in source cut on simple graph."""
        self.graph.add_tweights(self.index, 1.4, 0.2)
        self.assertAlmostEqual(self.graph.maxflow(), 0.2)
        self.assertEqual(self.graph.segment(self.index), False)

    def testAdditiveCapacities(self):
        """Check that add_tweights behavior is additive."""
        self.graph.add_tweights(self.index, 0.6, 1.1)
        self.assertAlmostEqual(self.graph.maxflow(), 0.6)
        self.assertEqual(self.graph.segment(self.index), True)
        self.graph.add_tweights(self.index, 0.9, 0.2)
        self.assertAlmostEqual(self.graph.maxflow(), 1.3)
        self.assertEqual(self.graph.segment(self.index), False)

class GraphTwoNodeCutTestCase(unittest.TestCase):
    """Test cases involving graph cuts with two non-source/sink nodes."""
    def setUp(self):
        self.graph = Graph()
        self.index = self.graph.add_node(2)

    def testNoEdges(self):
        """Check zero maxflow on an empty graph."""
        self.assertAlmostEqual(self.graph.maxflow(), 0.0)

    def testSSCapacities(self):
        """Check maxflow on only source/sink capacity graph."""
        self.graph.add_tweights(self.index, 3.2, 1.1)
        self.graph.add_tweights(self.index+1, 0.7, 2.2)
        self.assertAlmostEqual(self.graph.maxflow(), 1.8)
        self.assertEqual(self.graph.segments, [False, True])
        # Run consistency checks on the other graph access methods.
        self.assertEqual(self.graph.source_nodes, [self.index])
        self.assertEqual(self.graph.sink_nodes, [self.index+1])
        self.assertEqual(self.graph.segment(self.index), False)
        self.assertEqual(self.graph.segment(self.index+1), True)

    def testPartitionConsistency(self):
        """Check consistency of various partition access methods."""
        self.testSSCapacities()
        self.graph.add_tweights(self.index, 3.2, 1.1)
        self.graph.add_tweights(self.index+1, 0.7, 2.2)
        self.graph.maxflow()
        # Run consistency checks on graph access methods.
        self.assertEqual(self.graph.segments, [False, True])
        self.assertEqual(self.graph.source_nodes, [self.index])
        self.assertEqual(self.graph.sink_nodes, [self.index+1])
        self.assertEqual(self.graph.segment(self.index), False)
        self.assertEqual(self.graph.segment(self.index+1), True)

    def testCapacities(self):
        """Check maxflow on capacity graph with non source/sink edge."""
        self.graph.add_tweights(self.index, 1.0, 4.0)
        self.graph.add_tweights(self.index+1, 4.0, 1.0)
        self.graph.add_edge(self.index, self.index+1, 8.0, 1.0)
        self.assertAlmostEqual(self.graph.maxflow(), 3.0)
        self.assertEqual(self.graph.segments, [True, False])

    def testAdditiveCapacities(self):
        """Check that edge capacity setting is additive."""
        self.testCapacities()
        self.graph.add_edge(self.index, self.index+1, 9.0, 1.5)
        self.assertAlmostEqual(self.graph.maxflow(), 4.5)
        self.assertEqual(self.graph.segments, [True, False])
        self.graph.add_edge(self.index, self.index+1, 900.0, 1.5)
        self.assertAlmostEqual(self.graph.maxflow(), 5.0)
        self.assertEqual(self.graph.segments, [False, False])

class GraphConsistencyTestCase(unittest.TestCase):
    """Test case to check consistency off partition access methods."""
    def makeRandom(self, seed):
        """Constructs a random graph based on a specified seed."""
        import random
        r = random.Random(seed)
        num_nodes = r.randint(5, 30)
        self.graph.add_node(num_nodes)
        for i in xrange(num_nodes * num_nodes):
            fromnode, tonode = tuple(r.sample(xrange(-1, num_nodes), 2))
            fcap, tcap = r.random(), r.random()
            if fromnode == -1:
                self.graph.add_tweights(tonode, fcap*num_nodes, 0)
            elif tonode == -1:
                self.graph.add_tweights(fromnode, 0, tcap*num_nodes)
            else:
                self.graph.add_edge(fromnode, tonode, fcap, tcap)

    def checkConsistent(self):
        """Checks whether node partitions are consistent."""
        segs, nn = self.graph.segments, xrange(self.graph.num_nodes)
        self.assertEqual(self.graph.num_nodes, len(segs))
        sinkNodes = [w for w,i in enumerate(segs) if i]
        sourceNodes = [w for w,i in enumerate(segs) if not i]
        self.assertEqual(sorted(self.graph.source_nodes), sorted(sourceNodes))
        self.assertEqual(sorted(self.graph.sink_nodes), sorted(sinkNodes))
        sinkNodes = [w for w in nn if self.graph.segment(w)]
        sourceNodes = [w for w in nn if not self.graph.segment(w)]
        self.assertEqual(sorted(self.graph.source_nodes), sorted(sourceNodes))
        self.assertEqual(sorted(self.graph.sink_nodes), sorted(sinkNodes))

    def testConsistency(self):
        """Check consistency of partition access functions."""
        for i in xrange(10):
            self.graph = Graph()
            self.makeRandom(i)
            self.graph.maxflow()
            self.checkConsistent()

class GraphChainTestCase(unittest.TestCase):
    """Test case of graph cut on a chain structured graph."""
    def makeRandom(self, seed):
        """Construct and maxflow on a random chain structured graph."""
        import random
        r = random.Random(seed)
        while True:
            weights = [r.random() for i in xrange(r.randint(5, 30))]
            # Duplication is *extraordinarily* unlikely, but it is
            # easy to check for.
            if len(weights)==len(set(weights)): break
        minpoint = weights.index(min(weights))
        ni=self.graph.add_node(len(weights)-1)
        self.graph.add_tweights(ni, weights[0], 0)
        self.graph.add_tweights(ni+self.graph.num_nodes-1, 0, weights[-1])
        for n, weight in enumerate(weights[1:-1]):
            self.graph.add_edge(ni+n, ni+n+1, weight, 0)
        self.assertAlmostEqual(self.graph.maxflow(), min(weights))

        for i in xrange(minpoint):
            self.assertEqual(self.graph.segment(ni+i), False)
        for i in xrange(minpoint, self.graph.num_nodes):
            self.assertEqual(self.graph.segment(ni+i), True)

    def testChain(self):
        """Check the correctness of a chain structured graph."""
        for i in xrange(100):
            self.graph = Graph()
            self.makeRandom(i)

class GraphMalformedMethodCalls(unittest.TestCase):
    """Tests TypeErrors with argument numbers/types to Graph methods."""
    def setUp(self):
        self.graph = Graph()

    def testAddNodeArgumentNumber(self):
        """Tests add_node called with more than 1 argument."""
        an = self.graph.add_node
        self.assertRaises(TypeError, an, 1, 2)
        self.assertRaises(TypeError, an, 1, 1, 1)

    def testAddNodeArgumentType(self):
        """Tests add_node called with non-integral arguments."""
        an = self.graph.add_node
        self.assertRaises(TypeError, an, 'hiya!')
        self.assertRaises(TypeError, an, '2')
        self.assertRaises(TypeError, an, [1,2,'foo'])
        self.assertRaises(TypeError, an, complex(1,2))

    def testAddEdgeArgumentNumber(self):
        """Tests add_edge called with the wrong number of arguments."""
        ni = self.graph.add_node(2)
        ae = self.graph.add_edge
        self.assertRaises(TypeError, ae)
        self.assertRaises(TypeError, ae, ni, ni+1, 2.0)
        self.assertRaises(TypeError, ae, ni, ni+1, 2.0, 0.0, ni)

    def testAddEdgeArgumentType(self):
        """Tests add_edge called with the wrong types of arguments."""
        ni = self.graph.add_node(2)
        ae = self.graph.add_edge
        self.assertRaises(TypeError, ae, 'foo', ni+1, 0.0, 0.0)
        self.assertRaises(TypeError, ae, ni, [ni+1], 0.0, 0.0)
        self.assertRaises(TypeError, ae, ni, ni+1, complex(2,5), 0.0)
        self.assertRaises(TypeError, ae, ni, ni+1, 0.0, {0:0})

    def testAddTweightsArgumentNumber(self):
        """Tests add_tweights called with a wrong number of arguments."""
        ni = self.graph.add_node()
        at = self.graph.add_tweights
        self.assertRaises(TypeError, at)
        self.assertRaises(TypeError, at, ni)
        self.assertRaises(TypeError, at, ni, 1.0)
        self.assertRaises(TypeError, at, ni, 1.0, 2.0, ni)

    def testAddTweightsArgumentType(self):
        """Tests add_tweights called with wrong types of arguments."""
        ni = self.graph.add_node()
        at = self.graph.add_tweights
        self.assertRaises(TypeError, at, [ni], 2.0, 3.0)
        self.assertRaises(TypeError, at, ni, None, 3.0)
        self.assertRaises(TypeError, at, ni, '5.0', 3.0)
        self.assertRaises(TypeError, at, ni, 2.0, {2:3, 4:5})

    def testAddMaxflowArgumentNumber(self):
        """Tests maxflow called with any arguments."""
        mf = self.graph.maxflow
        self.assertRaises(TypeError, mf, 2)
        self.assertRaises(TypeError, mf, None)
        self.assertRaises(TypeError, mf, 'bar')
        self.assertRaises(TypeError, mf, mf, mf, mf, 5)

    def testSegmentArgumentNumber(self):
        """Tests segment called with other than one argument."""
        seg = self.graph.segment
        self.assertRaises(TypeError, seg)
        self.assertRaises(TypeError, seg, 1, 2)
        self.assertRaises(TypeError, seg, 1, 1, 1)

    def testSegmentArgumentType(self):
        """Tests segment called with non-integral arguments."""
        seg = self.graph.segment
        self.assertRaises(TypeError, seg, 'hiya!')
        self.assertRaises(TypeError, seg, '2')
        self.assertRaises(TypeError, seg, [1,2,'foo'])
        self.assertRaises(TypeError, seg, complex(1,2))

class EnergyVarCreationTestCase(unittest.TestCase):
    """Test cases involving only adding variables."""
    def setUp(self):
        self.energy = Energy()

    def testEmptyVarCount(self):
        """Check that the variable count is initially 0."""
        self.assertEqual(self.energy.num_nodes, 0)

    def testSingleAdditionVarCount(self):
        """Check that the variable count is 1 after a single addition."""
        self.energy.add_variable()
        self.assertEqual(self.energy.num_nodes, 1)

    def testMultipleAdditionVarCount(self):
        """Check variable count across multiple additions."""
        to_add = 1000
        for n in xrange(to_add):
            self.energy.add_variable()
            self.assertEqual(self.energy.num_nodes, n+1)

    def testVariableUniqueness(self):
        """Check uniqueness of created variable IDs."""
        to_add, current_ids = 1000, set()
        for n in xrange(to_add):
            var_id = self.energy.add_variable()
            self.failIf(var_id in current_ids)
            current_ids.add(var_id)

class EnergyConstantTermTestCase(unittest.TestCase):
    """Test cases for constant energy term addition.
 
    These tests use only add_term(E) form of add_term."""
    def setUp(self):
        self.energy = Energy()

    def testNoTerms(self):
        """Test that no term energy minimizes to 0."""
        self.assertAlmostEqual(self.energy.minimize(), 0.0)

    def testConstantTerm(self):
        """Test that constant term add shifts minimization energy."""
        self.energy.add_term(-2.3)
        self.assertAlmostEqual(self.energy.minimize(), -2.3)

    def testTermAdditiveness(self):
        """Test that addition of constant terms is indeed additive."""
        self.energy.add_term(-1.2)
        self.energy.add_term(2.3)
        self.assertAlmostEqual(self.energy.minimize(), 1.1)

class EnergyOneTermTestCase(unittest.TestCase):
    """Test cases for single variable energy term addition.

    These tests use only add_term(E) and add_term(x, E0, E1) form of
    add_term."""
    def setUp(self):
        self.energy = Energy()

    def testSingleVariableFalse(self):
        """Test when a single variable should be false."""
        var_id = self.energy.add_variable()
        self.energy.add_term(var_id, 1.2, 3.4)
        self.assertAlmostEqual(self.energy.minimize(), 1.2)
        self.assertEqual(self.energy.var(var_id), False)

    def testSingleVariableTrue(self):
        """Test when a single variable should be false."""
        var_id = self.energy.add_variable()
        self.energy.add_term(var_id, 1.2, -3.4)
        self.assertAlmostEqual(self.energy.minimize(), -3.4)
        self.assertEqual(self.energy.var(var_id), True)

    def testSingleVariableMultipleAdd(self):
        """Test that add_term on a single variable really is additive."""
        var_id = self.energy.add_variable()
        self.energy.add_term(var_id, 1.2, 0.4)
        self.assertAlmostEqual(self.energy.minimize(), 0.4)
        self.assertEqual(self.energy.var(var_id), True)
        self.energy.add_term(var_id, -1, 0.0)
        self.assertAlmostEqual(self.energy.minimize(), 0.2)
        self.assertEqual(self.energy.var(var_id), False)

    def testManyVariables(self):
        """Test when many variables should be true or false."""
        import random
        r = random.Random(0)
        num_to_add = 100
        var2truth = {}
        global_energy = 0.0
        for n in xrange(num_to_add):
            var_id = self.energy.add_variable()
            false_energy, true_energy = 0.0, 0.0
            while false_energy == true_energy:
                false_energy = r.uniform(-10, 10)
                true_energy = r.uniform(-10, 10)
            global_energy += min(true_energy, false_energy)
            self.energy.add_term(var_id, false_energy, true_energy)
            var2truth[var_id] = true_energy < false_energy
        # Add in a constant term for good measure.
        constant_energy = r.uniform(-100, 100)
        global_energy += constant_energy
        self.energy.add_term(constant_energy)
        # Now, run the minimization and run the tests.
        self.assertAlmostEqual(self.energy.minimize(), global_energy)
        for n, val in var2truth.items():
            self.assertEqual(val, self.energy.var(n))

    def testNonexistentVariables(self):
        """Tests adding terms on absent variables gives errors."""
        # Just have one variable.
        var_id = self.energy.add_variable()
        self.assertRaises(ValueError, self.energy.add_term, var_id-1, 1, 2)
        self.assertRaises(ValueError, self.energy.add_term, var_id+1, -2, 2)

class EnergyTwoTermTestCase(unittest.TestCase):
    """Test cases for single variable energy term addition.

    These tests use all three forms of add_term, that is, add_term(E),
    add_term(x, E0, E1), and add_term(x, y, E00, E01, E10, E11)."""
    def setUp(self):
        self.energy = Energy()

    def testTwoVariableOptimization1(self):
        """Tests a simple optimization involving two variables."""
        e = self.energy
        vara, varb = e.add_variable(), e.add_variable()
        # Make it appear attractive to set vara True, and varb False.
        e.add_term(vara, 5.0, 1.0)
        e.add_term(varb, -2.0, 1.5)
        # Then do something to make it attractive to make both False.
        e.add_term(vara, varb, 1.5, -1.9, 5.55, 2.1)
        self.assertAlmostEqual(e.minimize(), 5.0 - 2.0 + 1.5)
        self.assertEqual(e.var(vara), False)
        self.assertEqual(e.var(varb), False)

    def testTwoVariableOptimization2(self):
        """Tests a simple optimization involving two variables."""
        e = self.energy
        vara, varb = e.add_variable(), e.add_variable()
        # Make it appear attractive to set vara True, and varb False.
        e.add_term(vara, 5.0, 1.0)
        e.add_term(varb, -2.0, 1.5)
        # Then do something to make it attractive to make both True.
        e.add_term(vara, varb, 1.5, -1.9, 5.55, 1.9)
        # Also, test "additive" nature of energy terms.
        e.add_term(vara, varb, 2.3, 2.3, 2.3, 2.3 - 1e-10)
        self.assertAlmostEqual(e.minimize(), 1.0 + 1.5 + 1.9 + 2.3)
        self.assertEqual(e.var(vara), True)
        self.assertEqual(e.var(varb), True)

    def testNonexistentVariables(self):
        """Tests adding terms on absent variables gives errors."""
        # Just have one variable.
        var_id = self.energy.add_variable()
        at = self.energy.add_term
        self.assertRaises(ValueError, at, var_id, var_id-1, 1, 2, 2, 1)
        self.assertRaises(ValueError, at, var_id+1, var_id, -2, 2, -3, 0)

    def testNonsubmodularTerm(self):
        """Tests adding non-submodular terms gives ValueErrors."""
        e = self.energy
        vara, varb = e.add_variable(), e.add_variable()
        at = e.add_term
        self.assertRaises(ValueError, at, vara, varb, 20, 21, 19, 20.2)
        self.assertRaises(ValueError, at, vara, varb, 0, 0, 0, 0.1)
        self.assertRaises(ValueError, at, vara, varb, -5, -6, -7, -7.9)

class EnergyMalformedMethodCalls(unittest.TestCase):
    """Tests TypeErrors with argument numbers/types to Energy methods."""
    def setUp(self):
        self.energy = Energy()

    def testAddVariableArgumentNumber(self):
        """Tests add_variable called with any arguments."""
        av = self.energy.add_variable
        self.assertRaises(TypeError, av, 1)
        self.assertRaises(TypeError, av, 5)
        self.assertRaises(TypeError, av, 'foo')

    def testMinimizeArgumentNumber(self):
        """Tests minimize called with any arguments."""
        m = self.energy.minimize
        self.assertRaises(TypeError, m, 1)
        self.assertRaises(TypeError, m, 5)
        self.assertRaises(TypeError, m, 'foo')

    def testVarArgumentNumber(self):
        """Tests var called with other than one argument."""
        var = self.energy.var
        self.assertRaises(TypeError, var, 1, 2)
        self.assertRaises(TypeError, var)
        self.assertRaises(TypeError, var, 'flip', 'flop')

    def testVarArgumentType(self):
        """Tests var called with non-integral arguments."""
        var = self.energy.var
        self.assertRaises(TypeError, var, 'flip')
        self.assertRaises(TypeError, var, complex(1,2))
        self.assertRaises(TypeError, var, None)

    def testAddTermArgumentNumber(self):
        """Tests add_term called with other than 1, 3, or 6 arguments."""
        at = self.energy.add_term
        self.assertRaises(TypeError, at)
        self.assertRaises(TypeError, at, 0, 1)
        self.assertRaises(TypeError, at, 0, 1, 2, 3)
        self.assertRaises(TypeError, at, 0, 1, 2, 3, 4)
        self.assertRaises(TypeError, at, 0, 1, 2, 3, 4, 5, 6)
        self.assertRaises(TypeError, at, 0, 1, 2, 3, 4, 5, None)

    def testAddTermArgumentType(self):
        """Tests add_term called with arguments of inappropriate type."""
        at = self.energy.add_term
        # One argument form.
        self.assertRaises(TypeError, at, 'foo')
        self.assertRaises(TypeError, at, '0')
        # Three argument form.
        self.assertRaises(TypeError, at, 'foo', 1, 2)
        self.assertRaises(TypeError, at, '0', 1, 2)
        self.assertRaises(TypeError, at, '0', '1', '2')
        self.assertRaises(TypeError, at, 0, '1', 2)
        self.assertRaises(TypeError, at, 0, [1], 2)
        self.assertRaises(TypeError, at, 0, 1, None)
        self.assertRaises(TypeError, at, 0, 1, {2:3})
        # Six argument form.
        self.assertRaises(TypeError, at, 'foo', 1, 2, 3, 4, 5)
        self.assertRaises(TypeError, at, '0', 1, 2, 3, 4, 5)
        self.assertRaises(TypeError, at, '0', '1', '2', 3, 4, 5)
        self.assertRaises(TypeError, at, 0, '1', 2, 3, 4, 5)
        self.assertRaises(TypeError, at, 0, [1], 2, 3, 4, 5)
        self.assertRaises(TypeError, at, 0, 1, '2', 3, 4, 5)
        self.assertRaises(TypeError, at, 0, 1, 2, {3:3}, 4, 5)
        self.assertRaises(TypeError, at, 0, 1, 2, 3, [4], 5)
        self.assertRaises(TypeError, at, 0, 1, 2, 3, 4, self.energy)

if __name__ == '__main__':
    unittest.main()

from graphcut import *;

gc = QPBO(2,1)
gc.add_node(2)
gc.add_term(0,0,-5)
gc.add_term(1,3,6)
gc.add_term(0,1,2,3,4,6)
gc.solve()
gc.compute_weak_persistencies()
print gc.get_label(0)
print gc.get_label(1)



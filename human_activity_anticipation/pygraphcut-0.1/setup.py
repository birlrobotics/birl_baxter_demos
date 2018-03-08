# Copyright 2007 Thomas Finley, tfinley@gmail.com

from distutils.core import setup, Extension
from os.path import join
import sys

srcdir = 'src'
prefix = 'outside'
outside_files = ['graph.cpp', 'maxflow.cpp', 'energy.cpp','QPBO.cpp', 'QPBO_maxflow.cpp', 'QPBO_maxflow.cpp', 'QPBO_postprocessing.cpp' , 'QPBO_extra.cpp']
interface_files = ['interface.cpp', 'qpboobj.cpp' ]#'graphobj.cpp', 'energyobj.cpp','qpboobj.cpp']

# Define extra objects.
extra_obs = []
if sys.platform.find('darwin') != -1:
    # OS X's lazy linking makes some static initializers not work if we
    # link against STD C++ dynamically.  I'd love to find an alternate
    # solution to this problem.
    #extra_obs.append('/usr/lib/libstdc++.6.dylib')
    pass

module1 = Extension(
    'graphcut',
    sources = [join(srcdir,f) for f in interface_files]+[
    join(srcdir, prefix, o) for o in outside_files],
    include_dirs = [join(srcdir,prefix)],
    libraries = ['stdc++'],
    # The following line seems necessary in OS X for some reason.
    extra_objects = extra_obs)

ld = """The %s module gives one access to the functionality of
the graph cut minimization software of Boykov and Kolmogorov."""

setup(name = module1.name, version = '0.1', ext_modules = [module1],
      description = 'Graph cut energy minimization software.',
      long_description = ld % module1.name,
      author = 'Thomas Finley',
      author_email = 'tfinley@gmail.com',
      url = 'http://tfinley.net/software/pygraphcut/',
      license = 'Research Only',
      classifiers = [
    'Development Status :: 3 - Alpha',
    'Intended Audience :: Science/Research',
    'Programming Language :: C',
    'Programming Language :: Python',
    'Operating System :: POSIX',
    'Operating System :: MacOS :: MacOS X',
    'Topic :: Scientific/Engineering :: Mathematics',
    'Topic :: Software Development :: Libraries :: Python Modules' ]
      )

                               PyGraphcut Readme

   Copyright (c) 2007, Thomas W. Finley

Overview

   PyGraphcut is a Python module which encapsulates the functionality of the
   MAXFLOW graphcut code of Boykov and Kolmogorov. This encapsulated code
   allows one to specify graph cut maxflow/mincut problems, and the related
   quadratic pseudo-Boolean optimization problems, and PyGraphcut provides a
   convenient Python interface atop that.

   This software is licensed under the exact same terms as MAXFLOW, which
   typically means usage is restricted for research only. See the HTML
   documents on licensing for more precise information.

Availability

   To get the lastest version, see:
   http://tfinley.net/software/pygraphcut/

Documentation

   The HTML documentation included with the release in the directory html
   contains information on building, testing, installation and documentation
   of all features of the module.

Building and Installing

   The module builds and appears to work on my simple test files in Python
   2.3, 2.4, and 2.5. Earlier versions of Python will not work.

   Ideally, the following will work in creating a graphcut module importable
   from Python:

     * make
     * make test
     * make install

   See the HTML documentation on building for trouble shooting information.

Bugs and Commentary

   Please send information on issues of usage to Thomas Finley at
   tfinley@gmail.com .

     ----------------------------------------------------------------------

   Thomas Finley, 2007

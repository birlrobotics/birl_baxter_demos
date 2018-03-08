from distutils.core import setup, Extension
import sys, os, os.path, re

useparams = False

sources = 'glpk lp barcol bar obj util kkt tree environment'
source_roots = sources.split()
if useparams: source_roots.append('params')

# This build process will not work with anything prior to GLPK 4.16,
# since there were many notable changes in GLPK including,
# importantly, something which actually contains the version number.

libdirs, incdirs, extraobs = [], [], []

# The glpver argument is one which is used only for the purposes of
# PyGLPK development, and will be of no use or interest to the
# standard practitioner.  In order to assure compatibility with the
# many GLPK versions which exist, it is helpful to build against one
# of many

# This is very dirty.
m = re.match('glpver=(\d+)', sys.argv[-1])
if m:
    # We have defined that we want to build to a local GLPK version.
    minor_version = int(m.group(1))
    assert minor_version >= 16
    sys.argv = sys.argv[:-1]
    libdirs.append(os.path.join('locals', '4.%d'%minor_version, 'lib'))
    incdirs.append(os.path.join('locals', '4.%d'%minor_version, 'include'))
    if minor_version<37:
        libs = ['glpk.0.%d.0'%(minor_version-15)]
    else:
        libs = ['glpk.0']
    print (libdirs, incdirs)
else:
    # Try to get which is the executable path, and infer additional
    # library and include directories from there, based on a call to
    # whatever glpsol we find.
    glpsol_path = os.popen('which glpsol').read().strip()
    # If we can't find it, just hope that the default libs are correct.
    if glpsol_path:
        glpsol_path = os.path.abspath(glpsol_path)
        head, tail = os.path.split(glpsol_path)
        head, tail = os.path.split(head)
        libdirs.append(os.path.join(head, 'lib'))
        incdirs.append(os.path.join(head, 'include'))

# USERS DO CUSTOM INSTRUCTIONS HERE
# Perhaps set your libdir manually in case neither system defaults,
# nor the cleverness does not work.

#libs = ['glpk.something']
#libdirs = ['/my/dirs/are/here/lib']
#incdirs = ['/my/dirs/are/here/include']

# If the user did not define libraries themselves, set that up.  We
# require both glpk and gmp.
try:
    libs
except NameError:
    # The user nor test code did not set libs up yet.
    libs = ['glpk', 'gmp']

incdirs.append('src')

macros = []
if useparams: macros.append(('USEPARAMS', None))

# Now, finally, define that module!
module1 = Extension(
    'glpk',
    sources = [os.path.join('src',r+'.c') for r in source_roots],
    define_macros = macros, extra_compile_args=['-m64'], extra_link_args=['-m64'],
    library_dirs = libdirs, include_dirs = incdirs,
    libraries = libs, extra_objects = extraobs)

ld = """The PyGLPK module gives one access to the functionality
of the GNU Linear Programming Kit.  
"""

setup(name = 'glpk',
      version = '0.3',
      description = 'PyGLPK, a Python module encapsulating GLPK.',
      long_description = ld,
      author = 'Thomas Finley',
      author_email = 'tfinley@gmail.com',
      url = 'http://tfinley.net/software/pyglpk/',
      license = 'GPL',
      classifiers = [
    'Development Status :: 3 - Alpha',
    'Intended Audience :: Science/Research',
    'License :: OSI Approved :: GNU General Public License (GPL)',
    'Programming Language :: C',
    'Programming Language :: Python',
    'Operating System :: POSIX',
    'Operating System :: MacOS :: MacOS X',
    'Topic :: Scientific/Engineering :: Mathematics',
    'Topic :: Software Development :: Libraries :: Python Modules' ],
      ext_modules = [module1])

from distutils.core import setup, Extension
import sys, os, os.path, glob

# Come up with the list of files to compile.
ext_files  = glob.glob('*.c')
ext_files.extend(os.path.join('..','svm_light',  '%s.c'%b) for b in [
    'svm_common', 'svm_learn', 'svm_hideo'])
ext_files.extend(os.path.join('..','svm_struct', '%s.c'%b) for b in [
    'svm_struct_common', 'svm_struct_learn'])
ext_files.append(os.path.join('..','svm_struct_api.c'))

libdirs, incdirs = [], []

#incdirs.append('src')

# Now, finally, define that module!
module1 = Extension(
    'svmapi',
    sources = ext_files,
    library_dirs = libdirs, include_dirs = incdirs)

ld = """The SVM-api module is an interface for SVM-struct.
"""

setup(name = 'svmapi',
      version = '0.2',
      description = 'SVM-api, a Python module for SVM-struct.',
      long_description = ld,
      author = 'Thomas Finley',
      author_email = 'tfinley@gmail.com',
      url = 'http://tfinley.net/software/svmpython2/',
      license = 'Research only',
      classifiers = [
    'Development Status :: 3 - Alpha',
    'Intended Audience :: Science/Research',
    'License :: Research Only',
    'Programming Language :: C',
    'Programming Language :: Python',
    'Operating System :: POSIX',
    'Operating System :: MacOS :: MacOS X',
    'Topic :: Scientific/Engineering :: Mathematics',
    'Topic :: Software Development :: Libraries :: Python Modules' ],
      ext_modules = [module1])

#!/usr/bin/env python
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml                                                    
# setup_args = generate_distutils_setup(
#     packages=['baxter_examples','baxter_external_devices'],
#     package_dir={'': 'src'})

# setup(**setup_args)


d = generate_distutils_setup()
d['packages'] = ['pa_demo']
d['package_dir'] = {'': 'src'}

#d['scripts'] = ['gripper_action_client']
#d['scripts_dir']={'':'scripts'}

setup(**d)

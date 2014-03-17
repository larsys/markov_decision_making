from distutils.core import setup

setup(
    version='0.1.2',
    scripts=['VisGUI.py'],
    packages=['mdm_library'],
    package_dir={'': 'src'}
)


#from distutils.core import setup
#from catkin_pkg.python_setup import generate_distutils_setup

#d = generate_distutils_setup(
#    packages=['mdm_library'],
#    scripts=['VisGUI.py'],
#    package_dir={'': 'src'}
#)

#setup(**d)

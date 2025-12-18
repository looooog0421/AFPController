## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['motor_control'],
    package_dir={'': 'src'}         # 告诉它包在 src 目录下
)

setup(**setup_args)
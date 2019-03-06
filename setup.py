from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup()
d['packages'] = ['autostep_proxy','autostep_proxy_gui']
d['package_dir'] = {'': 'src'}
setup(**d)

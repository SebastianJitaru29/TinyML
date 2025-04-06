import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/nero/braccio_ws_fresh/install/braccio_hardware_interface'

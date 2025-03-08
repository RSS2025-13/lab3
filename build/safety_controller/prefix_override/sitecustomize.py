import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/racecar/racecar_ws/src/team13/lab3/install/safety_controller'

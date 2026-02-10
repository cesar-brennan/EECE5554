import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/bcesar/Documents/Git/EECE5554/LAB1/install/gps_driver'

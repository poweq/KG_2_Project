import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/jucy/ydlidar_ws_11_29/ydlidar_ws/install/jdamr200_node'

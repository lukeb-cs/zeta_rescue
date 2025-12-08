import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/fauntlht/dev_ws/src/CS354-Final-Project_Luke-Joshua-Jessica-Hunter/install/final_project'

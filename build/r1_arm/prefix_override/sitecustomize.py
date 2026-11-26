import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/wangwenyu/桌面/r1_workspace/install/r1_arm'

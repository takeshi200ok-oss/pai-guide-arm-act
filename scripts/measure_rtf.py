"""/clock を数秒購読してシミュレータの実時間比（RTF）を測る"""
import sys, time
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
rclpy.init(); n = Node("measure_rtf"); s = []
n.create_subscription(Clock, "/clock", lambda m: s.append((time.monotonic(), m.clock.sec + m.clock.nanosec * 1e-9)), 10)
dur = float(sys.argv[1]) if len(sys.argv) > 1 else 5.0
t = time.monotonic()
while time.monotonic() - t < dur:
    rclpy.spin_once(n, timeout_sec=0.1)
print("RTF=%.3f" % ((s[-1][1] - s[0][1]) / (s[-1][0] - s[0][0])))

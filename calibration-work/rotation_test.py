import os,sys,time,json,math,signal,subprocess
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu,LaserScan
from pathlib import Path
root=Path(__file__).resolve().parent
heartbeat=root/'heartbeat.json'
rclpy.init(); n=Node('rotation_watchdog' if '--watchdog' in sys.argv else 'rotation_calibration'); pub=n.create_publisher(Twist,'/cmd_vel',10)
if '--watchdog' in sys.argv:
 deadline=time.monotonic()+90
 while time.monotonic()<deadline:
  rclpy.spin_once(n,timeout_sec=.05)
  try:h=json.loads(heartbeat.read_text())
  except Exception:continue
  if h['done'] or time.monotonic()-h['time']>.8:
   for _ in range(20):pub.publish(Twist());rclpy.spin_once(n,timeout_sec=.05)
   break
 n.destroy_node();rclpy.shutdown();sys.exit()
name=sys.argv[1] if len(sys.argv)>1 else 'baseline'
speed=float(sys.argv[2]) if len(sys.argv)>2 else .15
duration=float(sys.argv[3]) if len(sys.argv)>3 else 10
assert 0<speed<=.25 and 0<duration<=40
records={'odom':[],'imu':[],'scan':[],'command':[]}; last={}; start=time.monotonic()
def stamp(m):return m.header.stamp.sec+m.header.stamp.nanosec*1e-9
def odom(m):
 t=time.monotonic();last['odom']=t;q=m.pose.pose.orientation
 records['odom'].append([t,stamp(m),m.pose.pose.position.x,m.pose.pose.position.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)),m.twist.twist.linear.x,m.twist.twist.angular.z])
def imu(m):
 t=time.monotonic();last['imu']=t;records['imu'].append([t,stamp(m),m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z])
def scan(m):
 t=time.monotonic();last['scan']=t;r=np.array(m.ranges);a=m.angle_min+np.arange(len(r))*m.angle_increment;valid=np.isfinite(r)&(r>.25)&(r<8)
 points=np.column_stack([r[valid]*np.cos(a[valid]),r[valid]*np.sin(a[valid])])
 records['scan'].append([t,stamp(m),points.tolist()])
subs=[n.create_subscription(Odometry,'/odom',odom,100),n.create_subscription(Imu,'/imu/data_raw',imu,100),n.create_subscription(LaserScan,'/scan',scan,30)]
def beat(done=False):
 temp=heartbeat.with_suffix('.tmp');temp.write_text(json.dumps({'time':time.monotonic(),'done':done}));temp.replace(heartbeat)
beat();watch=subprocess.Popen([sys.executable,__file__,'--watchdog'],stdout=open(root/(name+'-watchdog.log'),'w'),stderr=subprocess.STDOUT)
def stop_signal(*_):raise KeyboardInterrupt()
signal.signal(signal.SIGTERM,stop_signal)
try:
 for label,w,seconds in [('stationary_before',0,5),('ccw',speed,duration),('stationary_middle',0,4),('cw',-speed,duration),('stationary_after',0,4)]:
  print(label,w,seconds,flush=True);end=time.monotonic()+seconds;next_pub=0
  while time.monotonic()<end:
   now=time.monotonic();beat()
   if w and (set(last)!=set(['odom','imu','scan']) or any(now-v>.7 for v in last.values())):raise RuntimeError('Feedback missing/stale: stopping')
   if now>=next_pub:
    msg=Twist();msg.angular.z=float(w);pub.publish(msg);records['command'].append([now,w,label]);next_pub=now+.05
   rclpy.spin_once(n,timeout_sec=.01)
except BaseException as e:print('TEST INTERRUPTED',repr(e),flush=True)
finally:
 for _ in range(30):
  pub.publish(Twist());beat();rclpy.spin_once(n,timeout_sec=.05)
 beat(True);watch.wait(timeout=5);(root/(name+'.json')).write_text(json.dumps(records));print('SAVED',name,{k:len(v) for k,v in records.items()},flush=True);n.destroy_node();rclpy.shutdown()

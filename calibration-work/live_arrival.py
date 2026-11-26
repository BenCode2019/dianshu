import rclpy,time,math,json
from pathlib import Path
from rclpy.node import Node
from geometry_msgs.msg import Twist,PoseWithCovarianceStamped
from nav_msgs.msg import Odometry,Path as RosPath
from rcl_interfaces.msg import Log
from tf2_ros import Buffer,TransformListener
rclpy.init();n=Node('live_arrival_observer');buf=Buffer();listener=TransformListener(buf,n);rows=[];latest={};goal=None;subs=[]
def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def cmd(m,t):latest[t]=[m.linear.x,m.angular.z];rows.append([time.monotonic(),t,m.linear.x,m.angular.z])
def plan(m):
 global goal
 if m.poses:
  p=m.poses[-1].pose;goal=(p.position.x,p.position.y,yaw(p.orientation),m.header.frame_id)
def amcl(m):rows.append([time.monotonic(),'amcl',m.pose.pose.position.x,m.pose.pose.position.y,yaw(m.pose.pose.orientation),m.pose.covariance[0],m.pose.covariance[7],m.pose.covariance[35]])
for t in ['/cmd_vel_nav','/cmd_vel']:subs.append(n.create_subscription(Twist,t,lambda m,t=t:cmd(m,t),100))
subs.append(n.create_subscription(Odometry,'/odom',lambda m:cmd(m.twist.twist,'odom'),100));subs.append(n.create_subscription(RosPath,'/plan',plan,10));subs.append(n.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',amcl,20));subs.append(n.create_subscription(Log,'/rosout',lambda m:rows.append([time.monotonic(),'log',m.name,m.msg]) if m.level>=30 else None,100))
end=time.monotonic()+18;nextprint=0
while time.monotonic()<end:
 rclpy.spin_once(n,timeout_sec=.02);now=time.monotonic()
 if now>nextprint:
  nextprint=now+.5
  try:
   t=buf.lookup_transform('map','base_link',rclpy.time.Time()).transform;x=t.translation.x;y=t.translation.y;a=yaw(t.rotation);err=None if goal is None else [math.hypot(goal[0]-x,goal[1]-y),math.atan2(math.sin(goal[2]-a),math.cos(goal[2]-a))]
   row=[now,'state',x,y,a,goal,err,dict(latest)];rows.append(row);print(json.dumps(row),flush=True)
  except Exception as e:print(str(e),flush=True)
Path('calibration-work/live-arrival.json').write_text(json.dumps(rows));n.destroy_node();rclpy.shutdown()

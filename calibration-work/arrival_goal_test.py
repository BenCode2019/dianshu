import rclpy,time,math,json,signal
from pathlib import Path
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist,PoseWithCovarianceStamped
from nav_msgs.msg import Odometry,Path as RosPath
from nav2_msgs.action import NavigateToPose
from rcl_interfaces.msg import Log
from tf2_ros import Buffer,TransformListener
rclpy.init();n=Node('arrival_goal_test');buf=Buffer();listener=TransformListener(buf,n);client=ActionClient(n,NavigateToPose,'navigate_to_pose');rows=[];latest={};subs=[];gh=None;finished=False;pub=n.create_publisher(Twist,'/cmd_vel',10)
def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def record(k,*v):rows.append([time.monotonic(),k,*v])
def cmd(m,k):latest[k]=[m.linear.x,m.angular.z];record(k,m.linear.x,m.angular.z)
for t in ['/cmd_vel_nav','/cmd_vel','/cmd_vel_raw']:subs.append(n.create_subscription(Twist,t,lambda m,t=t:cmd(m,t),100))
subs.append(n.create_subscription(Odometry,'/odom',lambda m:(cmd(m.twist.twist,'odom'),record('odom_pose',m.pose.pose.position.x,m.pose.pose.position.y,yaw(m.pose.pose.orientation))),100))
subs.append(n.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',lambda m:record('amcl',m.pose.pose.position.x,m.pose.pose.position.y,yaw(m.pose.pose.orientation),m.pose.covariance[0],m.pose.covariance[7],m.pose.covariance[35]),50))
subs.append(n.create_subscription(RosPath,'/plan',lambda m:record('plan',[[p.pose.position.x,p.pose.position.y] for p in m.poses]),10))
subs.append(n.create_subscription(Log,'/rosout',lambda m:record('log',m.level,m.name,m.msg) if m.level>=20 and any(x in m.name for x in ['controller','navigator','amcl','behavior','planner','velocity']) else None,100))
def spin_until(f,seconds):
 end=time.monotonic()+seconds
 while rclpy.ok() and not f.done() and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.02)
 if not f.done():raise RuntimeError('Action response timed out')
 return f.result()
def interrupt(*a):raise KeyboardInterrupt()
signal.signal(signal.SIGTERM,interrupt)
try:
 if not client.wait_for_server(timeout_sec=10):raise RuntimeError('Navigation server unavailable')
 end=time.monotonic()+10
 while True:
  rclpy.spin_once(n,timeout_sec=.05)
  try:t=buf.lookup_transform('map','base_link',rclpy.time.Time());break
  except Exception:
   if time.monotonic()>end:raise RuntimeError('No map to base_link transform')
 start=(t.transform.translation.x,t.transform.translation.y,yaw(t.transform.rotation));gx=start[0]+.30*math.cos(start[2]);gy=start[1]+.30*math.sin(start[2]);ga=math.atan2(math.sin(start[2]+math.pi),math.cos(start[2]+math.pi));goal=NavigateToPose.Goal();goal.pose.header.frame_id='map';goal.pose.header.stamp=n.get_clock().now().to_msg();goal.pose.pose.position.x=gx;goal.pose.pose.position.y=gy;goal.pose.pose.orientation.z=math.sin(ga/2);goal.pose.pose.orientation.w=math.cos(ga/2)
 record('goal',start,[gx,gy,ga]);print('GOAL',json.dumps({'start':start,'target':[gx,gy,ga]}),flush=True)
 def feedback(m):
  f=m.feedback;record('feedback',float(f.distance_remaining),int(f.number_of_recoveries))
 gh=spin_until(client.send_goal_async(goal,feedback_callback=feedback),10)
 if not gh.accepted:raise RuntimeError('Goal rejected')
 print('ACCEPTED',flush=True);result=gh.get_result_async();end=time.monotonic()+65;nextprint=0
 while not result.done() and time.monotonic()<end:
  rclpy.spin_once(n,timeout_sec=.02);now=time.monotonic()
  try:
   t=buf.lookup_transform('map','base_link',rclpy.time.Time()).transform;x=t.translation.x;y=t.translation.y;a=yaw(t.rotation);err=[math.hypot(gx-x,gy-y),math.atan2(math.sin(ga-a),math.cos(ga-a))]
   if math.hypot(x-start[0],y-start[1])>1.0:raise RuntimeError('Exceeded test travel boundary')
   if now>nextprint:
    nextprint=now+.5;row=[x,y,a,err,dict(latest)];record('state',*row);print(json.dumps(row),flush=True)
  except RuntimeError:raise
  except Exception:pass
 if result.done():
  res=result.result();record('result',res.status,str(res.result));print('RESULT',res.status,str(res.result),flush=True);finished=True
 else:print('TIMEOUT: cancelling test goal',flush=True)
except BaseException as e:print('TEST INTERRUPTED',repr(e),flush=True);record('exception',repr(e))
finally:
 if gh is not None and not finished:
  try:spin_until(gh.cancel_goal_async(),5)
  except Exception as e:print('Cancel:',str(e),flush=True)
  for _ in range(25):pub.publish(Twist());rclpy.spin_once(n,timeout_sec=.04)
 end=time.monotonic()+2
 while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.02)
 Path('calibration-work/arrival-goal-test.json').write_text(json.dumps(rows));print('SAVED',len(rows),flush=True);n.destroy_node();rclpy.shutdown()

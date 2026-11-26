import json,time,rclpy
from pathlib import Path
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
rclpy.init();n=Node('restore_navigation_pose');p=n.create_publisher(PoseWithCovarianceStamped,'/initialpose',10);x=json.loads(Path('calibration-work/navigation-restart-state.json').read_text());m=PoseWithCovarianceStamped();m.header.frame_id='map';m.pose.pose.position.x=x['x'];m.pose.pose.position.y=x['y'];m.pose.pose.orientation.z=x['z'];m.pose.pose.orientation.w=x['w'];m.pose.covariance[0]=.01;m.pose.covariance[7]=.01;m.pose.covariance[35]=.01
end=time.monotonic()+8
while p.get_subscription_count()<1 and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.1)
assert p.get_subscription_count()>0,'AMCL unavailable'
for _ in range(3):m.header.stamp=n.get_clock().now().to_msg();p.publish(m);rclpy.spin_once(n,timeout_sec=.2)
n.destroy_node();rclpy.shutdown();print('Restored saved map pose')

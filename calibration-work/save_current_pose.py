import rclpy,time,json
from pathlib import Path
from rclpy.node import Node
from tf2_ros import Buffer,TransformListener
rclpy.init();n=Node('save_current_navigation_pose');b=Buffer();l=TransformListener(b,n);end=time.monotonic()+8
while True:
 rclpy.spin_once(n,timeout_sec=.05)
 try:t=b.lookup_transform('map','base_footprint',rclpy.time.Time()).transform;break
 except Exception:
  if time.monotonic()>end:raise RuntimeError('No current pose')
x={'x':t.translation.x,'y':t.translation.y,'z':t.rotation.z,'w':t.rotation.w};Path('calibration-work/navigation-restart-state.json').write_text(json.dumps(x));print('Saved current map pose',x);n.destroy_node();rclpy.shutdown()

import sys,json,math
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
root=Path(__file__).resolve().parent
name=sys.argv[1] if len(sys.argv)>1 else 'baseline';d=json.loads((root/(name+'.json')).read_text());scans=d['scan'];o=np.array(d['odom']);im=np.array(d['imu']);cmd=d['command']
def icp(p,q,initial=None):
 tree=cKDTree(q);R=np.eye(2) if initial is None else initial[:2,:2].copy();t=np.zeros(2) if initial is None else initial[:2,2].copy()
 for _ in range(40):
  x=p@R.T+t;dist,idx=tree.query(x);mask=dist<min(.20,np.quantile(dist,.8));a=x[mask];b=q[idx[mask]]
  if len(a)<70:raise RuntimeError('Too few scan matches')
  ac=a.mean(0);bc=b.mean(0);u,s,vt=np.linalg.svd((a-ac).T@(b-bc));r=vt.T@u.T
  if np.linalg.det(r)<0:vt[-1]*=-1;r=vt.T@u.T
  v=bc-r@ac;R=r@R;t=r@t+v
  if np.linalg.norm(v)<1e-5 and abs(math.atan2(r[1,0],r[0,0]))<1e-5:break
 T=np.eye(3);T[:2,:2]=R;T[:2,2]=t
 return T,float(np.sqrt(np.mean(dist[mask]**2))),float(mask.mean())
def angle(T):return math.atan2(T[1,0],T[0,0])
stationary=[x[0] for x in cmd if x[2]=='stationary_before'];bias=im[(im[:,0]>stationary[0]+1)&(im[:,0]<stationary[-1]),2:].mean(0)
print('GYRO stationary bias XYZ rad/s',bias.tolist());results=[]
for phase,post in [('ccw','stationary_middle'),('cw','stationary_after')]:
 c=[x for x in cmd if x[2]==phase];a=c[0][0];b=c[-1][0]+.05
 before=[s for s in scans if a-.6<s[0]<a-.1];after=[s for s in scans if b+.8<s[0]<b+1.5];first=before[len(before)//2];last=after[len(after)//2]
 section=[s for s in scans if first[0]<=s[0]<=last[0]];total=np.eye(3);rms=[]
 for old,new in zip(section,section[1:]):
  T,e,ratio=icp(np.array(new[2]),np.array(old[2]));total=total@T;rms.append(e)
 m=(im[:,0]>=first[0])&(im[:,0]<=last[0]);ga=np.trapz(im[m,4]-bias[2],im[m,0]);initial=np.eye(3);initial[:2,:2]=[[math.cos(ga),-math.sin(ga)],[math.sin(ga),math.cos(ga)]]
 T,e,ratio=icp(np.array(last[2]),np.array(first[2]),initial);la=angle(T);la+=2*math.pi*round((ga-la)/(2*math.pi))
 oa=np.interp(last[0],o[:,0],np.unwrap(o[:,4]))-np.interp(first[0],o[:,0],np.unwrap(o[:,4]));m=(im[:,0]>=first[0])&(im[:,0]<=last[0]);ga=np.trapz(im[m,4]-bias[2],im[m,0]);om=(o[:,0]>=a+1)&(o[:,0]<=b-.5)
 result=dict(phase=phase,command=c[0][1],duration=b-a,lidar_deg=math.degrees(la),odom_deg=math.degrees(oa),gyro_deg=math.degrees(ga),odom_to_lidar=la/oa,command_to_lidar=la/(c[0][1]*(b-a)),scan_rms=e,scan_inliers=ratio,incremental_deg=math.degrees(angle(total)),odom_speed_mean=float(o[om,6].mean()),translation=T[:2,2].tolist())
 print(json.dumps(result));results.append(result)
(root/(name+'-analysis.json')).write_text(json.dumps({'gyro_bias':bias.tolist(),'results':results},indent=2))

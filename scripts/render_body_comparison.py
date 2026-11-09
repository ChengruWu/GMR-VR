import json, sys, time, collections, bisect, subprocess
import imageio_ffmpeg
from pathlib import Path
import numpy as np
import cv2, mujoco
ROOT=Path(__file__).resolve().parents[2]
EVA=ROOT/'eva-mobile'; SESSION=EVA/'work_dirs/body_session_20261109_1433'
sys.path.insert(0,str(EVA/'src'))
import robots
from core.registry import ROBOT_REGISTRY
from eva_body_contract import parse_body_observation, RobotFeedback, GmrRequest, ordered_targets
from gmr_body import create_backend

out=ROOT/'GMR-VR'/'work_dirs'/'body_session_20261109_1433'
out.mkdir(parents=True, exist_ok=True)
controls=[json.loads(s) for s in (SESSION/'control.jsonl').open(encoding='utf8')]
needed={c['source']['frame_seq'] for c in controls}
frames={}
for s in (SESSION/'vr_capture.jsonl').open(encoding='utf8'):
 d=json.loads(s); f=d.get('frame',{})
 if f.get('seq') in needed: frames[f['seq']]=f
assert len(frames)==len(needed), (len(frames),len(needed))
backend=create_backend()
backend.config.update(allow_unvalidated_output=True,relative_task_alignment=True,relative_position_scale=0.8,max_solve_s=10,max_input_age_s=10,max_feedback_age_s=10)
results=[]; counts=collections.Counter(); last=None
for c in controls:
 f=frames[c['source']['frame_seq']]; now=time.monotonic_ns()
 ctrl=f['controllers']; status='waiting for grip'; reason=''
 if all(ctrl[h]['valid'] and ctrl[h]['grip_engaged'] for h in ('left','right')):
  obs=parse_body_observation(f['body'],session_id=f['session_id'],frame_seq=f['seq'],received_monotonic_ns=now)
  r=backend.step(GmrRequest(obs,RobotFeedback(tuple(c['feedback_qpos']),now),ctrl))
  status=r.status; reason=r.reason
  target=ordered_targets(r)
  if target is not None: last=list(target)
 counts[status]+=1
 results.append(dict(seq=f['seq'],status=status,reason=reason,offline_qpos=last if last is not None else c['feedback_qpos']))
print('offline results',dict(counts),flush=True)
(out/'offline_results.json').write_text(json.dumps({'method':'GMR with recorded feedback at each control cycle, target held on idle','config':{'relative_task_alignment':True,'relative_position_scale':0.8},'counts':counts,'cycles':results},ensure_ascii=False,indent=2),encoding='utf8')
model=mujoco.MjModel.from_xml_path(str(EVA/'examples/hardware/gento_luna/scenes/box_task.xml'))
data=mujoco.MjData(model); robot=ROBOT_REGISTRY.build('gento_luna')
addresses=[int(model.jnt_qposadr[model.joint(n).id]) for g in robot.actuator_groups for n in g.joint_names]
assert len(addresses)==24
model.vis.global_.offheight=520; model.vis.global_.offwidth=480
renderer=mujoco.Renderer(model,height=520,width=480)
cam=mujoco.MjvCamera(); cam.type=mujoco.mjtCamera.mjCAMERA_FREE; cam.lookat[:]=(0,0,0.9); cam.distance=3.0; cam.azimuth=135; cam.elevation=-15
edges=[(0,1),(0,2),(0,3),(1,4),(4,7),(7,10),(2,5),(5,8),(8,11),(3,6),(6,9),(9,12),(12,15),(9,13),(13,16),(16,18),(18,20),(20,22),(9,14),(14,17),(17,19),(19,21),(21,23)]
# XR axes are converted with the configured GMR axis map: (-z, -x, y).
# Robot forward is +X and right is -Y. Use a -45 degree azimuth and
# 15 degree downward camera angle, in orthographic view.
axis=np.asarray(backend.config['axis_map'], dtype=float)
first=frames[controls[0]['source']['frame_seq']]['body']['joints']
origin=axis @ np.asarray(first[0]['position'],dtype=float)
az=np.deg2rad(-45); elev=np.deg2rad(15)
view_right=np.array([-np.sin(az),np.cos(az),0.0])
view_up=np.array([-np.sin(elev)*np.cos(az),-np.sin(elev)*np.sin(az),np.cos(elev)])
def skeleton(f):
 im=np.full((520,480,3),(246,246,243),np.uint8); js=f['body']['joints']; pts=[]
 for j in js:
  p=j.get('position')
  if p is None:
   pts.append(None)
  else:
   v=axis @ np.asarray(p,dtype=float)-origin
   pts.append((round(240+v @ view_right*220),round(300-v @ view_up*220)))
 for a,b in edges:
  if pts[a] and pts[b]: cv2.line(im,pts[a],pts[b],(100,115,125),4,cv2.LINE_AA)
 for i,p in enumerate(pts):
  if p: cv2.circle(im,p,6,(40,128,230) if 'left' in js[i]['name'] else ((200,105,44) if 'right' in js[i]['name'] else (70,70,70)),-1,cv2.LINE_AA)
 return im

def render(q):
 data.qpos[addresses]=q; mujoco.mj_forward(model,data); renderer.update_scene(data,camera=cam)
 return cv2.cvtColor(renderer.render(),cv2.COLOR_RGB2BGR)

def txt(img,text,x,y,scale=.58,color=(225,230,235)):
 cv2.putText(img,text,(x,y),cv2.FONT_HERSHEY_SIMPLEX,scale,color,1,cv2.LINE_AA)
fps=20; start=controls[0]['time']; times=[c['time']-start for c in controls]; duration=times[-1]; n=int(duration*fps)+1
video=out/'body_comparison.mp4'; writer=cv2.VideoWriter(str(video),cv2.VideoWriter_fourcc(*'mp4v'),fps,(1440,640))
assert writer.isOpened()
for k in range(n):
 t=k/fps; i=max(0,bisect.bisect_right(times,t)-1); c=controls[i]; f=frames[c['source']['frame_seq']]; r=results[i]
 canvas=np.full((640,1440,3),(30,27,25),np.uint8)
 canvas[65:585,0:480]=skeleton(f); canvas[65:585,480:960]=render(c['feedback_qpos']); canvas[65:585,960:1440]=render(r['offline_qpos'])
 txt(canvas,'VR BODY | RIGHT-FRONT 3D',18,38,.7); txt(canvas,'RECORDED ROBOT | FEEDBACK',498,38,.65); txt(canvas,'OFFLINE GMR | TARGET',978,38,.7)
 txt(canvas,f't = {t:05.2f}s   frame {f["seq"]}',18,617); txt(canvas,f'control: {c["kind"]}  |  feedback before publish',498,617,.49); txt(canvas,f'GMR: {r["status"]}  |  recorded feedback input',978,617,.48)
 writer.write(canvas)
 if k%100==0: print(f'render {k}/{n}',flush=True)
writer.release(); renderer.close()
# VS Code's video preview supports H.264; OpenCV's mp4v encoder does not.
compatible_video=out/'body_comparison_h264.mp4'
subprocess.run([imageio_ffmpeg.get_ffmpeg_exe(), '-y', '-loglevel', 'error',
                '-i', str(video), '-c:v', 'libx264', '-pix_fmt', 'yuv420p',
                '-crf', '18', '-movflags', '+faststart', '-an', str(compatible_video)], check=True)
print('saved', compatible_video, 'frames', n, 'duration', duration, flush=True)






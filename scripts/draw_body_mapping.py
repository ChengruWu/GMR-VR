from PIL import Image, ImageDraw, ImageFont
from pathlib import Path
out=Path('work_dirs/body_session_20261109_1433/joint_mapping.png')
im=Image.new('RGB',(2000,1550),'#f4f6f9'); d=ImageDraw.Draw(im)
font='C:/Windows/Fonts/msyh.ttc'; bold='C:/Windows/Fonts/msyhbd.ttc'
def F(n,b=False): return ImageFont.truetype(bold if b else font,n)
def text(x,y,s,size=28,color='#263342',bold=False): d.text((x,y),s,font=F(size,bold),fill=color)
def line(points,color,w=8): d.line(points,fill=color,width=w,joint='curve')
def dot(x,y,c,r=10): d.ellipse((x-r,y-r,x+r,y+r),fill=c,outline='white',width=3)
def badge(x,y,n,c):
 d.ellipse((x-18,y-18,x+18,y+18),fill=c,outline='white',width=3)
 s=str(n); box=d.textbbox((0,0),s,font=F(18,True)); d.text((x-(box[2]-box[0])/2,y-14),s,font=F(18,True),fill='white')
text(75,35,'PICO Body → Gento Luna：当前 GMR 映射',47,bold=True)
text(78,105,'基于 2026-11-09 14:33 录制的实验配置  ·  正面示意图  ·  左右为人体 / 机器人自身的左右',26,color='#5d6b7a')
d.rounded_rectangle((55,158,1945,1005),25,fill='white',outline='#dbe2e9',width=2)
text(175,185,'人体 24 个关节点',34,bold=True); text(755,185,'Gento Luna 目标部位',34,bold=True); text(1280,185,'编号对应的 IK 任务',34,bold=True)
# human left on viewer right
p={'pelvis':(350,650),'left_hip':(390,675),'right_hip':(310,675),'spine1':(350,602),'spine2':(350,559),'spine3':(350,516),'neck':(350,455),'head':(350,405),'left_collar':(390,494),'right_collar':(310,494),'left_shoulder':(425,505),'right_shoulder':(275,505),'left_elbow':(460,580),'right_elbow':(240,580),'left_wrist':(475,660),'right_wrist':(225,660),'left_hand':(480,690),'right_hand':(220,690),'left_knee':(397,775),'right_knee':(303,775),'left_ankle':(400,875),'right_ankle':(300,875),'left_foot':(415,905),'right_foot':(285,905)}
edges=[('pelvis','spine1'),('spine1','spine2'),('spine2','spine3'),('spine3','neck'),('neck','head'),('spine3','left_collar'),('left_collar','left_shoulder'),('left_shoulder','left_elbow'),('left_elbow','left_wrist'),('left_wrist','left_hand'),('spine3','right_collar'),('right_collar','right_shoulder'),('right_shoulder','right_elbow'),('right_elbow','right_wrist'),('right_wrist','right_hand'),('pelvis','left_hip'),('left_hip','left_knee'),('left_knee','left_ankle'),('left_ankle','left_foot'),('pelvis','right_hip'),('right_hip','right_knee'),('right_knee','right_ankle'),('right_ankle','right_foot')]
for a,b in edges: line([p[a],p[b]],'#9eacb7',9)
for x,y in p.values(): dot(x,y,'#a9b4bf',9)
# robot schematic
line([(945,440),(945,595),(945,760)],'#65788c',34)
d.rounded_rectangle((870,755,1020,824),14,fill='#536c82'); d.rounded_rectangle((830,826,1060,885),14,fill='#344d63')
d.rounded_rectangle((885,468,1005,575),20,fill='#dce8ef',outline='#536c82',width=5)
d.rounded_rectangle((916,375,974,438),15,fill='#dce8ef',outline='#536c82',width=5)
for a,b,c in [((890,490),(840,560),(810,640)),((1000,490),(1050,560),(1080,640))]:
 line([a,b,c],'#65788c',24); dot(*a,'#40576b',13); dot(*b,'#40576b',13); dot(*c,'#40576b',13)
 d.rounded_rectangle((c[0]-18,c[1]+8,c[0]+18,c[1]+45),5,fill='#607b8e')
colors=['#7759aa','#357e98','#c68936','#d36f3b','#3f86c8','#d36f3b','#3f86c8','#d36f3b','#3f86c8']
rows=[('head',(945,408),'头部 → Head_J2_Link','仅姿态 Y/Z，位置不参与'),('spine3',(945,525),'胸部 → Gento Luna_Leg_J6_Link','位置 X/Y 与姿态；经 IK 带动支撑链'),('pelvis',(945,850),'骨盆 → base_Link','任务参考；本次底盘自由度固定'),('left_shoulder',(1000,490),'左肩 → Link2_L','位置 + 姿态'),('right_shoulder',(890,490),'右肩 → Link2_R','位置 + 姿态'),('left_elbow',(1050,560),'左肘 → Link4_L','位置 + 姿态'),('right_elbow',(840,560),'右肘 → Link4_R','位置 + 姿态'),('left_wrist',(1080,640),'左腕 → TCP_Link_L','位置 + 姿态；末端偏移 0.087 m'),('right_wrist',(810,640),'右腕 → TCP_Link_R','位置 + 姿态；末端偏移 0.087 m')]
for i,(name,rb,title,detail) in enumerate(rows,1):
 col=colors[i-1]; x,y=p[name]; badge(x,y,i,col); badge(*rb,i,col)
 y=255+(i-1)*78; badge(1304,y+20,i,col); text(1335,y,title,25,bold=True); text(1335,y+34,detail,20,color='#647383')
text(180,935,'彩色编号：参与任务的 9 点',23,color='#566677'); text(755,935,'编号指向机器人上的目标坐标系',23,color='#566677')
d.rounded_rectangle((55,1030,1945,1475),25,fill='white',outline='#dbe2e9',width=2)
text(90,1055,'如何阅读这张图',33,bold=True)
items=[('灰色人体点','没有直接配置 IK 目标：双侧 hip、knee、ankle、foot、hand，以及 spine1、spine2、neck、collar。'),('手腕与夹爪','wrist 映射到机械臂 TCP；hand 不参与该 IK。左右夹爪开合由同侧手柄 trigger 单独控制。'),('机器人支撑链','人体腿部不逐关节映射；胸部等目标通过 IK 间接影响机器人支撑链。'),('这次回放','首次有效姿态建立相对参考，位置变化比例为 0.8；底盘目标关闭，底盘在 IK 中固定。'),('含义与限制','连线表示任务目标，不是一根人体骨骼直接对应一根机器人关节；实际输出还受反馈和限幅影响。')]
for i,(title,detail) in enumerate(items):
 y=1120+i*64; text(90,y,title,25,color='#245e81',bold=True); text(285,y,detail,23)
text(77,1494,'依据：gmr_body/resources/body_config.json、luna_ik.json 和 BodyBackend；以本次相对对齐实验设置为准。',21,color='#6b7784')
im.save(out); print(out.resolve())

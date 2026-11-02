"""Program 2 live view: knowledge boundary plus current sensors, without RViz."""
import math
import time
import tkinter as tk
from tkinter import ttk
from .navigation import MIN_SCAN_COVERAGE_PERCENT


class LiveView(ttk.Frame):
    def __init__(self,parent):
        super().__init__(parent)
        self.knowledge=None;self.data=None;self.trail=[]
        self.fault_notice=tk.StringVar(value='')
        ttk.Label(self,textvariable=self.fault_notice,foreground='#a12327',wraplength=850,
            font=('Noto Sans Thai',-16)).pack(fill='x')
        self.progress_percent=0.
        self.finished=False
        self.finish_started=None
        self._finish_after=None
        self.progress_canvas=tk.Canvas(self,height=112,bg='#102737',highlightthickness=0)
        self.progress_canvas.pack(fill='x',pady=(0,6))
        self.progress_canvas.bind('<Configure>',lambda event:self.draw_progress())
        self.bind('<Destroy>',self.cancel_animation,add='+')
        self.canvas=tk.Canvas(self,bg='#142230',highlightthickness=0,height=160)
        self.canvas.pack(fill='both',expand=True)
        self.canvas.bind('<Configure>',lambda event:self.draw())
        ttk.Label(self,text='เขียว = หุ่น · แดง = LiDAR สด · ชมพู = ทางที่เลือกเฉพาะหน้า · ฟ้า = รอยทางจาก odometry',
            wraplength=850).pack(anchor='w',pady=4)
        ttk.Label(self,text='ม่วง = มุมที่ยังไม่เห็น · จุดเทา = สแกนเก่า · กรอบมาจาก knowledge ไม่มีสิ่งกีดขวางเดิม',
            wraplength=850).pack(anchor='w')
        metrics=ttk.Frame(self);metrics.pack(fill='x',pady=6)
        self.values={}
        for index,(key,label) in enumerate((('elapsed','เวลา [s]'),('distance','ระยะจาก odom [m]'),
                ('goal','ห่าง Goal [m]'),('scan','อายุ LiDAR ที่ใช้ [s]'),
                ('coverage','สแกนล่าสุดอ่านได้ [%]'),('missing','สแกนล่าสุดขาด [%]'),
                ('counts','จุดอ่านได้ / ทั้งหมด'),('unknown','มุมที่ยังไม่ยืนยัน'))):
            metrics.columnconfigure(index%4,weight=1)
            frame=ttk.Frame(metrics);frame.grid(row=index//4,column=index%4,sticky='ew',padx=5,pady=4)
            ttk.Label(frame,text=label).pack()
            value=tk.StringVar(value='—');self.values[key]=value
            ttk.Label(frame,textvariable=value,font=('DejaVu Sans',-23,'bold')).pack()
        ttk.Label(self,text='เปอร์เซ็นต์แสดงความครบของข้อมูลสแกน ไม่ใช่ความน่าจะเป็นว่าเดินแล้วไม่ชน',
            wraplength=850).pack(fill='x')
        self.coverage_gate=tk.StringVar(value=f'เกณฑ์สแกน ≥ {MIN_SCAN_COVERAGE_PERCENT:.0f}%: รอข้อมูล')
        ttk.Label(self,textvariable=self.coverage_gate,font=('Noto Sans Thai',-16,'bold')).pack(fill='x')
        self.diagnostics=tk.StringVar(value='เลือก knowledge แล้วกดตรวจเซนเซอร์เพื่อดูข้อมูลสด')
        ttk.Label(self,textvariable=self.diagnostics,wraplength=850).pack(fill='x')

    def set_knowledge(self,knowledge):
        self.knowledge=knowledge;self.reset()

    def reset(self):
        self.cancel_animation()
        self.data=None;self.trail=[]
        self.progress_percent=0.;self.finished=False;self.finish_started=None
        self.fault_notice.set('')
        self.coverage_gate.set(f'เกณฑ์สแกน ≥ {MIN_SCAN_COVERAGE_PERCENT:.0f}%: รอข้อมูล')
        for value in self.values.values():value.set('—')
        self.diagnostics.set('ยังไม่รับเซนเซอร์ · การเลือกไฟล์ยังไม่เริ่ม ROS หรือมอเตอร์')
        self.draw_progress()
        self.draw()

    def cancel_animation(self,event=None):
        if event is not None and event.widget is not self:return
        if self._finish_after is not None:
            self.after_cancel(self._finish_after)
            self._finish_after=None

    def update_progress(self,data):
        if data.get('drive') is True and data.get('reason')=='arrived_estimate':
            if not self.finished:
                self.finished=True
                self.finish_started=time.monotonic()
                self.animate_finish()
            self.progress_percent=100.
        elif not self.finished and data.get('anchored',data.get('odom_age') is not None):
            k=self.knowledge
            initial=math.dist((k.start.x,k.start.y),k.goal)
            pose=data['pose']
            remaining=math.dist((pose['x'],pose['y']),k.goal)
            self.progress_percent=min(99.,max(0.,100*(1-remaining/initial))) if initial>0 else 0.
        self.draw_progress()

    def draw_progress(self):
        c=self.progress_canvas;c.delete('all')
        width=max(240,c.winfo_width());left,right=28,width-28
        color='#5ff0b1' if self.finished else '#40d9ef'
        c.create_text(left,24,anchor='w',text='Finish · ถึง Goal แล้ว' if self.finished else 'Start → Goal',
            fill=color,font=('Noto Sans Thai',-21,'bold'))
        c.create_text(right,24,anchor='e',text=f'{self.progress_percent:.0f}%',fill=color,
            font=('DejaVu Sans',-27,'bold'),tags='progress_percent')
        c.create_line(left,61,right,61,fill='#294959',width=12,capstyle='round')
        tip=left+(right-left)*self.progress_percent/100
        if tip>left:c.create_line(left,61,tip,61,fill=color,width=12,capstyle='round')
        c.create_oval(tip-7,54,tip+7,68,fill='#ecfffa',outline=color,width=2)
        c.create_text(left,86,anchor='w',text='Start · 0%',fill='#b9cad8',font=('DejaVu Sans',-12))
        c.create_text(right,86,anchor='e',text='Goal · 100%',fill='#b9cad8',font=('DejaVu Sans',-12))
        c.create_text(width/2,104,text='คำนวณจากระยะห่าง Goal · อาจลดลงเมื่อเดินอ้อม',
            fill='#90aebe',font=('Noto Sans Thai',-12))

    def animate_finish(self):
        self._finish_after=None
        self.draw_finish()
        if self.finished and time.monotonic()-self.finish_started<4:
            self._finish_after=self.after(33,self.animate_finish)

    def draw_finish(self):
        c=self.canvas;c.delete('finish')
        if not self.finished:return
        elapsed=time.monotonic()-self.finish_started
        width,height=c.winfo_width(),c.winfo_height()
        x,y=width/2,height/2
        pulse=3*math.sin(min(elapsed,4)*5) if elapsed<4 else 0
        half=min(210,width/2-20)
        c.create_rectangle(x-half-5,y-80-pulse,x+half+5,y+80+pulse,
            fill='#1a6359',outline='',tags='finish')
        c.create_rectangle(x-half,y-75,x+half,y+75,fill='#102f35',outline='#5ff0b1',width=2,tags='finish')
        c.create_text(x,y-18,text='Finish',fill='#72f6ba',font=('DejaVu Sans',-48,'bold'),tags='finish')
        c.create_text(x,y+37,text='ถึง Goal แล้ว · 100%',fill='#e3fff4',font=('Noto Sans Thai',-22),tags='finish')
        if elapsed<4:
            colors=('#5ff0b1','#40d9ef','#ffcf66','#f68bbc')
            for i in range(42):
                px=(i*.61803398875%1)*width
                py=((elapsed*(70+i%5*17)+i*43)%(height+60))-30
                c.create_rectangle(px,py,px+4+i%4,py+8,fill=colors[i%4],outline='',tags='finish')

    def update_data(self,data):
        if not isinstance(data,dict) or self.knowledge is None or data.get('fingerprint')!=self.knowledge.fingerprint:return
        try:
            pose=data['pose']
            if not all(math.isfinite(pose[key]) for key in ('x','y','yaw')):return
            if not all(math.isfinite(data.get('sensor_pose',pose)[key]) for key in ('x','y','yaw')):return
            for key in ('points','path'):
                if not isinstance(data.get(key,[]),list):return
                if any(len(p)!=2 or not all(math.isfinite(v) for v in p) for p in data.get(key,[])):return
            if any(data.get(key) is not None and not math.isfinite(data[key])
                    for key in ('elapsed','distance','estimated_goal_error','scan_age','odom_age','scan_message_age','conditions_age',
                        'scan_coverage_percent','scan_missing_percent','scan_total_beams','scan_valid_beams',
                        'scan_missing_beams','unknown_scan_sectors','scan_required_coverage_percent')):return
            if any(data.get(key) is not None and not 0<=data[key]<=100
                    for key in ('scan_coverage_percent','scan_missing_percent')):return
            required=data.get('scan_required_coverage_percent',MIN_SCAN_COVERAGE_PERCENT)
            if not isinstance(required,(int,float)) or not math.isfinite(required) or not 0<required<=100:return
            checks=data.get('movement_conditions',[])
            if not isinstance(checks,list) or any(not isinstance(item,dict) or not {'code','label','passed','detail','scope'}<=item.keys()
                    or item['passed'] is not True and item['passed'] is not False and item['passed'] is not None for item in checks):return
            velocity=data.get('requested',[0.,0.])
            if len(velocity)!=2 or not all(math.isfinite(v) for v in velocity):return
        except (KeyError,TypeError,ValueError):return
        self.data=data
        self.update_progress(data)
        notice=''
        if data.get('stop_latched') and data.get('reason')!='arrived_estimate':
            notice=data.get('explanation',data.get('reason','หยุด'))+' · กดหยุดก่อนเริ่มรอบใหม่'
            fault=data.get('sensor_fault')
            if isinstance(fault,dict):
                age=fault.get('age')
                if isinstance(age,(int,float)) and math.isfinite(age):
                    notice+=f'\nข้อความ {fault.get("stream","")} ที่ทำให้หยุด: '
                    notice+=f'เก่า {age:.3f} s' if age>=0 else f'เร็วกว่าเวลาคอม {-age:.3f} s'
                    notice+=' (แยกจากอายุสแกนปัจจุบันด้านล่าง)'
        self.fault_notice.set(notice)
        point=(pose['x'],pose['y'])
        if not self.trail or math.dist(point,self.trail[-1])>.003:
            self.trail.append(point);self.trail=self.trail[-2000:]
        for key,source in (('elapsed','elapsed'),('distance','distance'),('goal','estimated_goal_error'),('scan','scan_age')):
            value=data.get(source);self.values[key].set('—' if value is None else f'{value:.2f}')
        for key,source in (('coverage','scan_coverage_percent'),('missing','scan_missing_percent')):
            value=data.get(source);self.values[key].set('—' if value is None else f'{value:.1f}')
        coverage=data.get('scan_coverage_percent')
        required=data.get('scan_required_coverage_percent',MIN_SCAN_COVERAGE_PERCENT)
        gate='รอข้อมูล' if coverage is None else 'ผ่าน' if coverage>=required else 'ไม่ผ่าน'
        self.coverage_gate.set(f'เกณฑ์สแกน ≥ {required:.0f}%: {gate}')
        total=data.get('scan_total_beams');valid=data.get('scan_valid_beams')
        self.values['counts'].set('—' if not total or valid is None else f'{valid:.0f} / {total:.0f}')
        unknown=data.get('unknown_scan_sectors')
        self.values['unknown'].set('—' if unknown is None else f'{unknown:.0f}')
        age=data.get('scan_message_age')
        freshness='ยังไม่มีสแกน' if age is None else ('สด' if -.06<=age<=self.knowledge.hardware.scan_timeout else 'เก่าหรือเวลาไม่ตรง')
        missing=data.get('scan_missing_beams')
        checks=data.get('movement_conditions',[])
        failed=sum(isinstance(item,dict) and item.get('passed') is False for item in checks)
        unknown_checks=sum(isinstance(item,dict) and item.get('passed') is None for item in checks)
        self.diagnostics.set(f"{'เดินจริง' if data.get('drive') else 'ตรวจเซนเซอร์ ไม่สั่งมอเตอร์'} · {data.get('mode','')} / {data.get('reason','')}\n"
            f"v={velocity[0]:.3f} m/s · ω={velocity[1]:.3f} rad/s · guard: {data.get('guard','—')}\n"
            f"สแกนล่าสุด: {freshness} · ขาด {missing if missing is not None else '—'} จุด · "
            f"ประวัติเติม {data.get('filled_scan_gaps','—')} ช่อง · ประมาณช่องแคบ {data.get('interpolated_scan_gaps',0)} ช่อง\n"
            f"ไม่ผ่าน {failed} ข้อ · ยังตรวจไม่ได้ {unknown_checks} ข้อ — ดูแท็บเงื่อนไขการเดิน")
        self.draw()

    def draw(self):
        canvas=self.canvas;canvas.delete('all')
        if self.knowledge is None:
            canvas.create_text(canvas.winfo_width()/2,canvas.winfo_height()/2,text='เลือกไฟล์ knowledge.json',fill='#b9cad8',font=('Noto Sans Thai',-22));return
        k=self.knowledge;a=k.arena
        scale=max(1.,min((canvas.winfo_width()-100)/a.width,(canvas.winfo_height()-100)/a.height))
        ox=(canvas.winfo_width()-a.width*scale)/2;oy=(canvas.winfo_height()+a.height*scale)/2
        def xy(x,y):return ox+x*scale,oy-y*scale
        canvas.create_rectangle(*xy(0,a.height),*xy(a.width,0),outline='#a9bccb',width=2,tags='arena')
        data=self.data
        pose=data['pose'] if data else vars(k.start)
        yaw=pose['yaw'];cy,sy=math.cos(yaw),math.sin(yaw)
        sensor_pose=data.get('sensor_pose',pose) if data else pose
        sensor_c,sensor_s=math.cos(sensor_pose['yaw']),math.sin(sensor_pose['yaw'])
        def sensor_point(point):
            x,y=point
            return xy(sensor_pose['x']+sensor_c*x-sensor_s*y,sensor_pose['y']+sensor_s*x+sensor_c*y)
        if data:
            age=data.get('scan_age')
            color='#f16a6a' if age is not None and age<=k.hardware.scan_timeout else '#7c8188'
            for point in data.get('points',[]):
                x,y=sensor_point(point)
                canvas.create_oval(x-2,y-2,x+2,y+2,fill=color,outline='',tags='lidar')
            for sector in data.get('unknown',[]):
                try:
                    origin=sector['origin'];angle=sector['angle'];half=sector['half_width'];radius=sector['minimum']
                    if not all(math.isfinite(v) for v in (*origin,angle,half,radius)):continue
                    # Draw the near edge of each unobserved sector; no saved
                    # obstacle layout or fake range is drawn in its place.
                    points=[sensor_point((origin[0]+radius*math.cos(angle-half+2*half*i/4),
                        origin[1]+radius*math.sin(angle-half+2*half*i/4))) for i in range(5)]
                    canvas.create_line(*[v for p in points for v in p],fill='#b497d6',width=3,tags='unknown')
                except (KeyError,TypeError,ValueError):continue
        if len(self.trail)>1:
            canvas.create_line(*[v for p in self.trail for v in xy(*p)],fill='#39b5e8',width=3,tags='trail')
        path=data.get('path',[]) if data else []
        if len(path)>1:canvas.create_line(*[v for p in path for v in xy(*p)],fill='#ef5da8',width=3,tags='planned_path')
        for label,point,color in (('Start',(k.start.x,k.start.y),'#36d4dd'),('Goal',k.goal,'#ff9d38')):
            x,y=xy(*point);radius=(k.hardware.radius if label=='Start' else k.hardware.goal_tolerance)*scale
            canvas.create_oval(x-radius,y-radius,x+radius,y+radius,outline=color,width=2,tags=label.lower())
            canvas.create_text(x,y-radius-12,text=label,fill=color,font=('DejaVu Sans',-14))
        x,y=xy(pose['x'],pose['y']);radius=k.hardware.radius*scale;safe=k.hardware.safe_radius*scale
        canvas.create_oval(x-safe,y-safe,x+safe,y+safe,outline='#61ce99',dash=(4,4),tags='robot_margin')
        canvas.create_oval(x-radius,y-radius,x+radius,y+radius,outline='#61e49a',width=3,tags='robot')
        canvas.create_line(x,y,*xy(pose['x']+.18*cy,pose['y']+.18*sy),fill='#61e49a',width=3,arrow='last',tags='heading')
        if not data or data.get('odom_age') is None:
            canvas.create_text(canvas.winfo_width()/2,25,text='ยังไม่ยืนยันตำแหน่งสด · แสดง Start ที่บันทึก',fill='#b9cad8',font=('Noto Sans Thai',-15))
        self.draw_finish()

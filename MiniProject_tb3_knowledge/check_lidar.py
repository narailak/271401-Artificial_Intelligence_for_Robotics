#!/usr/bin/env python3
"""Read-only LDS-02 distance viewer. Front is +x in the robot base frame."""
import argparse
import json
import math
from pathlib import Path
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk
from core.model import Hardware

HARDWARE=Hardware()
MIN_RANGE=HARDWARE.lidar_min
MAX_RANGE=HARDWARE.scan_range


def measurements(scan, mount, half_angle, offset=0.,direction=0.):
    """Keep actual finite returns; select the nearest return in the chosen base-frame cone."""
    points=[]
    selected=None
    mx,my,yaw=mount
    for i,distance in enumerate(scan.ranges):
        if not math.isfinite(distance) or not max(MIN_RANGE,scan.range_min)<=distance<=min(MAX_RANGE,scan.range_max):continue
        corrected=distance+offset
        if corrected<=0:continue
        angle=scan.angle_min+i*scan.angle_increment+yaw
        angle=math.atan2(math.sin(angle),math.cos(angle))
        point=dict(x=mx+corrected*math.cos(angle),y=my+corrected*math.sin(angle),
                   raw=distance,corrected=corrected,angle=angle)
        points.append(point)
        difference=math.atan2(math.sin(angle-direction),math.cos(angle-direction))
        if abs(difference)<=half_angle+1e-12 and (selected is None or distance<selected['raw']):selected=point
    return points,selected


def edge_gap(point,direction,edge):
    """Distance normal to a test plane, minus the measured sensor-to-body edge offset."""
    return point['corrected']*math.cos(point['angle']-direction)-edge


class LidarApp:
    def __init__(self,root,args):
        self.root,self.args=root,args
        self.node=self.context=self.executor=None
        self.scan=self.mount=None
        self.received=0.
        self.timer=None
        self.status=tk.StringVar(value='รอ /scan และ TF · โปรแกรมนี้ไม่ส่งคำสั่งมอเตอร์')
        self.front=tk.StringVar(value='7.0')
        self.direction=tk.StringVar(value='0.0')
        self._choosing_direction=False
        self.cone=tk.StringVar(value='5.0')
        self.offset=tk.StringVar(value='0.0')
        self.actual=tk.StringVar(value='')
        self.zoom=tk.StringVar(value='1.0')
        self.values={key:tk.StringVar(value='—') for key in ('raw','sensor','front','guard','angle','age','coverage')}
        self.points=[];self.selected=None;self.fresh=False
        self.reason='รอเซนเซอร์'
        root.title('ตรวจระยะ LiDAR · TurtleBot3 Burger / LDS-02')
        root.geometry('1160x820');root.minsize(980,700)
        style=ttk.Style(root)
        for name in ('TLabel','TButton','TEntry'):style.configure(name,font=('Noto Sans Thai',-15))
        outer=ttk.Frame(root,padding=16);outer.pack(fill='both',expand=True)
        ttk.Label(outer,text='ตรวจระยะจริง · เลือกทิศวัด · ด้านหน้าหุ่นคือเส้นสีฟ้า',font=('Noto Sans Thai',-24,'bold')).pack(anchor='w')
        ttk.Label(outer,textvariable=self.status,wraplength=1100).pack(fill='x',pady=8)
        body=ttk.Frame(outer);body.pack(fill='both',expand=True)
        panel=ttk.Frame(body);panel.pack(side='left',fill='y',padx=(0,16))
        scroll=tk.Canvas(panel,width=350,highlightthickness=0)
        bar=ttk.Scrollbar(panel,orient='vertical',command=scroll.yview)
        bar.pack(side='right',fill='y');scroll.pack(side='left',fill='both',expand=True)
        scroll.configure(yscrollcommand=bar.set)
        controls=ttk.Frame(scroll)
        window=scroll.create_window(0,0,window=controls,anchor='nw')
        controls.bind('<Configure>',lambda event:scroll.configure(scrollregion=scroll.bbox('all')))
        scroll.bind('<Configure>',lambda event:scroll.itemconfigure(window,width=event.width))
        self.canvas=tk.Canvas(body,bg='#122333',highlightthickness=0)
        self.canvas.pack(side='right',fill='both',expand=True)
        self.canvas.bind('<Configure>',lambda event:self.draw())
        ttk.Label(controls,text='ทิศที่ต้องการตรวจ').pack(anchor='w')
        self.direction_box=ttk.Combobox(controls,state='readonly',values=(
            'หน้า · 0°','ซ้าย · 90°','ขวา · -90°','หลัง · 180°','กำหนดมุมเอง'))
        self.direction_box.current(0);self.direction_box.pack(fill='x',pady=5)
        self.direction_box.bind('<<ComboboxSelected>>',self.choose_direction)
        self.direction.trace_add('write',self.direction_edited)
        for title,var in (('มุมวัดจากหน้าหุ่น [°]',self.direction),
                          ('LiDAR → ขอบหุ่นด้านที่เลือก [cm]',self.front),('ครึ่งมุมกรวยวัด [°]',self.cone),
                          ('ชดเชยระยะ LiDAR [cm]',self.offset),('ระยะภาพจากศูนย์หุ่น [m]',self.zoom)):
            row=ttk.Frame(controls);row.pack(fill='x',pady=5)
            ttk.Label(row,text=title).pack(side='left')
            ttk.Entry(row,textvariable=var,width=7).pack(side='right')
        ttk.Label(controls,text='มุมบวกไปทางซ้าย มุมลบไปทางขวา\nค่าขอบตั้งต้น: หน้า/หลัง 7 cm, ซ้าย/ขวา 8.9 cm\nต้องวัดจากศูนย์ LiDAR ถึงขอบด้านนั้นจริง\nรวมล้อและอุปกรณ์ที่ยื่นออกมา',wraplength=330).pack(anchor='w',pady=8)
        ttk.Separator(controls).pack(fill='x',pady=7)
        for key,label in (('raw','LiDAR ดิบ → วัตถุ [cm]'),('sensor','LiDAR ชดเชยแล้ว → วัตถุ [cm]'),
                          ('front','ช่องว่างจากขอบด้านที่เลือก [cm]'),
                          ('guard','ถึงวงตรวจ Program 1/2 [cm]'),('angle','มุมลำแสงที่เลือก [°]'),
                          ('age','อายุสแกน [s]'),('coverage','ลำแสงอ่านได้ [%]')):
            row=ttk.Frame(controls);row.pack(fill='x',pady=4)
            ttk.Label(row,text=label).pack(side='left')
            ttk.Label(row,textvariable=self.values[key],font=('DejaVu Sans',-20,'bold')).pack(side='right')
        ttk.Separator(controls).pack(fill='x',pady=9)
        ttk.Label(controls,text=f'วงประ = วงครอบหุ่น {HARDWARE.radius*100:.1f} + เผื่อ {HARDWARE.safety_margin*100:.0f} cm\nวงตรวจรวมรัศมี {HARDWARE.safe_radius*100:.1f} cm\nค่าถึงวงตรวจติดลบ = จุดวัดอยู่ในวงตรวจ',wraplength=330).pack(anchor='w',pady=4)
        ttk.Label(controls,text='ระยะจริงจากขอบด้านที่เลือกถึงแผ่นทดสอบ [cm]',wraplength=330).pack(anchor='w')
        ttk.Entry(controls,textvariable=self.actual,width=12).pack(anchor='w',pady=5)
        ttk.Button(controls,text='คำนวณค่าชดเชยจากระยะจริง',command=self.calibrate).pack(fill='x',pady=4)
        ttk.Button(controls,text='บันทึกค่าตรวจวัด JSON',command=self.save).pack(fill='x',pady=4)
        ttk.Label(controls,text='ให้หุ่นอยู่นิ่ง วางแผ่นเรียบตั้งฉากในกรวยสีฟ้า\nแสดงวัตถุที่ใกล้ที่สุดในกรวย ไม่ได้ระบุชนิดวัตถุ\nไม่มี return หรือสแกนเก่า: ไม่ยืนยันระยะ\nค่าชดเชยนี้ใช้เฉพาะหน้าตรวจวัดนี้\nควรเทียบไม้บรรทัดหลายระยะก่อนนำไปใช้งาน',wraplength=330).pack(anchor='w',pady=10)
        root.protocol('WM_DELETE_WINDOW',self.close)

    def settings(self):
        front,cone,offset,zoom=map(float,(self.front.get(),self.cone.get(),self.offset.get(),self.zoom.get()))
        direction=float(self.direction.get())
        if not all(math.isfinite(v) for v in (front,cone,offset,zoom,direction)):
            raise ValueError('ค่าตั้งต้องเป็นตัวเลขที่มีค่าจำกัด')
        if not 0<front<=50 or not .5<=cone<=30 or not -20<=offset<=20 or not .2<=zoom<=8:
            raise ValueError('ระยะถึงขอบ 0–50 cm, ครึ่งมุม 0.5–30°, ชดเชย ±20 cm, ระยะภาพ 0.2–8 m')
        direction=math.radians(direction%360)
        return front/100,math.radians(cone),offset/100,zoom,direction

    def direction_edited(self,*args):
        if not self._choosing_direction:self.direction_box.set('กำหนดมุมเอง')
        self.actual.set('');self.selected=None

    def choose_direction(self,event=None):
        presets={'หน้า · 0°':(0,7.),'ซ้าย · 90°':(90,8.9),
                 'ขวา · -90°':(-90,8.9),'หลัง · 180°':(180,7.)}
        selected=presets.get(self.direction_box.get())
        if selected:
            self._choosing_direction=True
            try:
                self.direction.set(str(selected[0]));self.front.set(str(selected[1]))
            finally:self._choosing_direction=False
        self.actual.set('');self.selected=None

    def start_ros(self):
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import qos_profile_sensor_data
        from rclpy.signals import SignalHandlerOptions
        from sensor_msgs.msg import LaserScan
        import tf2_ros
        self.context=Context()
        rclpy.init(args=[],context=self.context,signal_handler_options=SignalHandlerOptions.NO)
        self.node=rclpy.create_node('lidar_distance_viewer',context=self.context)
        self.executor=SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.buffer=tf2_ros.Buffer()
        self.listener=tf2_ros.TransformListener(self.buffer,self.node)
        self.subscription=self.node.create_subscription(LaserScan,self.args.scan_topic,self.receive,qos_profile_sensor_data)

    def receive(self,scan):
        if (not scan.header.frame_id or len(scan.ranges)<2
                or not all(math.isfinite(v) for v in (scan.angle_min,scan.angle_increment,scan.range_min,scan.range_max))
                or scan.angle_increment==0 or not 0<=scan.range_min<scan.range_max):
            self.scan=self.mount=None;self.reason='ข้อมูลสแกนผิดรูปแบบ';return
        self.scan=scan;self.mount=None;self.received=time.monotonic()

    def refresh(self):
        self.timer=None
        if self.args.demo:self.received=time.monotonic()
        if self.node:
            from rclpy.time import Time
            from core.ros import yaw_from_quaternion
            for _ in range(8):self.executor.spin_once(timeout_sec=0.)
            if self.scan:
                try:
                    if self.scan.header.frame_id.lstrip('/')==self.args.base_frame.lstrip('/'):
                        self.mount=(0.,0.,0.)
                    else:
                        tf=self.buffer.lookup_transform(self.args.base_frame,self.scan.header.frame_id,Time.from_msg(self.scan.header.stamp)).transform
                        if not all(math.isfinite(v) for v in (tf.translation.x,tf.translation.y,tf.translation.z)):
                            raise ValueError('TF ไม่ถูกต้อง')
                        self.mount=(tf.translation.x,tf.translation.y,yaw_from_quaternion(tf.rotation))
                    self.reason=''
                except Exception as exc:self.mount=None;self.reason='รอ TF: '+str(exc).split('\n')[0]
        try:
            front,cone,offset,_,direction=self.settings()
            self.fresh=False;self.points=[];self.selected=None
            if self.scan:
                scan=self.scan
                stamp=scan.header.stamp.sec+scan.header.stamp.nanosec*1e-9
                age=self.node.get_clock().now().nanoseconds*1e-9-stamp if self.node else 0.
                self.values['age'].set(f'{age:.2f}')
                self.fresh=self.mount is not None and -.06<=age<=.6 and time.monotonic()-self.received<=.6
                self.values['coverage'].set(f'{100*sum(math.isfinite(v) and max(MIN_RANGE,scan.range_min)<=v<=min(MAX_RANGE,scan.range_max) for v in scan.ranges)/len(scan.ranges):.1f}')
                if self.mount:self.points,self.selected=measurements(scan,self.mount,cone,offset,direction)
                if not self.fresh:self.selected=None
            for key in ('raw','sensor','front','guard','angle'):self.values[key].set('—')
            if self.selected:
                p=self.selected
                self.values['raw'].set(f"{p['raw']*100:.1f}")
                self.values['sensor'].set(f"{p['corrected']*100:.1f}")
                gap=edge_gap(p,direction,front)
                self.values['front'].set(f'{gap*100:.1f}')
                self.values['guard'].set(f"{(math.hypot(p['x'],p['y'])-HARDWARE.safe_radius)*100:.1f}")
                self.values['angle'].set(f"{math.degrees(p['angle']):+.1f}")
            angle=math.degrees(math.atan2(math.sin(direction),math.cos(direction)))
            detail=self.reason or (f'ข้อมูลสด · วัดทิศ {angle:+.1f}°' if self.selected else f'ไม่ยืนยันระยะทิศ {angle:+.1f}°: ไม่มี return ที่ใช้ได้ หรือสแกนเก่า')
            if self.scan:detail+=f' · กรองระยะดิบ {max(MIN_RANGE,self.scan.range_min)*100:.0f}–{min(MAX_RANGE,self.scan.range_max)*100:.0f} cm'
            self.status.set(('DEMO · ข้อมูลสมมติ · ' if self.args.demo else '')+detail)
            self.draw()
        except (ValueError,TypeError) as exc:
            self.fresh=False;self.selected=None
            for key in ('raw','sensor','front','guard','angle'):self.values[key].set('—')
            self.status.set(str(exc))
        self.timer=self.root.after(80,self.refresh)

    def draw(self):
        c=self.canvas;c.delete('all')
        try:front,cone,_,zoom,direction=self.settings()
        except ValueError:return
        width,height=c.winfo_width(),c.winfo_height()
        scale=max(1,min(width,height)*.46/zoom);cx,cy=width/2,height/2
        def xy(x,y):return cx-y*scale,cy-x*scale
        radius=HARDWARE.safe_radius*scale
        c.create_oval(cx-radius,cy-radius,cx+radius,cy+radius,outline='#6bf0af',dash=(5,4),width=2,tags='safety_circle')
        for cm in (10,20,50,100,200,400):
            r=cm/100*scale
            if cm/100>zoom:continue
            c.create_oval(cx-r,cy-r,cx+r,cy+r,outline='#284252')
            c.create_text(cx+r-15,cy+12,text=f'{cm} cm',fill='#849eaf',font=('DejaVu Sans',-11))
        mx,my,_=self.mount or (-.032,0.,0.)
        dx,dy=math.cos(direction),math.sin(direction)
        edge=(mx+front*dx,my+front*dy)
        c.create_polygon(*[v for p in ((mx,my),(mx+zoom*math.cos(direction+cone),my+zoom*math.sin(direction+cone)),
            (mx+zoom*math.cos(direction-cone),my+zoom*math.sin(direction-cone))) for v in xy(*p)],fill='#173c50',outline='',tags='measurement_cone')
        for point in self.points:
            if math.hypot(point['x'],point['y'])>zoom:continue
            x,y=xy(point['x'],point['y'])
            c.create_oval(x-2,y-2,x+2,y+2,fill='#ffbd72' if self.fresh else '#77838d',outline='')
        c.create_rectangle(*xy(.038,.07),*xy(-.102,-.07),outline='#6bf0af',fill='#20473e',width=2)
        for side in (-1,1):
            c.create_rectangle(*xy(0.033,side*.08+.009),*xy(-.033,side*.08-.009),fill='#8399a6',outline='')
        x,y=xy(mx,my);r=.035*scale
        c.create_oval(x-r,y-r,x+r,y+r,outline='#e6fff9',width=2)
        c.create_line(*xy(mx,my),*xy(mx+zoom*.75,my),fill='#42d9ff',width=2,arrow='last',dash=(6,4),tags='front_axis')
        c.create_line(*xy(mx,my),*xy(mx+zoom*.8*dx,my+zoom*.8*dy),fill='#6bf0af',width=2,arrow='last',dash=(4,4),tags='direction_axis')
        c.create_line(*xy(edge[0]-.09*dy,edge[1]+.09*dx),*xy(edge[0]+.09*dy,edge[1]-.09*dx),fill='#6bf0af',width=3)
        c.create_text(width/2,22,text='ด้านหน้า · +x',fill='#42d9ff',font=('Noto Sans Thai',-20,'bold'))
        c.create_text(36,height/2,text='ซ้าย',fill='#b9d3df');c.create_text(width-36,height/2,text='ขวา',fill='#b9d3df')
        c.create_text(width/2,height-70,text='หลัง · -x',fill='#b9d3df')
        if self.selected:
            p=self.selected;target=xy(p['x'],p['y'])
            c.create_line(x,y,*target,fill='#ffd36e',width=3,tags='measurement')
            c.create_oval(target[0]-6,target[1]-6,target[0]+6,target[1]+6,outline='#fff2ca',width=2)
            gap=edge_gap(p,direction,front)*100
            c.create_text(width/2,height-36,text=f'ขอบด้านที่เลือก → วัตถุ {gap:.1f} cm',fill='#ffd36e',font=('Noto Sans Thai',-23,'bold'))
        else:c.create_text(width/2,height-36,text='ยังไม่ยืนยันระยะวัตถุในทิศที่เลือก',fill='#b9d3df',font=('Noto Sans Thai',-19))

    def calibrate(self):
        try:
            if not self.fresh or self.selected is None:raise ValueError('ต้องมีสแกนสดและวัตถุในทิศที่เลือกก่อน')
            front,cone,offset,_,direction=self.settings()
            _,point=measurements(self.scan,self.mount,cone,offset,direction)
            if point is None:raise ValueError('ไม่พบวัตถุในทิศที่เลือก')
            actual=float(self.actual.get())/100
            if not math.isfinite(actual) or actual<0:raise ValueError('ระยะจริงต้องเป็นตัวเลขตั้งแต่ 0 cm')
            cosine=math.cos(point['angle']-direction)
            measured=edge_gap(point,direction,front)
            corrected=offset+(actual-measured)/cosine
            if abs(corrected)>.2:raise ValueError('ค่าชดเชยเกิน ±20 cm ตรวจระยะถึงขอบ วัตถุที่เลือก และ TF ก่อน')
            self.offset.set(f'{corrected*100:.3f}')
        except ValueError as exc:messagebox.showerror('ตรวจระยะ',str(exc),parent=self.root)

    def save(self):
        try:
            front,cone,offset,_,direction=self.settings()
            if not self.fresh or self.selected is None:raise ValueError('ต้องมีข้อมูลสดก่อนบันทึก')
            _,point=measurements(self.scan,self.mount,cone,offset,direction)
            if point is None:raise ValueError('ไม่พบวัตถุในทิศที่เลือก')
            path=filedialog.asksaveasfilename(parent=self.root,initialfile='lidar_measurement.json',defaultextension='.json',filetypes=[('JSON','*.json')])
            if not path:return
            data=dict(purpose='measurement_only_not_navigation',demo=self.args.demo,scan_topic=self.args.scan_topic,
                scan_frame=self.scan.header.frame_id,base_frame=self.args.base_frame,mount_m=self.mount,
                direction_deg=math.degrees(math.atan2(math.sin(direction),math.cos(direction))),
                lidar_to_edge_m=front,cone_half_angle_deg=math.degrees(cone),range_offset_m=offset,
                selected=point,edge_gap_m=edge_gap(point,direction,front),
                robot_radius_m=HARDWARE.radius,safety_margin_m=HARDWARE.safety_margin,
                scan_min_m=MIN_RANGE,scan_max_m=MAX_RANGE)
            Path(path).write_text(json.dumps(data,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
        except (ValueError,OSError) as exc:messagebox.showerror('บันทึกไม่ได้',str(exc),parent=self.root)

    def close(self):
        if self.timer is not None:self.root.after_cancel(self.timer);self.timer=None
        if self.executor:self.executor.shutdown(timeout_sec=1.);self.executor=None
        if self.node:self.node.destroy_node();self.node=None
        if self.context and self.context.ok():self.context.shutdown()
        self.root.destroy()


def self_test():
    from types import SimpleNamespace
    scan=SimpleNamespace(ranges=[float('nan'),.4,.3,float('inf'),.1],range_min=.16,range_max=3.5,angle_min=-.02,angle_increment=.01)
    points,p=measurements(scan,(-.032,0.,0.),math.radians(5))
    assert len(points)==2 and p['raw']==.3
    assert math.isclose(p['x']-(-.032+.07),.23)
    _,p=measurements(scan,(-.032,0.,0.),math.radians(5),.02)
    assert math.isclose(p['corrected'],.32)
    assert measurements(scan,(0.,0.,math.pi/2),math.radians(5))[1] is None
    scan=SimpleNamespace(ranges=[.4,.5,.6,.7],range_min=0.,range_max=100.,angle_min=0.,angle_increment=math.pi/2)
    for degrees,distance in ((0,.4),(90,.5),(180,.6),(-90,.7)):
        direction=math.radians(degrees)
        _,point=measurements(scan,(-.032,.01,0.),math.radians(5),direction=direction)
        assert point is not None and point['raw']==distance
        assert math.isclose(edge_gap(point,direction,.07),distance-.07)
    _,point=measurements(scan,(0.,0.,math.pi/2),math.radians(5),direction=math.pi/2)
    assert point['raw']==.4
    seam=SimpleNamespace(ranges=[.8,.9],range_min=.16,range_max=3.5,angle_min=math.radians(-179.5),angle_increment=.01)
    assert measurements(seam,(0.,0.,0.),math.radians(1),direction=math.pi)[1]['raw']==.8
    print('LiDAR checks passed: four directions, ±180° seam, TF rotation, invalid returns, edge gap and offset.')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scan-topic',default='/scan')
    parser.add_argument('--base-frame',default='base_footprint')
    parser.add_argument('--demo',action='store_true',help='Synthetic readings only; no ROS or robot')
    parser.add_argument('--self-test',action='store_true')
    args=parser.parse_args()
    if args.self_test:self_test();return
    root=tk.Tk();app=LidarApp(root,args)
    try:
        if args.demo:
            from types import SimpleNamespace
            app.mount=(-.032,0.,0.)
            app.scan=SimpleNamespace(header=SimpleNamespace(frame_id='demo_scan',stamp=SimpleNamespace(sec=0,nanosec=0)),
                angle_min=-math.pi,angle_increment=math.pi/180,range_min=.16,range_max=3.5,
                ranges=[.37 if abs(i-180)<=5 else 1.2 for i in range(360)])
        else:app.start_ros()
        app.refresh();root.mainloop()
    except (ImportError,RuntimeError) as exc:
        messagebox.showerror('เปิด LiDAR ไม่ได้',f'{exc}\nเปิด terminal ที่ source ROS แล้วลองใหม่',parent=root)
    finally:
        if app.executor:app.executor.shutdown(timeout_sec=1.)
        if app.node:app.node.destroy_node()
        if app.context and app.context.ok():app.context.shutdown()


if __name__=='__main__':main()

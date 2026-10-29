#!/usr/bin/env python3
"""Program 2: load one knowledge file; inspect, demonstrate or navigate on PC."""
import argparse
import json
from pathlib import Path
import queue
import subprocess
import sys
import threading
import time

from core.model import Knowledge

DEFAULT=Path(__file__).resolve().parent/'output/knowledge.json'
RUNTIME_STATUS_PREFIX='TB3_STATUS: '
RUNTIME_DATA_PREFIX='TB3_DATA: '


def parser():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--knowledge',type=Path,default=DEFAULT,help='Single file from Program 1')
    p.add_argument('--gui',action='store_true',help='Open file-selection window (default with no arguments)')
    action=p.add_mutually_exclusive_group()
    action.add_argument('--inspect',action='store_true',help='Validate and print file; no ROS')
    action.add_argument('--demo',action='store_true',help='Synthetic obstacle arena, no robot or ROS; not the surveyed layout')
    action.add_argument('--drive',action='store_true',help='Real motor commands; requires a fresh localized reference')
    p.add_argument('--no-window',action='store_true',help='Print demo results without Tk playback')
    p.add_argument('--rviz',action='store_true',help='Open RViz automatically during live ROS operation')
    p.add_argument('--ui-stream',action='store_true',help=argparse.SUPPRESS)
    p.add_argument('--output-type',choices=('auto','stamped','twist'),default='auto')
    p.add_argument('--robot-id',default='tb3',help='Logical execution-robot label, must match saved reference')
    p.add_argument('--scan-topic',default='/scan')
    p.add_argument('--odom-topic',default='/odom')
    p.add_argument('--command-topic',default='/knowledge/cmd_vel')
    p.add_argument('--output-topic',default='/cmd_vel')
    p.add_argument('--log',type=Path,default=Path(__file__).resolve().parent/'output/robot-run.jsonl')
    return p


class RunnerApp:
    """File picker and explicit run controls; selecting a file never starts ROS."""
    def __init__(self,root,args):
        import tkinter as tk
        from tkinter import ttk
        self.root,self.args=root,args
        self.knowledge=self.selected_path=self.process=None
        self.events=queue.Queue()
        self.stopping=self.closing=False
        self.stop_time=0.
        self.path=tk.StringVar(value='ยังไม่ได้เลือกไฟล์')
        self.details=tk.StringVar(value='เลือก knowledge.json ที่บันทึกจาก Program 1')
        self.status=tk.StringVar(value='เลือกไฟล์แล้วกดทดลองได้ ไม่มีการเริ่มเดินอัตโนมัติจากการเลือกไฟล์')
        self.output_type=tk.StringVar(value=args.output_type)
        self.rviz=tk.BooleanVar(value=args.rviz)
        root.title('2 · ใช้ Knowledge | TurtleBot3')
        root.geometry(f'{min(1360,root.winfo_screenwidth()-60)}x{min(880,root.winfo_screenheight()-80)}');root.minsize(1000,700)
        style=ttk.Style(root)
        for name in ('TLabel','TButton','TCheckbutton'):
            style.configure(name,font=('Noto Sans Thai',-16))
        body=ttk.Frame(root,padding=18);body.pack(fill='both',expand=True)
        ttk.Label(body,text='นำทางจาก Knowledge · ดูเซนเซอร์และเส้นทางในหน้านี้',font=('Noto Sans Thai',-24)).pack(anchor='w')
        ttk.Label(body,textvariable=self.status,wraplength=1250).pack(side='bottom',fill='x',pady=8)
        split=ttk.Panedwindow(body,orient='horizontal');split.pack(fill='both',expand=True,pady=(10,0))
        left=ttk.Frame(split,width=360);right=ttk.Frame(split)
        split.add(left,weight=0);split.add(right,weight=1)
        self.stop_button=ttk.Button(left,text='หยุด / ปิดการทดลอง',command=self.stop)
        self.stop_button.pack(side='bottom',fill='x',ipady=10,pady=8)
        scroll=tk.Canvas(left,width=350,highlightthickness=0)
        bar=ttk.Scrollbar(left,orient='vertical',command=scroll.yview)
        bar.pack(side='right',fill='y');scroll.pack(fill='both',expand=True)
        scroll.configure(yscrollcommand=bar.set)
        controls=ttk.Frame(scroll,padding=(0,0,8,0));window=scroll.create_window(0,0,window=controls,anchor='nw')
        controls.bind('<Configure>',lambda event:scroll.configure(scrollregion=scroll.bbox('all')))
        scroll.bind('<Configure>',lambda event:scroll.itemconfigure(window,width=event.width))
        self.choose_button=ttk.Button(controls,text='เลือกไฟล์ knowledge.json',command=self.choose_file)
        self.choose_button.pack(fill='x',ipady=7,pady=(12,6))
        ttk.Entry(controls,textvariable=self.path,state='readonly',font=('DejaVu Sans',-13)).pack(fill='x')
        ttk.Label(controls,textvariable=self.details,wraplength=340,justify='left').pack(anchor='w',pady=12)
        self.demo_button=ttk.Button(controls,text='ทดลองจำลอง — ยังไม่มีหุ่น',command=lambda:self.start('demo'))
        self.demo_button.pack(fill='x',ipady=6)
        ttk.Label(controls,text='สนามสมมติใหม่ ไม่ใช่สิ่งกีดขวางในแมพ SLAM เดิม',wraplength=340).pack(anchor='w',pady=(3,10))
        ttk.Checkbutton(controls,text='เปิด RViz เสริม (ไม่จำเป็น)',variable=self.rviz).pack(anchor='w')
        row=ttk.Frame(controls);row.pack(fill='x',pady=7)
        ttk.Label(row,text='ชนิด /cmd_vel').pack(side='left',padx=(0,6))
        ttk.Combobox(row,textvariable=self.output_type,values=('auto','stamped','twist'),state='readonly',width=10).pack(side='left')
        self.dry_button=ttk.Button(controls,text='ตรวจเซนเซอร์ ROS — ไม่สั่งมอเตอร์',command=lambda:self.start('dry'))
        self.dry_button.pack(fill='x',ipady=5,pady=3)
        self.drive_button=ttk.Button(controls,text='เริ่มเดินจริง',command=lambda:self.start('drive'))
        self.drive_button.pack(fill='x',ipady=5,pady=3)
        ttk.Label(controls,text='เดินจริงต้องมี reference จาก Program 1\nหยุด Localization ก่อนเริ่ม และให้หุ่นอยู่นิ่งระหว่างส่งต่อ',wraplength=340).pack(anchor='w',pady=8)
        tabs=ttk.Notebook(right);tabs.pack(fill='both',expand=True)
        from core.live_view import LiveView
        self.live_view=LiveView(tabs);logs=ttk.Frame(tabs);conditions=ttk.Frame(tabs)
        tabs.add(self.live_view,text='สนาม / เซนเซอร์สด');tabs.add(conditions,text='เงื่อนไขการเดิน');tabs.add(logs,text='รายละเอียด / Log')
        self.conditions_text=tk.Text(conditions,state='disabled',font=('Noto Sans Thai',-15),wrap='word')
        condition_scroll=ttk.Scrollbar(conditions,orient='vertical',command=self.conditions_text.yview)
        condition_scroll.pack(side='right',fill='y');self.conditions_text.pack(fill='both',expand=True)
        self.conditions_text.configure(yscrollcommand=condition_scroll.set)
        self.last_condition_text=''
        self.console=tk.Text(logs,height=6,state='disabled',font=('DejaVu Sans Mono',-12),wrap='word')
        self.console.pack(fill='both',expand=True)
        root.protocol('WM_DELETE_WINDOW',self.close)
        self.sync();root.after(100,self.poll)

    def sync(self):
        busy=self.process is not None
        ready=self.knowledge is not None and not busy
        self.choose_button['state']='disabled' if busy else 'normal'
        self.demo_button['state']=self.dry_button['state']='normal' if ready else 'disabled'
        reference=self.knowledge.reference if self.knowledge else None
        can_drive=ready and reference.mode!='simulation' and reference.odom_pose is not None and bool(reference.odom_source)
        self.drive_button['state']='normal' if can_drive else 'disabled'
        self.stop_button['state']='normal' if busy and not self.stopping else 'disabled'

    def choose_file(self):
        from tkinter import filedialog,messagebox
        path=filedialog.askopenfilename(parent=self.root,title='เลือก Knowledge จาก Program 1',
            initialdir=str(DEFAULT.parent),filetypes=[('Knowledge JSON','*.json'),('All files','*')])
        if not path:return
        try:self.load_file(path)
        except (OSError,ValueError,TypeError) as exc:
            messagebox.showerror('เปิด Knowledge ไม่ได้',str(exc),parent=self.root)

    def load_file(self,path):
        if self.process is not None:raise ValueError('หยุดการทดลองก่อนเปลี่ยนไฟล์')
        selected=Path(path).expanduser().resolve()
        knowledge=Knowledge.load(selected)
        self.selected_path,self.knowledge=selected,knowledge
        self.live_view.set_knowledge(knowledge)
        self.update_conditions(dict(reason='ยังไม่มีผลตรวจ',movement_conditions=[]))
        self.path.set(str(selected))
        start=knowledge.start
        self.details.set(f'สนาม {knowledge.arena.width:.2f} × {knowledge.arena.height:.2f} m\n'
            f'Start ({start.x:.3f}, {start.y:.3f}) · yaw {start.yaw:.3f} rad · Goal ({knowledge.goal[0]:.3f}, {knowledge.goal[1]:.3f})\n'
            f'Robot ID: {knowledge.reference.robot_id} · โหมด: {knowledge.reference.mode}')
        self.status.set('โหลดแล้ว กดทดลองจำลองได้ · ไฟล์ simulation ใช้สั่งเดินจริงไม่ได้'
                        if knowledge.reference.mode=='simulation' else 'โหลดแล้ว ตรวจเซนเซอร์ก่อนสั่งเดินจริง')
        self.sync()

    def start(self,mode):
        from tkinter import messagebox
        if self.knowledge is None or self.process is not None:return
        try:
            # Re-read at launch so externally edited/deleted files fail before
            # creating a process. The child also validates the same schema.
            knowledge=Knowledge.load(self.selected_path)
            if knowledge.fingerprint!=self.knowledge.fingerprint:
                raise ValueError('ไฟล์เปลี่ยนหลังเลือก กรุณาเลือกไฟล์ใหม่')
            if mode=='drive' and (knowledge.reference.mode=='simulation' or knowledge.reference.odom_pose is None or not knowledge.reference.odom_source):
                raise ValueError('ไฟล์นี้ยังไม่มี reference จริง กลับไปเตรียมใน Program 1')
            if mode=='drive' and not -.1<=time.time()-knowledge.reference.captured_at<=1800:
                raise ValueError('reference เริ่มต้นหมดอายุ (เกิน 30 นาที หรือเวลาคลาดเคลื่อน) กลับ Program 1 รับ Start และส่งออกใหม่')
            command=[sys.executable,str(Path(__file__).resolve()),'--knowledge',str(self.selected_path)]
            self.update_conditions(dict(reason='รอผลตรวจรอบใหม่',movement_conditions=[]))
            if mode=='demo':command.append('--demo')
            else:
                self.live_view.reset()
                command.append('--ui-stream')
                command.extend(['--robot-id',knowledge.reference.robot_id,'--output-type',self.output_type.get(),
                    '--scan-topic',self.args.scan_topic,'--odom-topic',self.args.odom_topic,
                    '--command-topic',self.args.command_topic,'--output-topic',self.args.output_topic,'--log',str(self.args.log)])
                if self.rviz.get():command.append('--rviz')
                if mode=='drive':command.append('--drive')
            self.process=subprocess.Popen(command,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,bufsize=1)
            self.stopping=False
            self.status.set({'demo':'กำลังจำลอง เปิดหน้าต่างสนามสมมติแยก',
                'dry':'กำลังตรวจเซนเซอร์ ROS ไม่ส่งคำสั่งมอเตอร์',
                'drive':'เริ่มโหมดเดินจริง หุ่นจะเดินเมื่อเซนเซอร์และ guard พร้อม'}[mode])
            self.console.configure(state='normal');self.console.delete('1.0','end');self.console.configure(state='disabled')
            process=self.process
            def read_output():
                try:
                    for line in process.stdout:self.events.put(line)
                finally:process.stdout.close()
            threading.Thread(target=read_output,daemon=True).start()
            self.sync()
        except (OSError,ValueError,TypeError) as exc:
            self.status.set(str(exc));messagebox.showerror('เริ่มไม่ได้',str(exc),parent=self.root)

    def stop(self):
        if self.process is not None and self.process.poll() is None and not self.stopping:
            self.stopping=True;self.stop_time=time.monotonic()
            self.process.terminate()
            self.status.set('ส่งคำขอหยุดแล้ว กำลังรอโปรแกรมและ guard ปิด')
            self.sync()

    def update_conditions(self,data):
        from core.diagnostics import format_movement_conditions
        checks=data.get('movement_conditions')
        if not isinstance(checks,list) or any(not isinstance(item,dict) for item in checks):return
        failed=sum(item.get('passed') is False for item in checks)
        unknown=sum(item.get('passed') is None for item in checks)
        age=data.get('conditions_age',0.)
        text=(f"เหตุหลัก: {data.get('reason','—')}\nไม่ผ่าน {failed} ข้อ · ยังตรวจไม่ได้ {unknown} ข้อ · ผลตรวจอายุ {age:.1f} s\n"
            'ระบบต้องผ่าน; เดินปกติ / ก้าวตรวจ / หมุน เป็นทางเลือกคนละแบบ\n\n'+format_movement_conditions(checks))
        if text==self.last_condition_text:return
        position=self.conditions_text.yview()[0]
        self.conditions_text.configure(state='normal');self.conditions_text.delete('1.0','end')
        self.conditions_text.insert('end',text);self.conditions_text.configure(state='disabled')
        self.conditions_text.yview_moveto(position);self.last_condition_text=text

    def poll(self):
        try:
            while True:
                line=self.events.get_nowait()
                if line.startswith(RUNTIME_DATA_PREFIX):
                    if not self.stopping:
                        try:
                            data=json.loads(line[len(RUNTIME_DATA_PREFIX):])
                            self.live_view.update_data(data)
                            if self.live_view.data is data:self.update_conditions(data)
                        except (ValueError,TypeError):pass
                    continue
                self.console.configure(state='normal');self.console.insert('end',line)
                if int(self.console.index('end-1c').split('.')[0])>350:self.console.delete('1.0','100.0')
                self.console.see('end');self.console.configure(state='disabled')
                if line.startswith(RUNTIME_STATUS_PREFIX) and not self.stopping:
                    message=line[len(RUNTIME_STATUS_PREFIX):].strip()
                    if message:self.status.set(message)
                elif line.startswith('Error: ') and not self.stopping:
                    self.status.set(line.strip()[7:])
        except queue.Empty:pass
        if self.process is not None:
            result=self.process.poll()
            if result is not None:
                self.process.wait();self.process=None
                if self.stopping:self.status.set('หยุดแล้ว')
                elif result==0:self.status.set('จบการทดลองแล้ว')
                else:self.status.set(f'โปรแกรมหยุด (รหัส {result}): '+self.status.get())
                self.stopping=False;self.sync()
            elif self.stopping and time.monotonic()-self.stop_time>5:
                self.status.set('ยังรอโปรแกรมปิด หากใช้หุ่นจริงให้ตรวจว่าหุ่นหยุดแล้ว')
        if self.closing and self.process is None:self.root.destroy();return
        self.root.after(100,self.poll)

    def close(self):
        if self.process is None:self.root.destroy()
        else:self.closing=True;self.stop()


def launch_gui(args):
    import tkinter as tk
    root=tk.Tk();app=RunnerApp(root,args)
    if args.knowledge!=DEFAULT:
        from tkinter import messagebox
        try:app.load_file(args.knowledge)
        except (OSError,ValueError,TypeError) as exc:
            messagebox.showerror('เปิด Knowledge ไม่ได้',str(exc),parent=root)
    root.mainloop()


def show_demo(knowledge,result):
    import math
    import tkinter as tk
    from tkinter import ttk
    root=tk.Tk();root.title('2 · Knowledge demo — สนามสมมติ ไม่มี ROS');root.geometry('900x760')
    ttk.Label(root,text='สาธิตสนามสมมติ ไม่ใช่แมพ SLAM ของคุณ',font=('Noto Sans Thai',-22)).pack(pady=8)
    ttk.Label(root,text='ใช้ knowledge เดิมรับ LiDAR จำลอง · ไม่ส่งคำสั่งหุ่น',font=('Noto Sans Thai',-16)).pack()
    canvas=tk.Canvas(root,bg='#152331',highlightthickness=0);canvas.pack(fill='both',expand=True,padx=15,pady=10)
    state=tk.StringVar(value=f"{result['status']} · {result['time']:.1f}s · {result['distance']:.2f}m · collisions {result['collisions']}")
    ttk.Label(root,textvariable=state,font=('Noto Sans Thai',-16)).pack(pady=8)
    trace=result.get('trace',[]);index=[1]
    def draw():
        canvas.delete('all');a=knowledge.arena
        scale=max(1,min((canvas.winfo_width()-70)/a.width,(canvas.winfo_height()-70)/a.height))
        ox,oy=(canvas.winfo_width()-a.width*scale)/2,(canvas.winfo_height()+a.height*scale)/2
        def xy(x,y):return ox+x*scale,oy-y*scale
        canvas.create_rectangle(*xy(0,a.height),*xy(a.width,0),outline='#b4c7d1',width=2)
        for obstacle in result.get('demo_obstacles',[]):
            x,y,r=obstacle['x'],obstacle['y'],obstacle['radius']
            canvas.create_oval(*xy(x-r,y+r),*xy(x+r,y-r),fill='#5d6f7e')
        for label,point,color in (('Start',(knowledge.start.x,knowledge.start.y),'#20c4c9'),('Goal',knowledge.goal,'#ff9234')):
            x,y=xy(*point);canvas.create_oval(x-5,y-5,x+5,y+5,fill=color);canvas.create_text(x,y-16,text=label,fill=color,font=('DejaVu Sans',-14))
        current=trace[:index[0]]
        if len(current)>1:canvas.create_line(*[v for p in current for v in xy(p[0],p[1])],fill='#ee4f99',width=3)
        if current:
            x,y,yaw=current[-1];cx,cy=xy(x,y);r=knowledge.hardware.radius*scale
            canvas.create_oval(cx-r,cy-r,cx+r,cy+r,outline='#20c4c9',width=2)
            canvas.create_line(cx,cy,*xy(x+.18*math.cos(yaw),y+.18*math.sin(yaw)),fill='#20c4c9',width=2,arrow='last')
    def advance():
        if index[0]<len(trace):index[0]=min(len(trace),index[0]+4);draw()
        root.after(35,advance)
    canvas.bind('<Configure>',lambda e:draw())
    root.after(50,advance);root.mainloop()


def main(argv=None):
    arguments=list(sys.argv[1:] if argv is None else argv)
    args=parser().parse_args(arguments)
    if not arguments or args.gui:
        if args.inspect or args.demo or args.drive:
            parser().error('--gui cannot be combined with --inspect, --demo or --drive')
        launch_gui(args)
        return 0
    try:
        if args.log.expanduser().resolve()==args.knowledge.expanduser().resolve():
            raise ValueError('Log and knowledge must use different files')
        if args.log.suffix.lower()!='.jsonl':
            raise ValueError('Use .jsonl for runtime logs; do not select map/knowledge files')
        knowledge=Knowledge.load(args.knowledge)
        print(json.dumps({'knowledge':str(args.knowledge),'fingerprint':knowledge.fingerprint,
            'start':knowledge.document()['mission']['start'],'goal':knowledge.goal,
            'reference_mode':knowledge.reference.mode},ensure_ascii=False,indent=2),flush=True)
        if args.inspect:return 0
        if args.demo:
            from core.demo import run_demo
            result=run_demo(knowledge)
            print(json.dumps({k:v for k,v in result.items() if k not in ('trace','estimated_trace','decisions')},ensure_ascii=False,indent=2),flush=True)
            if not args.no_window:show_demo(knowledge,result)
            return 0
        from core.ros import run_navigation
        print('DRIVE: หุ่นจะเดินเมื่อข้อมูลพร้อม' if args.drive else 'DRY RUN: ไม่ส่งความเร็ว กำลังรอเซนเซอร์ ROS',flush=True)
        return run_navigation(knowledge,drive=args.drive,output_type=args.output_type,robot_id=args.robot_id,
            scan_topic=args.scan_topic,odom_topic=args.odom_topic,command_topic=args.command_topic,
            output_topic=args.output_topic,launch_rviz=args.rviz,log_path=args.log,ui_stream=args.ui_stream)
    except (OSError,ValueError,TypeError,ImportError,RuntimeError) as exc:
        print(f'Error: {exc}',file=sys.stderr)
        return 2


if __name__=='__main__':raise SystemExit(main())

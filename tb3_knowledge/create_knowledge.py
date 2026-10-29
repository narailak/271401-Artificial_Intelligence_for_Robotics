#!/usr/bin/env python3
"""Program 1: map and mission GUI; exports a single map-free knowledge file."""
from dataclasses import asdict, replace
import json
import math
from pathlib import Path
import queue
import signal
import sys
import threading
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

from PIL import Image, ImageTk
import numpy as np

from core.model import Arena, Hardware, InitialReference, Knowledge, Policy, Pose, SUMMARY_KEYS, digest, wrap
from core.maps import SlamMap, MissionWorld
from core.simulation import run_episode
from core.training import train, policy_candidates

OUTPUT = Path(__file__).resolve().parent/'output'


class App:
    def __init__(self, root):
        self.root = root
        self.slam = self.arena = self.world = self.policy = self.photo = None
        self.boundary_detection = None
        self.reference = InitialReference()
        self.hardware = Hardware()
        self.report = None
        self.trained_context = None
        self.captured_start = None
        self.busy = self.frozen = False
        self.monitor = self.localization = None
        self.pending_rectangle = []
        self.trace = []
        self.preview_result = None
        self.preview_uses_knowledge = None
        self.preview_context = None
        self.active_operation = None
        self.worker_messages = queue.Queue()
        self.image_transform = None
        self.arena_fields_dirty = False
        self.writing_arena = False
        self.mission_state = 'pending'
        self.status = tk.StringVar(value='เลือกแผนที่ กำหนดสนามและภารกิจ แล้วสร้าง knowledge')
        self.map_info = tk.StringVar(value='ยังไม่ได้โหลดแผนที่')
        self.summary = tk.StringVar(value='ยังไม่ได้สร้าง knowledge')
        self.action_feedback=tk.StringVar(value='ยังไม่ได้กดดำเนินการ')
        self.action_failed=False
        self.localization_report=tk.StringVar(value='ยังไม่ได้เปิด Localization')
        self._rviz_auto_closed=False
        self.localization_status = tk.StringVar(value='โหมดจำลอง: ไม่ต้องมีหุ่นหรือเปิด ROS')
        self.simulation_mode = tk.BooleanVar(value=True)
        self.robot_id = tk.StringVar(value='tb3')
        self.click_mode = tk.StringVar(value='goal')
        self.arena_vars = {k: tk.StringVar(value=v) for k,v in {'origin_x':'0','origin_y':'0','yaw':'0','width':'2.5','height':'2.5'}.items()}
        self.pose_vars = {k: tk.StringVar(value=v) for k,v in {'x':'0.4','y':'0.4','yaw':'0','gx':'2','gy':'2'}.items()}
        self.candidates = tk.StringVar(value='4')
        self.mission_feedback=tk.StringVar(value='ยังไม่ได้ตรวจ Start–Goal')
        self.simulation_summary=tk.StringVar(value='ยังไม่มีผลซิม เลือกปุ่มจำลองที่ต้องการ')
        self.simulation_method=tk.StringVar(value='ยังไม่ได้จำลอง')
        self.simulation_outcome=tk.StringVar(value='เลือกปุ่มจำลองที่ต้องการ')
        self.simulation_values={key:tk.StringVar(value='—') for key in ('time','distance','goal_error','collisions')}
        self.result_columns=None
        self._build()
        for variable in self.pose_vars.values():variable.trace_add('write',self.mission_edited)
        for variable in self.arena_vars.values():variable.trace_add('write',self.arena_edited)
        root.protocol('WM_DELETE_WINDOW', self.close)
        root.after(100, self.poll)

    def _build(self):
        root = self.root
        root.title('1 · สร้าง Knowledge | TurtleBot3')
        root.geometry(f'{min(1360,root.winfo_screenwidth()-50)}x{min(880,root.winfo_screenheight()-80)}')
        root.minsize(1000, 680)
        style=ttk.Style(root)
        for name in ('TLabel','TButton','TCheckbutton','TRadiobutton','TNotebook.Tab'):
            style.configure(name, font=('Noto Sans Thai',-15))
        for state,color in (('pending','#885800'),('valid','#167342'),('invalid','#b3261e')):
            style.configure(f'Mission{state}.TLabel',foreground=color)
        style.configure('ResultValue.TLabel',font=('Noto Sans Thai',-28,'bold'))
        style.configure('ResultCompactValue.TLabel',font=('Noto Sans Thai',-22,'bold'))
        style.configure('ResultMethod.TLabel',font=('Noto Sans Thai',-19,'bold'))
        style.configure('ResultOutcome.TLabel',font=('Noto Sans Thai',-17))
        style.configure('TLabelframe.Label',font=('Noto Sans Thai',-16,'bold'))
        outer=ttk.Frame(root,padding=14); outer.pack(fill='both',expand=True)
        ttk.Label(outer,text='แผนที่และภารกิจ → Knowledge ไฟล์เดียว',font=('Noto Sans Thai',-24)).pack(anchor='w')
        ttk.Label(outer,text='Program 1 · จำลองได้ทันที · notebook คำนวณทั้งหมด').pack(anchor='w',pady=(0,8))
        ttk.Label(outer,textvariable=self.action_feedback,wraplength=1250,foreground='#155b85').pack(fill='x',pady=5)
        ttk.Label(outer,textvariable=self.status,wraplength=1250).pack(side='bottom',fill='x',pady=(8,0))
        split=ttk.Panedwindow(outer,orient='horizontal'); split.pack(fill='both',expand=True)
        left=ttk.Frame(split,width=410); right=ttk.Frame(split)
        split.add(left,weight=0); split.add(right,weight=1)
        tabs=ttk.Notebook(left); tabs.pack(fill='both',expand=True,padx=(0,10))
        self.setup_tab=ttk.Frame(tabs,padding=10); self.train_tab=ttk.Frame(tabs,padding=10)
        tabs.add(self.setup_tab,text='1. แผนที่ / ภารกิจ'); tabs.add(self.train_tab,text='2. ฝึก / ส่งออก')
        self.tabs=tabs
        def scrollable(page):
            canvas=tk.Canvas(page,highlightthickness=0,width=380)
            scrollbar=ttk.Scrollbar(page,orient='vertical',command=canvas.yview)
            scrollbar.pack(side='right',fill='y');canvas.pack(side='left',fill='both',expand=True)
            canvas.configure(yscrollcommand=scrollbar.set)
            inner=ttk.Frame(canvas,padding=(0,0,4,0))
            window=canvas.create_window(0,0,window=inner,anchor='nw')
            inner.bind('<Configure>',lambda e:canvas.configure(scrollregion=canvas.bbox('all')))
            canvas.bind('<Configure>',lambda e:canvas.itemconfigure(window,width=e.width))
            def wheel(event):
                widget=root.winfo_containing(event.x_root,event.y_root)
                while widget is not None:
                    if widget is page:
                        direction=-1 if event.num==4 or event.num!=5 and event.delta>0 else 1
                        canvas.yview_scroll(3*direction,'units');return 'break'
                    widget=widget.master
            for sequence in ('<MouseWheel>','<Button-4>','<Button-5>'):
                root.bind_all(sequence,wheel,add='+')
            return inner
        s=scrollable(self.setup_tab)
        self.map_button=self.action_button(s,text='เลือก arena.yaml (+ PGM)',command=self.browse_map)
        self.map_button.pack(fill='x',ipady=5)
        ttk.Label(s,textvariable=self.map_info,wraplength=380).pack(anchor='w',pady=6)
        self.detect_button=self.action_button(s,text='ตรวจขอบสนามอัตโนมัติ',command=self.detect_boundary)
        self.detect_button.pack(fill='x',pady=4)
        ttk.Label(s,text='หาแนวผนังสี่ด้านจากแมพ แล้วใช้กรอบที่พบให้ทันที\nโหลดแมพจะตรวจให้แล้ว กดปุ่มนี้เมื่อต้องการตรวจใหม่',wraplength=370).pack(anchor='w',pady=(0,6))
        ttk.Label(s,text='กรอบสนาม: พิกัด Map [m], มุม [deg]').pack(anchor='w')
        for keys in (('origin_x','origin_y'),('width','height'),('yaw',)):
            row=ttk.Frame(s); row.pack(fill='x',pady=2)
            for key in keys:
                ttk.Label(row,text=key,width=9).pack(side='left')
                ttk.Entry(row,textvariable=self.arena_vars[key],width=10,font=('DejaVu Sans',-14)).pack(side='left',padx=(0,8))
        self.arena_button=self.action_button(s,text='ใช้ค่ากรอบที่แก้ไข',command=self.apply_arena)
        self.arena_button.pack(fill='x',pady=4)
        ttk.Label(s,text='ใช้เมื่อแก้ origin / ขนาด / yaw เอง · ล้างผลฝึกเดิมและเลือก Start–Goal ใหม่',wraplength=370).pack(anchor='w',pady=(0,4))
        ttk.Separator(s).pack(fill='x',pady=6)
        self.hardware_info=tk.StringVar(value=self.hardware_text())
        ttk.Label(s,textvariable=self.hardware_info,wraplength=370).pack(anchor='w',pady=4)
        self.hardware_button=self.action_button(s,text='ขนาดหุ่น / ระยะเผื่อ / ระยะถึง Goal',command=self.edit_hardware)
        self.hardware_button.pack(fill='x',pady=4)
        ttk.Label(s,text='ตั้งค่าก่อนตรวจ Start–Goal · เปลี่ยนค่าแล้วต้องตรวจและฝึกใหม่',wraplength=370).pack(anchor='w',pady=(0,6))
        row=ttk.Frame(s); row.pack(fill='x',pady=3)
        for label,value in (('ดูภาพ','view'),('Start','start'),('Goal','goal'),('กรอบ 3 จุด','rectangle')):
            button=ttk.Radiobutton(row,text=label,variable=self.click_mode,value=value,command=self.mode_changed)
            button.pack(side='left',padx=3)
            if value=='start':self.start_mode_button=button
        ttk.Label(s,text='กรอบ 3 จุด: มุมกำเนิด, ปลายแกน x, จุดด้าน +y\nตัวเลข Start/Goal ด้านล่างเป็นพิกัดในสนาม',wraplength=380).pack(anchor='w',pady=3)
        self.start_entries=[]
        for label, keys in (('Start [m, deg]',('x','y','yaw')),('Goal [m]',('gx','gy'))):
            row=ttk.Frame(s); row.pack(fill='x',pady=3)
            ttk.Label(row,text=label,width=15).pack(side='left')
            for key in keys:
                entry=ttk.Entry(row,textvariable=self.pose_vars[key],width=7,font=('DejaVu Sans',-14))
                entry.pack(side='left',padx=2)
                if key in ('x','y','yaw'):self.start_entries.append(entry)
        self.mission_button=self.action_button(s,text='ตรวจและใช้ Start–Goal',command=self.apply_mission)
        self.mission_button.pack(fill='x',pady=4)
        self.mission_feedback_label=ttk.Label(s,textvariable=self.mission_feedback,style='Missionpending.TLabel',wraplength=370)
        self.mission_feedback_label.pack(fill='x',pady=(0,8))
        ttk.Checkbutton(s,text='โหมดจำลอง — ยังไม่มีหุ่น',variable=self.simulation_mode,command=self.mode_changed).pack(anchor='w',pady=3)
        row=ttk.Frame(s); row.pack(fill='x')
        ttk.Label(row,text='ชื่อหุ่น (Robot ID)').pack(side='left')
        ttk.Entry(row,textvariable=self.robot_id,width=17,font=('DejaVu Sans',-14)).pack(side='left',padx=6)
        ttk.Label(s,text='ชื่ออ้างอิงใน knowledge เช่น tb3 ใช้ให้ตรงกันใน Program 2\nไม่ใช่ IP หรือ ROS namespace · มีหุ่นตัวเดียวใช้ tb3 ได้',wraplength=370).pack(anchor='w',pady=4)
        self.action_button(s,text='เปิด Localization / หาตำแหน่งอัตโนมัติ',command=self.start_localization).pack(fill='x',pady=3)
        self.action_button(s,text='ค้นหาตำแหน่งใหม่ทั่วแมพ',command=self.retry_localization).pack(fill='x',pady=3)
        ttk.Label(s,text='ค้นหาทั่วแมพ หรือใช้ 2D Pose Estimate ใน RViz\nเทียบ LiDAR โดยหุ่นอยู่นิ่ง รอพร้อม=True แล้วกดรับ Start',wraplength=370).pack(anchor='w',pady=4)
        self.action_button(s,text='เชื่อม ROS / รับตำแหน่ง Start',command=self.capture_start).pack(fill='x',pady=3)
        self.action_button(s,text='หยุด Localization ก่อนเปิด Program 2',command=self.stop_localization).pack(fill='x',pady=3)
        ttk.Label(s,textvariable=self.localization_status,wraplength=380).pack(anchor='w',pady=4)
        t=scrollable(self.train_tab)
        row=ttk.Frame(t);row.pack(fill='x',pady=4)
        ttk.Label(row,text='จำนวนชุดค่าที่จะลอง').pack(side='left')
        self.candidate_box=ttk.Combobox(row,textvariable=self.candidates,values=tuple(str(i) for i in range(1,len(policy_candidates())+1)),state='readonly',width=5,font=('DejaVu Sans',-14))
        self.candidate_box.pack(side='right')
        ttk.Label(t,text=f'มีตัวเลือกเตรียมไว้ {len(policy_candidates())} ชุด แต่ละชุดกำหนดพฤติกรรมการเดินต่างกัน\nเลือก 4 = ลอง 4 ชุดแรก ชุดละ 2 รอบ ไม่ใช่พารามิเตอร์ 4 ตัว',wraplength=370).pack(anchor='w',pady=5)
        self.action_button(t,text='ดูชุดค่าที่เตรียมไว้',command=self.show_candidates).pack(fill='x',pady=3)
        ttk.Label(t,text='ทดลองให้ครบจำนวนชุดที่เลือก แล้วประเมินชุดที่เลือกด้วย seed ใหม่',wraplength=370).pack(anchor='w',pady=6)
        self.train_button=self.action_button(t,text='สร้าง Knowledge',command=self.build_knowledge)
        self.train_button.pack(fill='x',ipady=8,pady=4)
        self.progress=ttk.Progressbar(t,mode='indeterminate'); self.progress.pack(fill='x',pady=6)
        ttk.Label(t,textvariable=self.summary,wraplength=370).pack(anchor='w',pady=7)
        self.baseline_button=self.action_button(t,text='จำลอง: ค่าเริ่มต้น',command=lambda:self.preview(False))
        self.baseline_button.pack(fill='x',pady=3)
        self.knowledge_button=self.action_button(t,text='จำลอง: ใช้ Knowledge',command=lambda:self.preview(True))
        self.knowledge_button.pack(fill='x',pady=3)
        ttk.Label(t,text='ใช้ตัวนำทางเดียวกัน ต่างกันที่ชุดค่าพารามิเตอร์\nกดปุ่มใด จะแสดงเส้นทาง เวลา และระยะทางเฉพาะรอบนั้น',wraplength=370).pack(anchor='w',pady=5)
        self.export_button=self.action_button(t,text='บันทึก knowledge.json',command=self.export)
        self.export_button.pack(fill='x',ipady=8,pady=8)
        self.action_button(t,text='ล้างผลซิม / เส้นทาง',command=self.clear_trace).pack(fill='x',pady=3)
        ttk.Label(t,text='หลังส่งออกจะล็อก session\nเปลี่ยนสนาม/ภารกิจเพื่อเริ่มใหม่\nส่งออกโหมดจำลองใช้สั่งหุ่นจริงไม่ได้',wraplength=370).pack(anchor='w',pady=8)
        right.columnconfigure(0,weight=1)
        right.rowconfigure(0,weight=1)
        self.canvas=tk.Canvas(right,bg='#142230',highlightthickness=0)
        self.canvas.grid(row=0,column=0,sticky='nsew')
        self.canvas.bind('<Configure>',lambda e:self.draw())
        self.canvas.bind('<Button-1>',self.click)
        self.legend_label=ttk.Label(right,text='เทา = unknown (ผ่านไม่ได้) · ฟ้า = Start · ส้ม = Goal\nวงทึบ = ขนาดหุ่น · วงประ = รวมระยะเผื่อ · เส้นทาง = ผลซิม',wraplength=740)
        self.legend_label.grid(row=1,column=0,sticky='ew',pady=6)
        right.bind('<Configure>',lambda e:self.legend_label.configure(wraplength=max(250,e.width-10)))
        self.report_panel=ttk.LabelFrame(right,text='เงื่อนไข Localize · เลื่อนดูรายละเอียดได้',padding=4)
        self.report_panel.grid(row=3,column=0,sticky='ew',pady=4)
        self.condition_text=tk.Text(self.report_panel,height=6,width=1,wrap='word',font=('Noto Sans Thai',-14))
        report_scroll=ttk.Scrollbar(self.report_panel,orient='vertical',command=self.condition_text.yview)
        report_scroll.pack(side='right',fill='y');self.condition_text.pack(fill='x',expand=True)
        self.condition_text.configure(yscrollcommand=report_scroll.set,state='disabled')
        self.localization_report.trace_add('write',lambda *args:self.render_localization_report())
        self.report_panel.grid_remove()
        self.results_panel=ttk.LabelFrame(right,text='ผลการจำลองล่าสุด',padding=(12,8))
        self.results_panel.grid(row=2,column=0,sticky='ew',pady=(0,4))
        heading=ttk.Frame(self.results_panel);heading.pack(fill='x',pady=(0,5))
        ttk.Label(heading,textvariable=self.simulation_method,style='ResultMethod.TLabel').pack(side='left')
        self.outcome_label=ttk.Label(heading,textvariable=self.simulation_outcome,style='ResultOutcome.TLabel')
        self.outcome_label.pack(side='right',padx=(10,0))
        self.metrics_frame=ttk.Frame(self.results_panel);self.metrics_frame.pack(fill='x')
        self.metric_widgets={}
        self.metric_value_labels={}
        for key,label in (('time','เวลา (วินาที)'),('distance','ระยะทาง (เมตร)'),('goal_error','ห่าง Goal (เมตร)'),('collisions','การชน (ครั้ง)')):
            frame=ttk.Frame(self.metrics_frame)
            ttk.Label(frame,text=label,anchor='center').pack(fill='x')
            value_label=ttk.Label(frame,textvariable=self.simulation_values[key],style='ResultValue.TLabel',anchor='center')
            value_label.pack(fill='x');self.metric_value_labels[key]=value_label
            self.metric_widgets[key]=frame
        self.metrics_frame.bind('<Configure>',self.layout_results)
        ttk.Label(self.results_panel,text='เวลาเป็นเวลาการเคลื่อนที่ในซิม · แสดงเฉพาะรอบล่าสุด').pack(anchor='w',pady=(3,0))
        self.sync()

    def render_localization_report(self):
        position=self.condition_text.yview()[0]
        self.condition_text.configure(state='normal');self.condition_text.delete('1.0','end')
        self.condition_text.insert('end',self.localization_report.get())
        self.condition_text.configure(state='disabled');self.condition_text.yview_moveto(position)

    def action_button(self,parent,*,text,command,**kwargs):
        return ttk.Button(parent,text=text,command=lambda:self.run_action(text,command),**kwargs)

    def run_action(self,label,command):
        if self.busy and label!='หยุด Localization ก่อนเปิด Program 2':
            self.action_feedback.set(f'ไม่ผ่าน: {label} · กำลังคำนวณ กรุณารอให้จบก่อน')
            return False
        self.action_failed=False
        self.status.set(f'กำลังดำเนินการ: {label}')
        self.action_feedback.set(self.status.get());self.root.update_idletasks()
        try:
            result=command()
            if result=='cancelled':self.status.set('ผู้ใช้ยกเลิกการดำเนินการ')
            elif self.status.get()==f'กำลังดำเนินการ: {label}':self.status.set('ดำเนินการเรียบร้อย')
            if self.action_failed or result is False:outcome='ไม่ผ่าน'
            elif result=='cancelled':outcome='ยกเลิก'
            elif self.busy:outcome='กำลังทำงาน'
            else:outcome='ผ่าน'
            self.action_feedback.set(f'{outcome}: {label}')
        except Exception as exc:self.error(exc)

    def layout_results(self,event):
        self.outcome_label.configure(wraplength=max(150,event.width//2))
        columns=4 if event.width>=720 else 2
        if columns==self.result_columns:return
        self.result_columns=columns
        for label in self.metric_value_labels.values():
            label.configure(style='ResultValue.TLabel' if columns==4 else 'ResultCompactValue.TLabel')
        for index in range(4):
            self.metrics_frame.columnconfigure(index,weight=1 if index<columns else 0,uniform='metric' if index<columns else '')
        for index,widget in enumerate(self.metric_widgets.values()):
            widget.grid(row=index//columns,column=index%columns,sticky='ew',padx=6,pady=3)

    def hardware_text(self):
        h=self.hardware
        return f'รัศมีหุ่น {h.radius:.3f} m + ระยะเผื่อ {h.safety_margin:.3f} m\nระยะถึง Goal {h.goal_tolerance:.2f} m · จำกัด {h.max_speed:.2f} m/s\nค่าเริ่มต้น Burger / LDS-02 · ต้องวัดอุปกรณ์เสริมและสอบเทียบการเบรกจริง'

    def sync(self):
        enabled=not self.busy
        self.start_mode_button['state']='normal' if enabled and self.simulation_mode.get() else 'disabled'
        for entry in self.start_entries:entry['state']='disabled' if not enabled else 'normal' if self.simulation_mode.get() else 'readonly'
        self.map_button['state']='normal' if enabled else 'disabled'
        self.detect_button['state']='normal' if enabled and self.slam is not None else 'disabled'
        self.arena_button['state']='normal' if enabled and self.slam is not None else 'disabled'
        self.mission_button['state']='normal' if enabled and self.world is not None else 'disabled'
        self.hardware_button['state']='normal' if enabled else 'disabled'
        self.baseline_button['state']='normal' if enabled and self.world is not None else 'disabled'
        self.knowledge_button['state']='normal' if enabled and self.policy is not None else 'disabled'
        self.train_button['state']='normal' if enabled and self.world is not None and not self.frozen else 'disabled'
        self.export_button['state']='normal' if enabled and self.policy is not None else 'disabled'

    def invalidate(self,keep_reference=False):
        self.policy=self.report=None; self.frozen=False; self.trained_context=None
        if not keep_reference:
            self.reference=InitialReference();self.captured_start=None
        self.clear_trace(); self.summary.set('ภารกิจหรือสนามเปลี่ยน ต้องสร้าง knowledge ใหม่')
        self.set_mission_feedback('pending','ยังไม่ได้ตรวจ Start–Goal สำหรับค่าปัจจุบัน')
        self.sync()

    def browse_map(self):
        if self.busy:return
        path=filedialog.askopenfilename(parent=self.root,title='เลือกแผนที่ SLAM',filetypes=[('Map YAML','*.yaml *.yml')])
        if not path:return 'cancelled'
        try:
            self.load_map(path)
        except FileNotFoundError:
            image=filedialog.askopenfilename(parent=self.root,title='หา PGM ไม่พบ เลือกไฟล์ภาพ',filetypes=[('Map PGM','*.pgm')])
            if image:
                try:self.load_map(path,image)
                except Exception as exc:self.error(exc)
            else:return 'cancelled'
        except Exception as exc:self.error(exc)

    def load_map(self,path,image_path=None):
        slam=SlamMap.load(path,image_path)
        try:
            detection=slam.detect_arena()
            arena=detection.arena
            message=f'ตรวจแนวผนังสี่ด้านแล้ว: {arena.width:.2f} × {arena.height:.2f} m · มุม {math.degrees(arena.yaw):.2f}°'
        except ValueError as exc:
            detection=None;arena=slam.known_arena()
            message=f'ตรวจผนังไม่ชัด: {exc} · ใช้กรอบพื้นที่สำรวจชั่วคราว ปรับกรอบ 3 จุดก่อนฝึก'
        # A new imported map must not retain the old map->odom/AMCL session.
        if self.monitor is not None or self.localization is not None:self.stop_localization()
        self.slam,self.arena=slam,arena
        self.boundary_detection=detection
        self.world=MissionWorld(slam,arena)
        self.invalidate()
        self.write_arena(arena)
        self.map_info.set(f'{Path(path).name} · {slam.width} × {slam.height} px · {slam.resolution:g} m/pixel\n{message}')
        self.suggest_mission()
        self.status.set(message+' · ตรวจกรอบสีเขียวบนภาพก่อนเลือกภารกิจ')
        self.draw();self.sync()

    def detect_boundary(self):
        if self.busy or self.slam is None:self.status.set('ต้องโหลดแมพและรอการคำนวณจบก่อน');return False
        try:
            detection=self.slam.detect_arena()
            self.write_arena(detection.arena)
            if not self.apply_arena():return
            self.boundary_detection=detection
            arena=detection.arena
            message=f'ตรวจขอบสี่ด้าน: {arena.width:.2f} × {arena.height:.2f} m · มุม {math.degrees(arena.yaw):.2f}°'
            self.map_info.set(f'{Path(self.slam.yaml_path).name}\n{message}')
            self.status.set(message+' · ค่า origin/ขนาด/yaw ถูกเติมแล้ว เลือกภารกิจใหม่ก่อนฝึก')
        except ValueError as exc:
            self.status.set(f'ตรวจขอบไม่ได้: {exc} · คงกรอบปัจจุบันไว้');return False

    def write_arena(self,arena):
        self.writing_arena=True
        try:
            for key in self.arena_vars:
                value=math.degrees(arena.yaw) if key=='yaw' else getattr(arena,key)
                self.arena_vars[key].set(f'{value:.4f}')
        finally:self.writing_arena=False
        self.arena_fields_dirty=False

    def arena_edited(self,*args):
        if self.writing_arena:return
        self.arena_fields_dirty=True
        self.set_mission_feedback('pending','กรอบยังไม่ได้ใช้: กดใช้ค่ากรอบที่แก้ไข แล้วตรวจ Start–Goal ใหม่')
        if self.preview_result is not None:self.clear_trace()

    def read_arena(self):
        a={k:float(v.get()) for k,v in self.arena_vars.items()}
        a['yaw']=math.radians(a['yaw'])
        return Arena(**a)

    def apply_arena(self):
        if self.busy or self.slam is None:self.status.set('ต้องโหลดแมพและรอการคำนวณจบก่อน');return False
        try:
            arena=self.read_arena()
            self.arena=arena;self.world=MissionWorld(self.slam,arena)
            self.arena_fields_dirty=False
            self.boundary_detection=None
            self.invalidate();self.suggest_mission();self.draw()
            self.status.set('ใช้ค่ากรอบที่แก้ไขแล้ว ตรวจตำแหน่ง Start–Goal ก่อนฝึก')
            return True
        except Exception as exc:
            self.set_mission_feedback('invalid',f'ยังใช้ภารกิจไม่ได้: {exc}')
            self.error(exc)
            return False

    def suggest_mission(self):
        a,w,h=self.arena,self.world,self.hardware
        start=None
        for distance in (.28,.35,.4,.45,.5,.55,.6,.7,.8):
            if 2*distance>=min(a.width,a.height):continue
            corners=((distance,distance),(a.width-distance,distance),
                     (distance,a.height-distance),(a.width-distance,a.height-distance))
            if all(w.free(x,y,h.safe_radius) for x,y in corners):
                start=Pose(distance,distance,0);break
        if start is None:
            for y in np.arange(.2,a.height,.15):
                for x in np.arange(.2,a.width,.15):
                    if w.free(float(x),float(y),h.safe_radius):
                        start=Pose(float(x),float(y),0);break
                if start:break
        if start is None:
            raise ValueError('ไม่มีพื้นที่พอวางหุ่นในกรอบนี้ ตรวจ crop และระยะเผื่อ')
        target=None
        for x,y in ((a.width-start.x,a.height-start.y),(a.width-start.x,start.y),(start.x,a.height-start.y)):
            if math.dist((start.x,start.y),(x,y))>=.5 and w.free(x,y,h.safe_radius) and w.reachable(start,(x,y),h.safe_radius):
                target=Pose(x,y);break
        if target is None:
            pool=[(float(x),float(y)) for y in np.arange(.2,a.height,.1) for x in np.arange(.2,a.width,.1)
                  if math.dist((start.x,start.y),(x,y))>=.5 and w.free(float(x),float(y),h.safe_radius)]
            pool.sort(key=lambda p:math.dist((start.x,start.y),p),reverse=True)
            for p in pool:
                if w.reachable(start,p,h.safe_radius):target=Pose(*p);break
        if target is None:
            raise ValueError('ไม่พบ Goal ที่ห่าง Start อย่างน้อย 0.5 m และเดินเชื่อมกันได้ ตรวจสนาม')
        self.set_pose(Pose(start.x,start.y,math.atan2(target.y-start.y,target.x-start.x)),(target.x,target.y))

    def set_pose(self,start,goal=None):
        self.pose_vars['x'].set(f'{start.x:.4f}');self.pose_vars['y'].set(f'{start.y:.4f}')
        self.pose_vars['yaw'].set(f'{math.degrees(start.yaw):.3f}')
        if goal is not None:
            self.pose_vars['gx'].set(f'{goal[0]:.4f}');self.pose_vars['gy'].set(f'{goal[1]:.4f}')

    def mission(self):
        v={k:float(var.get()) for k,var in self.pose_vars.items()}
        if not all(math.isfinite(value) for value in v.values()):raise ValueError('Start–Goal ต้องเป็นตัวเลขที่มีค่าจำกัด')
        return Pose(v['x'],v['y'],math.radians(v['yaw'])),(v['gx'],v['gy'])

    def validate_mission(self):
        if self.world is None:raise ValueError('เลือกแผนที่ก่อน')
        if self.arena_fields_dirty:raise ValueError('แก้ค่ากรอบแล้ว กดใช้ค่ากรอบที่แก้ไขก่อนตรวจ Start–Goal')
        start,goal=self.mission()
        Knowledge(self.arena,start,goal,hardware=self.hardware)
        r=self.hardware.safe_radius
        if not self.world.free(start.x,start.y,r):raise ValueError('Start อยู่ในกำแพง/unknown หรือชิดสิ่งกีดขวางเกินไป')
        if not self.world.free(*goal,r):raise ValueError('Goal วางหุ่นไม่ได้ ตรวจจุดและระยะเผื่อ')
        if not self.world.reachable(start,goal,r):raise ValueError('Start–Goal ไม่มีทางเชื่อมที่ตัวหุ่นผ่านได้')
        return start,goal

    def apply_mission(self):
        if self.busy:return
        try:
            if not self.simulation_mode.get() and self.captured_start is None:
                raise ValueError('ต้อง Localize และรับ Start จาก ROS ก่อนตรวจภารกิจจริง')
            self.validate_mission();self.invalidate(keep_reference=True);self.draw()
            self.set_mission_feedback('valid','ผ่าน: Start และ Goal วางหุ่นได้ และมีทางเชื่อมที่ตัวหุ่นผ่านได้\nพร้อมสร้าง Knowledge ในแท็บ 2')
            self.status.set('ตรวจ Start–Goal ผ่านแล้ว ไปแท็บ 2 เพื่อสร้าง knowledge')
        except Exception as exc:
            self.set_mission_feedback('invalid',f'ไม่ผ่าน: {exc}')
            self.error(exc)

    def set_mission_feedback(self,state,text):
        self.mission_state=state
        self.mission_feedback.set(text)
        self.mission_feedback_label.configure(style=f'Mission{state}.TLabel')

    def mode_changed(self):
        if not self.busy:
            self.pending_rectangle=[]
            if self.simulation_mode.get():self.localization_status.set('โหมดจำลอง: ไม่ส่งคำสั่งหุ่น')
            else:self.localization_status.set('ใช้หุ่นจริง: ต้องรับ Start จาก localization และ capture odometry ก่อนส่งออก')
            self.draw();self.sync()
            self.action_feedback.set('ผ่าน: เปลี่ยนโหมดแล้ว · '+self.localization_status.get())

    def click(self,event):
        if self.busy or self.slam is None or self.image_transform is None:return
        ox,oy,scale=self.image_transform
        u,v=(event.x-ox)/scale-.5,(event.y-oy)/scale-.5
        if not (-.5<=u<self.slam.width-.5 and -.5<=v<self.slam.height-.5):return
        map_x,map_y=self.slam.pixel_to_map(u,v)
        mode=self.click_mode.get()
        if mode=='rectangle':
            self.pending_rectangle.append((map_x,map_y))
            if len(self.pending_rectangle)==3:
                o,x,y=self.pending_rectangle
                width=math.dist(o,x)
                if width<.3:self.pending_rectangle=[];self.status.set('แกน x สั้นเกินไป');return
                angle=math.atan2(x[1]-o[1],x[0]-o[0])
                height=-math.sin(angle)*(y[0]-o[0])+math.cos(angle)*(y[1]-o[1])
                try:
                    self.write_arena(Arena(width,height,o[0],o[1],angle));self.apply_arena()
                except Exception as exc:self.error(exc)
                self.pending_rectangle=[]
            else:self.status.set(f'เลือกกรอบแล้ว {len(self.pending_rectangle)}/3 จุด')
        elif mode in ('start','goal'):
            x,y=self.arena.map_to_local(map_x,map_y)
            if mode=='start' and not self.simulation_mode.get():
                self.status.set('ไม่ผ่าน: Start จริงต้องรับจาก ROS หลัง Localize ไม่ใช้การคลิกตำแหน่งคร่าว ๆ')
                self.action_feedback.set(self.status.get());return
            if not self.arena.contains(x,y,self.hardware.safe_radius):self.status.set('จุดชิดขอบหรืออยู่นอกกรอบสนาม');return
            if mode=='start':
                self.pose_vars['x'].set(f'{x:.4f}');self.pose_vars['y'].set(f'{y:.4f}')
            else:
                self.pose_vars['gx'].set(f'{x:.4f}');self.pose_vars['gy'].set(f'{y:.4f}')
            self.invalidate(keep_reference=mode=='goal')
            self.status.set(f'ผ่าน: เลือก {mode} แล้ว ต้องตรวจ Start–Goal อีกครั้ง')
        self.draw()

    def draw(self):
        c=self.canvas;c.delete('all')
        if self.slam is None:
            c.create_text(c.winfo_width()/2,c.winfo_height()/2,text='เลือก YAML/PGM เพื่อดูสนาม',fill='#b9cad8',font=('Noto Sans Thai',-20));return
        colors=np.array([[235,240,242],[29,40,50],[115,130,140]],dtype=np.uint8)
        scale=max(.1,min((c.winfo_width()-36)/self.slam.width,(c.winfo_height()-36)/self.slam.height))
        width,height=max(1,int(self.slam.width*scale)),max(1,int(self.slam.height*scale))
        ox,oy=(c.winfo_width()-width)/2,(c.winfo_height()-height)/2
        self.image_transform=(ox,oy,scale)
        self.photo=ImageTk.PhotoImage(Image.fromarray(colors[self.slam.states]).resize((width,height),Image.Resampling.NEAREST))
        c.create_image(ox,oy,image=self.photo,anchor='nw')
        def map_point(x,y):
            u,v=self.slam.map_to_pixel(x,y);return ox+(u+.5)*scale,oy+(v+.5)*scale
        def point(x,y):return map_point(*self.arena.local_to_map(x,y))
        if self.arena:
            corners=[map_point(x,y) for x,y in self.arena.corners()]
            c.create_line(*[v for p in corners+[corners[0]] for v in p],fill='#00d7c4',width=3)
            c.create_text(*corners[0],text='  (0,0)',anchor='nw',fill='#00a897',font=('DejaVu Sans',-14))
            for index,p in enumerate(self.pending_rectangle):
                x,y=map_point(*p);c.create_oval(x-4,y-4,x+4,y+4,fill='#ff5788');c.create_text(x,y-12,text=str(index+1),fill='red')
            try:
                start,goal=self.mission()
                if len(self.trace)>1:
                    c.create_line(*[v for p in self.trace for v in point(*p)],fill='#dd3974' if self.preview_uses_knowledge else '#3b87e0',width=3,tags='simulation_path')
                for label,p,color in (('Start',(start.x,start.y),'#02b5d4'),('Goal',goal,'#ff8d26')):
                    x,y=point(*p)
                    radius=self.hardware.radius/self.slam.resolution*scale
                    safe_radius=self.hardware.safe_radius/self.slam.resolution*scale
                    c.create_oval(x-safe_radius,y-safe_radius,x+safe_radius,y+safe_radius,outline=color,width=2,dash=(5,4),tags=f'{label.lower()}_margin')
                    c.create_oval(x-radius,y-radius,x+radius,y+radius,outline=color,width=3,tags=f'{label.lower()}_body')
                    c.create_oval(x-3,y-3,x+3,y+3,fill=color,outline=color)
                    title=label
                    if label=='Start' and not self.simulation_mode.get() and self.captured_start is None:
                        title='Start (ยังไม่ได้รับจาก ROS)'
                    c.create_text(x,y-safe_radius-12,text=title,fill=color,font=('DejaVu Sans',-14))
                if self.simulation_mode.get() or self.captured_start is not None:
                    x,y=point(start.x,start.y);ex,ey=point(start.x+.18*math.cos(start.yaw),start.y+.18*math.sin(start.yaw))
                    c.create_line(x,y,ex,ey,fill='#0084a0',width=3,arrow='last',tags='start_heading')
            except (ValueError,KeyError):pass

    def show_candidates(self):
        names=('ค่าเริ่มต้น','อ้อมอีกด้าน ให้พื้นที่ว่างสำคัญขึ้น','อ้อมอีกด้าน ลดน้ำหนัก clearance',
               'เพิ่ม clearance และลดการย้อนทางเดิม','มองใกล้และแก้ติดวนเร็ว','มุ่ง Goal มากขึ้น มองไกล',
               'รักษาทิศและด้านอ้อมนานขึ้น','ลดการวนและตรวจความคืบหน้าเร็ว')
        window=tk.Toplevel(self.root);window.title('ชุดค่าที่ใช้ค้นหา Knowledge');window.geometry('820x560')
        ttk.Label(window,text='เลือก N = ลอง N ชุดแรกให้ครบ แต่ละชุดทดสอบ 2 seed',wraplength=770).pack(anchor='w',padx=14,pady=10)
        text=tk.Text(window,wrap='word',font=('Noto Sans Thai',-15),padx=12,pady=8)
        text.pack(fill='both',expand=True,padx=12,pady=(0,12))
        base=asdict(Policy())
        for index,candidate in enumerate(policy_candidates()):
            changed={key:value for key,value in asdict(candidate).items() if value!=base[key]}
            text.insert('end',f'{index+1}. {names[index]}\n')
            text.insert('end',(', '.join(f'{key}={value}' for key,value in changed.items()) if changed else 'ใช้ค่าเริ่มต้นทั้งหมด')+'\n\n')
        text.configure(state='disabled')

    def edit_hardware(self):
        if self.busy:return
        window=tk.Toplevel(self.root);window.title('ค่าฮาร์ดแวร์ (ต้องวัดจริง)');window.transient(self.root);window.grab_set()
        fields={}
        for key in ('radius','safety_margin','max_speed','max_angular_speed','braking','goal_tolerance'):
            row=ttk.Frame(window,padding=6);row.pack(fill='x')
            ttk.Label(row,text=key,width=22).pack(side='left')
            var=tk.StringVar(value=str(getattr(self.hardware,key)));fields[key]=var
            ttk.Entry(row,textvariable=var,width=12,font=('DejaVu Sans',-15)).pack(side='right')
        def apply():
            try:
                self.hardware=replace(self.hardware,**{k:float(v.get()) for k,v in fields.items()})
                self.hardware_info.set(self.hardware_text());self.invalidate();window.destroy()
                self.tabs.select(self.setup_tab)
                self.status.set('ค่าฮาร์ดแวร์เปลี่ยน ตรวจและใช้ Start–Goal อีกครั้งก่อนฝึก')
            except ValueError as exc:self.error(exc)
        self.action_button(window,text='ใช้ค่า / ต้องสร้าง knowledge ใหม่',command=apply).pack(fill='x',padx=6,pady=6)

    def build_knowledge(self):
        if self.busy or self.frozen:return
        try:
            start,goal=self.validate_mission()
            candidates=int(self.candidates.get())
            if not 1<=candidates<=len(policy_candidates()):raise ValueError('ใช้ 1–8 ชุดค่า')
            if not self.simulation_mode.get():
                if self.reference.mode!='localized' or self.captured_start is None:
                    raise ValueError('รับตำแหน่งจาก localization ก่อน หรือเลือกโหมดจำลอง')
                if math.dist((start.x,start.y),(self.captured_start.x,self.captured_start.y))>.01 or abs(wrap(start.yaw-self.captured_start.yaw))>.02:
                    raise ValueError('Start เปลี่ยนจากตำแหน่งที่ capture รับตำแหน่งจาก ROS ใหม่')
        except Exception as exc:self.error(exc);return
        self.clear_trace()
        self.active_operation='training'
        self.busy=True;self.sync();self.progress.configure(mode='indeterminate',value=0);self.progress.start();self.status.set('กำลังทดลองพารามิเตอร์ในซิม');self.tabs.select(self.train_tab)
        world,arena,hardware=self.world,self.arena,self.hardware
        context=self.context_fingerprint(start,goal)
        def worker():
            try:
                policy,report=train(world,arena,start,goal,hardware,candidate_limit=candidates,
                    progress=lambda text:self.worker_messages.put(('progress',str(text))))
                self.worker_messages.put(('trained',(policy,report,context)))
            except Exception as exc:self.worker_messages.put(('error',str(exc)))
        threading.Thread(target=worker,daemon=True).start()

    def preview(self,use_knowledge):
        if self.busy:return
        try:
            start,goal=self.validate_mission()
            needs_knowledge=use_knowledge
            if needs_knowledge and self.policy is None:raise ValueError('สร้าง knowledge ก่อน')
            context=self.context_fingerprint(start,goal)
            if needs_knowledge and context!=self.trained_context:
                raise ValueError('ภารกิจ/ค่าฮาร์ดแวร์เปลี่ยน ต้องสร้าง knowledge ใหม่')
        except Exception as exc:self.error(exc);return
        self.clear_trace()
        self.active_operation='preview'
        self.simulation_method.set('ใช้ Knowledge' if use_knowledge else 'ค่าเริ่มต้น')
        self.simulation_outcome.set('กำลังจำลอง…')
        self.outcome_label.configure(foreground='#885800')
        world,arena,hardware,selected=self.world,self.arena,self.hardware,self.policy
        self.busy=True;self.sync();self.progress.configure(mode='indeterminate',value=0);self.progress.start();self.status.set('กำลังจำลอง ไม่มีการส่งคำสั่งหุ่น')
        def worker():
            try:
                result=run_episode(world,arena,start,goal,selected if use_knowledge else Policy(),hardware,seed=2026,noise=True)
                self.worker_messages.put(('preview',(result,context,use_knowledge)))
            except Exception as exc:self.worker_messages.put(('error',str(exc)))
        threading.Thread(target=worker,daemon=True).start()

    def clear_trace(self):
        self.trace=[];self.preview_result=None;self.preview_uses_knowledge=None;self.preview_context=None
        self.refresh_results();self.draw()

    def mission_edited(self,*args):
        self.set_mission_feedback('pending','กรอบยังไม่ได้ใช้: กดใช้ค่ากรอบที่แก้ไขก่อน' if self.arena_fields_dirty else 'Start–Goal เปลี่ยนแล้ว กดตรวจและใช้ Start–Goal อีกครั้ง')
        if self.preview_result is not None:
            self.clear_trace()
            self.status.set('Start–Goal เปลี่ยน ล้างผลซิมเดิมแล้ว ตรวจภารกิจและสร้าง knowledge ใหม่')
        else:self.draw()

    def refresh_results(self):
        result=self.preview_result
        if result is None:
            self.simulation_summary.set('ยังไม่มีผลซิม เลือกปุ่มจำลองที่ต้องการ')
            self.simulation_method.set('ยังไม่ได้จำลอง')
            self.simulation_outcome.set('เลือกปุ่มจำลองที่ต้องการ')
            self.outcome_label.configure(foreground='')
            for variable in self.simulation_values.values():variable.set('—')
            return
        label='ใช้ Knowledge' if self.preview_uses_knowledge else 'ค่าเริ่มต้น'
        status='ถึง Goal' if result['status']=='success' else f"ไม่ถึง Goal ({result['status']})"
        self.simulation_method.set(label)
        self.simulation_outcome.set(status)
        self.outcome_label.configure(foreground='#167342' if result['status']=='success' else '#b3261e')
        for key,precision in (('time',2),('distance',3),('goal_error',3),('collisions',0)):
            self.simulation_values[key].set(f'{result[key]:.{precision}f}')
        self.simulation_summary.set(f"{label} · {status}\nเวลา {result['time']:.2f} วินาที · ระยะทาง {result['distance']:.3f} เมตร\nห่าง Goal {result['goal_error']:.3f} เมตร · ชน {result['collisions']} ครั้ง")

    def poll(self):
        try:
            while True:
                kind,data=self.worker_messages.get_nowait()
                if kind=='progress':self.summary.set(data);continue
                self.busy=False;self.progress.stop();self.progress.configure(mode='determinate',value=0 if kind=='error' else 100)
                if kind=='error':
                    self.status.set(data)
                    if self.active_operation=='preview':
                        self.simulation_outcome.set('จำลองไม่สำเร็จ')
                        self.outcome_label.configure(foreground='#b3261e')
                elif kind=='trained':
                    self.policy,self.report,self.trained_context=data
                    self.summary.set(self.report_text(data[1]))
                    self.status.set('สร้าง knowledge แล้ว เลือกจำลองก่อนส่งออกได้')
                elif kind=='preview':
                    result,context,use_knowledge=data
                    try:current_context=None if self.arena_fields_dirty else self.context_fingerprint(*self.mission())
                    except ValueError:current_context=None
                    if context!=current_context:
                        self.clear_trace();self.status.set('ภารกิจเปลี่ยนระหว่างจำลอง ผลเดิมถูกล้าง กรุณาตรวจภารกิจใหม่')
                    else:
                        self.preview_context=context;self.preview_result=result;self.preview_uses_knowledge=use_knowledge
                        self.trace=[(p[0],p[1]) for p in self.preview_result.get('trace',[])]
                        self.refresh_results();self.draw()
                        self.status.set('จำลองเสร็จ: ดูเวลาและระยะทางใต้แผนที่ · ไม่ส่งคำสั่งหุ่น')
                failed=kind=='error' or kind=='preview' and (self.preview_result is None or self.preview_result.get('status')!='success')
                self.action_feedback.set(('ไม่ผ่าน: ' if failed else 'ผ่าน: ')+('สร้าง Knowledge' if self.active_operation=='training' else 'จำลอง'))
                self.active_operation=None
                self.sync()
        except queue.Empty:pass
        if self.monitor:
            if not self.simulation_mode.get():self.report_panel.grid()
            snapshot=self.monitor.snapshot()
            reason=snapshot.get('reason','')
            messages={
                'waiting_global_localization':'รอ Map server / AMCL พร้อม เพื่อค้นหาตำแหน่งทั่วแมพ',
                'initializing_global_localization':'เริ่มกระจายสมมติฐานตำแหน่งทั่วแมพ',
                'waiting_fresh_AMCL_pose':'กำลังเทียบ LiDAR กับแมพ รอตำแหน่งจาก AMCL',
                'waiting_fresh_odometry':'รอ odometry สดจากหุ่น ตรวจการเชื่อมต่อและเวลาสองเครื่อง',
                'waiting_localization_scan':'รอ LiDAR สดที่เห็นรอบหุ่นเพียงพอ ตรวจสิ่งบังเซนเซอร์',
                'localization_map_mismatch':'แมพใน ROS ไม่ตรงกับไฟล์ที่เลือก หยุดชุดเก่าแล้วเปิด Localization จากโปรแกรมนี้',
                'waiting_scan_match':'กำลังเตรียมเทียบสแกนทั่วแมพ',
                'matching_scan_to_map':'กำลังเทียบ LiDAR กับตำแหน่งและทิศต่าง ๆ ทั่วแมพ',
                'waiting_lidar_TF':'รอ TF ตำแหน่งติดตั้ง LiDAR บนตัวหุ่น',
                'waiting_initial_pose_service':'พบตำแหน่งที่ตรงสแกน รอ AMCL รับตำแหน่งตั้งต้น',
                'initializing_scan_match':'ส่งตำแหน่งจากการเทียบสแกนให้ AMCL แล้ว กำลังตรวจซ้ำ',
                'scan_map_mismatch':'สแกน/ตำแหน่ง AMCL ยังไม่ตรงแมพเพียงพอ รอผลตรวจซ้ำ หากค้างให้กดค้นหาใหม่ หรือกำหนด 2D Pose Estimate ใน RViz',
                'robot_moved_during_search':'หุ่นเปลี่ยนตำแหน่งระหว่างค้นหา ให้หุ่นนิ่งแล้วกดค้นหาใหม่',
                'insufficient_localization_scan':'ข้อมูล LiDAR รอบนี้ไม่พอ รอลองรอบใหม่อัตโนมัติ ตรวจสิ่งบังและจุดสแกนที่ขาด',
                'automatic_localization_map_too_large':'แมพใหญ่เกินขอบเขตค้นหาอัตโนมัติ ใช้แมพสนามขนาดเล็กหรือช่วยด้วย 2D Pose Estimate',
                'no_localization_map_obstacles':'แมพไม่มีผนังหรือจุดอ้างอิงเพียงพอสำหรับเทียบตำแหน่ง',
                'no_localization_free_space':'แมพไม่มีพื้นที่ให้วางหุ่นพร้อมระยะเผื่อ ตรวจขนาดหุ่นและแมพ',
                'no_localization_match':'ยังไม่พบตำแหน่งที่เทียบสแกนได้ ลองค้นหาใหม่หรือตรวจแมพ',
                'keep_robot_stationary':'ให้หุ่นอยู่นิ่งระหว่างหาตำแหน่ง',
                'localization_uncertain':'กำลังค้นหา ตำแหน่งยังมีความคลาดเคลื่อนสูง',
                'waiting_localization_particles':'รอกลุ่มสมมติฐานตำแหน่งจาก AMCL',
                'localization_ambiguous':'ยังตรงได้หลายตำแหน่ง กำลังลองสแกนใหม่ ใช้ 2D Pose Estimate ใน RViz เพื่อช่วยระบุตำแหน่งและทิศได้',
                'waiting_localization_stable':'พบตำแหน่งแล้ว กำลังตรวจความนิ่งของผลประมาณ',
                'automatic_localization_timeout':'ยังยืนยันตำแหน่งไม่ได้ ลองค้นหาใหม่ หรือช่วยด้วย 2D Pose Estimate ใน RViz',
                'ready':'พบตำแหน่งและผลประมาณนิ่งแล้ว กดรับตำแหน่ง Start ได้',
            }
            detail=f"ROS: {messages.get(reason,reason)} · พร้อม={snapshot.get('ready',False)}"
            support=snapshot.get('particle_support')
            if support is not None:detail+=f'\nน้ำหนักสมมติฐานใกล้ตำแหน่งนี้ {support:.0%}'
            if snapshot.get('scan_agreement') is not None:detail+=f"\nลำแสงที่ตรงกับแมพ {snapshot['scan_agreement']:.0%}"
            if snapshot.get('match_attempts',0)>0:detail+=f"\nเทียบสแกนอัตโนมัติรอบที่ {snapshot['match_attempts']}"
            if reason=='waiting_localization_stable':
                detail+=f"\nผลประมาณ {snapshot.get('stability_samples',0)}/4 ค่า · ช่วงเวลา {snapshot.get('stability_span',0.):.1f}/1.5 s"
                detail+=f"\nตำแหน่งแกว่ง {100*snapshot.get('stability_position_drift',0.):.1f}/2.5 cm · ทิศ {math.degrees(snapshot.get('stability_yaw_drift',0.)):.1f}/4.0°"
            self.localization_status.set(detail)
            from core.localization import localization_diagnostics
            blocker=snapshot.get('blocking_reason',reason)
            self.localization_report.set('สถานะ Localize: '+messages.get(reason,reason)+'\nติด condition: '+blocker+'\n'+localization_diagnostics(snapshot))
            if snapshot.get('ready') and not self._rviz_auto_closed:
                try:
                    if self.localization:self.localization.close_rviz()
                    self._rviz_auto_closed=True
                    self.action_feedback.set('ผ่าน: Localize สำเร็จ · '+('ปิด RViz แล้ว · ' if self.localization else '')+'กดรับตำแหน่ง Start ได้')
                except Exception as exc:self.error(exc)

        self.root.after(100,self.poll)

    def report_text(self,report):
        return (f"เลือกชุด #{report.get('selected_index',0)+1} · ฝึกสำเร็จ {report.get('successes',0)}/{report.get('cases',0)} · ชน {report.get('collisions',0)}\n"
                f"ประเมินแยก: {report.get('evaluation_successes',0)}/{report.get('evaluation_cases',0)}\n"
                f"ค่าเริ่มต้น: {report.get('baseline_successes',0)}/{report.get('baseline_cases',0)}\n"
                f"ทดลองครบ {report.get('evaluated_candidates',0)} ชุด")

    def start_localization(self):
        if self.busy or self.slam is None:self.status.set('ต้องโหลดแมพและรอการคำนวณจบก่อน');return False
        if self.simulation_mode.get():self.status.set('โหมดจำลอง ไม่ต้องเปิด AMCL');return False
        try:
            from core.localization import start_localization
            if self.localization is None or self.localization.poll() is not None:
                if self.monitor:self.monitor.stop();self.monitor=None
                self.localization=start_localization(self.slam.yaml_path,self.slam.image_path)
                self.invalidate()
            else:self.localization.ensure_rviz()
            self._rviz_auto_closed=False
            self.ensure_setup_monitor()
            self.status.set('กำลังหาตำแหน่งจาก LiDAR เทียบแมพ ให้หุ่นนิ่ง รอพร้อม=True แล้วกดรับ Start')
        except Exception as exc:self.error(exc)

    def ensure_setup_monitor(self):
        if self.monitor is None:
            from core.ros import SetupMonitor
            self.monitor=SetupMonitor(auto_localize=True,expected_map=self.slam,hardware=self.hardware)
            self.monitor.start()

    def retry_localization(self):
        if self.busy or self.slam is None:self.status.set('ต้องโหลดแมพและรอการคำนวณจบก่อน');return False
        if self.simulation_mode.get():self.status.set('โหมดจำลอง: ไม่ต้องค้นหาตำแหน่งจาก ROS');return False
        if self.localization is None or self.localization.poll() is not None:
            self.start_localization();return
        try:
            self.ensure_setup_monitor()
            self.monitor.request_global_localization()
            self._rviz_auto_closed=False
            self.invalidate()
            self.status.set('เริ่มค้นหาทั่วแมพ ให้หุ่นอยู่นิ่ง และรับ Start ใหม่เมื่อพร้อม')
        except Exception as exc:self.error(exc)

    def capture_start(self):
        if self.busy or self.arena is None:self.status.set('ต้องโหลดสนามก่อนรับ Start');return False
        if self.simulation_mode.get():self.status.set('โหมดจำลอง: คลิก Start และตั้งมุมหันได้เลย');return False
        try:
            if self.monitor is None:
                self.ensure_setup_monitor()
                self.status.set('กำลังหาตำแหน่งอัตโนมัติ ให้หุ่นนิ่ง รอพร้อม=True แล้วกดรับ Start อีกครั้ง');return False
            self.ensure_setup_monitor()
            pose,reference=self.monitor.capture(self.arena,self.robot_id.get().strip())
            self.invalidate();self.reference=reference;self.captured_start=pose;self.set_pose(pose);self.validate_mission();self.draw()
            self.set_mission_feedback('valid','ผ่าน: รับ Start จากหุ่นแล้ว จุด Start–Goal วางหุ่นได้และมีทางเชื่อม')
            self.status.set('รับ Start และ odometry แล้ว ไปสร้าง knowledge; จะ capture ซ้ำก่อนส่งออก')
        except Exception as exc:self.error(exc)

    def export_to(self,path):
        if self.policy is None or self.report is None:raise ValueError('สร้าง knowledge ให้เสร็จก่อน')
        start,goal=self.validate_mission()
        if self.context_fingerprint(start,goal)!=self.trained_context:
            raise ValueError('สนาม Start–Goal หรือ hardware เปลี่ยนหลังฝึก ต้องสร้าง knowledge ใหม่')
        if self.simulation_mode.get():reference=InitialReference(robot_id=self.robot_id.get().strip())
        else:
            if self.monitor is None:raise ValueError('ต้องเชื่อม localization และ capture odometry')
            pose,reference=self.monitor.capture(self.arena,self.robot_id.get().strip())
            if math.dist((pose.x,pose.y),(start.x,start.y))>.04 or abs((pose.yaw-start.yaw+math.pi)%(2*math.pi)-math.pi)>.10:
                raise ValueError('หุ่นเปลี่ยนตำแหน่งจากภารกิจที่ฝึก รับ Start และฝึกใหม่ก่อนส่งออก')
            # Use the fresh synchronized localization estimate as the anchor.
            # Small estimator updates are allowed; physical movements above
            # tolerance require retraining with the new Start.
            start=pose
            if not self.world.free(start.x,start.y,self.hardware.safe_radius):
                raise ValueError('Start จาก localization ล่าสุดชิดสิ่งกีดขวางเกินไป ตรวจและรับตำแหน่งใหม่')
        output=Path(path).expanduser().resolve()
        if output==(OUTPUT/'training_report.json').resolve():
            raise ValueError('ชื่อนี้สงวนไว้สำหรับรายงาน เลือกชื่อ knowledge.json หรือชื่ออื่น')
        sources={Path(self.slam.yaml_path).resolve(),Path(self.slam.image_path).resolve()}
        if output in sources:raise ValueError('ห้ามเขียนทับแผนที่ต้นฉบับ')
        summary={k:v for k,v in self.report.items() if k in SUMMARY_KEYS}
        knowledge=Knowledge(self.arena,start,goal,self.policy,self.hardware,reference,summary)
        knowledge.save(output)
        OUTPUT.mkdir(parents=True,exist_ok=True)
        (OUTPUT/'training_report.json').write_text(json.dumps(self.report,indent=2,allow_nan=False),encoding='utf-8')
        self.reference=reference;self.frozen=True;self.sync()
        return knowledge

    def context_fingerprint(self,start,goal):
        return digest({'arena':asdict(self.arena),'start':asdict(start),'goal':goal,'hardware':asdict(self.hardware)})

    def export(self):
        if self.busy:return
        OUTPUT.mkdir(parents=True,exist_ok=True)
        path=filedialog.asksaveasfilename(parent=self.root,initialdir=str(OUTPUT),initialfile='knowledge.json',defaultextension='.json',filetypes=[('Knowledge','*.json')])
        if not path:return 'cancelled'
        try:
            self.export_to(path)
            self.status.set(f'บันทึกแล้ว: {path} · '+('simulation only ใช้สั่งหุ่นจริงไม่ได้' if self.simulation_mode.get() else 'ให้หุ่นอยู่นิ่ง ปิด AMCL ก่อนเปิด Program 2'))
        except Exception as exc:self.error(exc)

    def error(self,exc):
        self.action_failed=True
        self.action_feedback.set(f'ไม่ผ่าน: {exc}')
        self.status.set(str(exc));messagebox.showerror('ตรวจข้อมูล',str(exc),parent=self.root)

    def stop_localization(self):
        if self.monitor:self.monitor.stop();self.monitor=None
        if self.localization:self.localization.terminate()
        self.localization=None
        self._rviz_auto_closed=False
        self.localization_report.set('Localization หยุดแล้ว · ต้องเปิดใหม่ก่อนรับ Start')
        self.localization_status.set('หยุด localization ที่โปรแกรมนี้เปิดแล้ว')
        self.status.set('หุ่นต้องอยู่นิ่งและ odometry ไม่ reset ระหว่างส่งต่อ; หากเปิด AMCL เองให้ปิดด้วย')

    def close(self):
        if self.busy and not messagebox.askyesno('กำลังคำนวณ','ออกและยกเลิกผลที่ยังไม่เสร็จ?',parent=self.root):return
        self.stop_localization()
        self.root.destroy()


def main():
    root=tk.Tk();app=App(root)
    previous={}
    def request_shutdown(signum,frame):
        root.quit()
    for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):
        previous[sig]=signal.signal(sig,request_shutdown)
    try:
        root.mainloop()
    finally:
        try:app.stop_localization()
        finally:
            try:root.destroy()
            except tk.TclError:pass
            for sig,handler in previous.items():signal.signal(sig,handler)


if __name__=='__main__':main()

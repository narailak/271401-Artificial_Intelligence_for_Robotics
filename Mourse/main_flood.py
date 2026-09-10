"""
main_flood.py - Simulator ใช้ RatFlood (Flood Fill) + Map จาก man_version
รันด้วย: python main_flood.py
"""
import time
import tkinter as tk
from tkinter import messagebox, filedialog
import numpy as np
import sys
import pathlib

from rat_flood import RatFlood, NORTH, SOUTH, WEST, EAST

# Add man_version to path
sys.path.append(str(pathlib.Path(__file__).parent / "man_version"))

from stage.stage import Stage
from config import StageConfig


class FloodSimulator:
    def __init__(self, root):
        self.root = root
        self.root.title("Micromouse - Flood Fill Rat + man_version Map")
        self.root.configure(bg="#0f172a")

        self.cell_size = 20
        self.offset_x = 0
        self.offset_y = 0

        self.seed_val = 80
        self.running = False
        self.delay_ms = 80      # Slightly faster animation
        self.step_count = 0
        self.walk_count = 0
        self.turn_count = 0
        self.total_compute_time_ms = 0.0

        self.use_saved_map = False
        self.map_file_path = "stage/maps/map_example.json"

        self.generate_new_maze(init=True)
        self.setup_layout()

    # ---------------------------------------------------------------- Maze
    def generate_new_maze(self, init=False):
        if not init:
            try:
                val = self.seed_entry.get().strip()
                self.seed_val = int(val) if val else None
            except ValueError:
                messagebox.showerror("Error", "กรุณากรอก Seed เป็นตัวเลขจำนวนเต็ม")
                return

        self.running = False
        self.step_count = 0
        self.walk_count = 0
        self.turn_count = 0
        self.total_compute_time_ms = 0.0

        cfg = StageConfig(
            seed=self.seed_val,
            use_saved_map=self.use_saved_map,
            map_file=self.map_file_path
        )
        self.stage = Stage(cfg)
        self.grid_size = self.stage.width

        # stage uses (x=col, y=row); rat uses (row, col)
        start_pos = (self.stage.start[1], self.stage.start[0])
        goal_pos  = (self.stage.finish[1], self.stage.finish[0])

        self.rat = RatFlood(start_pos=start_pos, goal_pos=goal_pos,
                            grid_size=self.grid_size)
        self.visited_cells = set()

        # Sense initial front
        self.sense_current_front()

        if not init:
            self.draw_maze()

        if hasattr(self, 'action_var'):
            self._update_hud("-")

    # ---------------------------------------------------------------- Sensing
    def sense_current_front(self):
        r, c = self.rat.position
        dr, dc = self.rat.facing
        curr_cell = (c, r)
        next_cell = (c + dc, r + dr)

        if not self.stage.in_bounds(next_cell):
            wall = True
        else:
            wall = not self.stage.is_open(curr_cell, next_cell)

        self.rat.sense_and_update(wall)

    # ---------------------------------------------------------------- Simulation
    def step_simulation(self):
        goal_r, goal_c = self.stage.finish[1], self.stage.finish[0]
        if self.rat.position == (goal_r, goal_c):
            self.running = False
            dist = self.rat.get_distance_to_cheese()
            self._update_hud("WIN 🎉")
            messagebox.showinfo(
                "สำเร็จ! (Flood Fill)",
                f"หนูถึงเส้นชัยแล้ว!\n"
                f"เดินจริง (Walks)  : {self.walk_count} ช่อง\n"
                f"หมุนตัว (Turns)   : {self.turn_count} รอบ\n"
                f"รอบตัดสินใจรวม   : {self.step_count} รอบ\n"
                f"เวลาคำนวณสะสม  : {self.total_compute_time_ms:.4f} ms\n"
                f"ระยะตรงจากเป้า   : {dist:.1f} cm"
            )
            return False

        t0 = time.perf_counter()

        # Sense → decide → apply
        self.sense_current_front()
        action = self.rat.decide_next_action()

        elapsed = (time.perf_counter() - t0) * 1000.0
        self.total_compute_time_ms += elapsed

        if action is None:
            self.running = False
            print("[Error] อัลกอริทึมไม่สามารถหาเส้นทางได้")
            return False

        self.rat.apply_action(action)
        self.step_count += 1
        if action == "FORWARD":
            self.walk_count += 1
        else:
            self.turn_count += 1

        self.visited_cells.add(self.rat.position)

        if hasattr(self, 'action_var'):
            self._update_hud(action)

        self.draw_maze()
        return True

    def run_loop(self):
        if self.running:
            ok = self.step_simulation()
            if ok:
                self.root.after(self.delay_ms, self.run_loop)
            else:
                self.running = False
                self.play_btn.config(text="เล่นอัตโนมัติ")

    def toggle_run(self):
        if self.running:
            self.running = False
            self.play_btn.config(text="เล่นอัตโนมัติ")
        else:
            goal_r, goal_c = self.stage.finish[1], self.stage.finish[0]
            if self.rat.position == (goal_r, goal_c):
                self.generate_new_maze()
            self.running = True
            self.play_btn.config(text="หยุดชั่วคราว")
            self.run_loop()

    def reset_sim(self):
        self.generate_new_maze(init=True)
        self.draw_maze()

    def browse_map_file(self):
        initial_dir = pathlib.Path(__file__).parent / "man_version" / "stage" / "maps"
        fp = filedialog.askopenfilename(
            initialdir=str(initial_dir),
            title="เลือกไฟล์แผนที่ JSON",
            filetypes=(("JSON files", "*.json"), ("All files", "*.*"))
        )
        if fp:
            p = pathlib.Path(fp)
            try:
                rel = p.relative_to(pathlib.Path(__file__).parent / "man_version" / "stage")
                self.map_file_path = str(rel).replace("\\", "/")
            except ValueError:
                self.map_file_path = str(p).replace("\\", "/")
            self.use_saved_map = True
            self.map_label.config(text=pathlib.Path(self.map_file_path).name)
            self.generate_new_maze(init=True)

    def use_random_maze(self):
        self.use_saved_map = False
        self.map_label.config(text="สร้างแบบสุ่ม (Perfect Maze)")
        self.generate_new_maze(init=False)

    # ---------------------------------------------------------------- HUD
    def _update_hud(self, action):
        dist = self.rat.get_distance_to_cheese()
        self.action_var.set(action)
        self.position_var.set(str(self.rat.position))
        self.walk_var.set(str(self.walk_count))
        self.turn_var.set(str(self.turn_count))
        self.total_var.set(str(self.step_count))
        self.smell_var.set(f"{dist:.1f} cm")
        self.time_var.set(f"{self.total_compute_time_ms:.2f} ms")

    # ---------------------------------------------------------------- Layout
    def setup_layout(self):
        main = tk.Frame(self.root, bg="#0f172a", padx=15, pady=15)
        main.pack(fill="both", expand=True)

        # ---- Canvas area ----
        canvas_frame = tk.LabelFrame(
            main,
            text=" Flood Fill Rat - (เส้นเทา = กำแพงจริง | เส้นแดง = กำแพงที่รู้แล้ว | สีฟ้า = เส้นทางที่เดิน) ",
            bg="#0f172a", fg="#94a3b8", font=("Helvetica", 10, "bold"),
            bd=2, relief="groove", labelanchor="nw"
        )
        canvas_frame.pack(side="left", padx=(0, 15), fill="both", expand=True)

        # Legend
        legend = tk.Frame(canvas_frame, bg="#0f172a")
        legend.pack(side="bottom", fill="x", padx=10, pady=(5, 10))

        def leg(parent, color, text, label, circle=False, border=None):
            f = tk.Frame(parent, bg="#0f172a")
            f.pack(side="left", expand=True)
            sc = tk.Canvas(f, width=20, height=20, bg="#0f172a", bd=0, highlightthickness=0)
            sc.pack(side="left", padx=5)
            bc = border or color
            if circle:
                sc.create_oval(2, 2, 18, 18, fill=color, outline=bc, width=1.5)
                sc.create_text(10, 10, text=text, fill="#fff", font=("Helvetica", 8, "bold"))
            else:
                sc.create_rectangle(2, 2, 18, 18, fill=color, outline=bc)
                sc.create_text(10, 10, text=text, fill="#fff", font=("Helvetica", 10, "bold"))
            tk.Label(f, text=label, bg="#0f172a", fg="#cbd5e1",
                     font=("Helvetica", 9, "bold")).pack(side="left")

        leg(legend, "#2ecc71", "S", "Start")
        leg(legend, "#e74c3c", "F", "Finish")
        leg(legend, "#f59e0b", "R", "Rat", circle=True, border="#d97706")
        leg(legend, "#dbeafe", "", "เส้นทางที่เดิน", border="#93c5fd")
        leg(legend, "#ef4444", "", "กำแพงที่รู้แล้ว")
        leg(legend, "#94a3b8", "", "กำแพงจริงในสนาม")

        self.canvas = tk.Canvas(canvas_frame, bg="#ffffff", bd=0, highlightthickness=0)
        self.canvas.pack(padx=10, pady=10, fill="both", expand=True)
        self.canvas.bind("<Configure>", self.on_resize)

        # ---- Sidebar ----
        self.sidebar = tk.Frame(main, bg="#1e293b", width=320, padx=15, pady=15)
        self.sidebar.pack(side="right", fill="both", expand=False)
        self.sidebar.pack_propagate(False)

        tk.Label(self.sidebar, text="Flood Fill Rat",
                 bg="#1e293b", fg="#f8fafc",
                 font=("Helvetica", 15, "bold")).pack(anchor="w")
        tk.Label(self.sidebar, text="Online Flood Fill + man_version maps",
                 bg="#1e293b", fg="#64748b",
                 font=("Helvetica", 9, "italic")).pack(anchor="w", pady=(0, 12))

        # Map selection
        mf = tk.LabelFrame(self.sidebar, text=" แหล่งแผนที่ ",
                            bg="#1e293b", fg="#94a3b8",
                            font=("Helvetica", 9, "bold"), padx=10, pady=8)
        mf.pack(fill="x", pady=(0, 10))

        self.map_label = tk.Label(mf, text="สร้างแบบสุ่ม (Perfect Maze)",
                                  bg="#0f172a", fg="#f8fafc",
                                  font=("Helvetica", 9), anchor="w", padx=5)
        self.map_label.pack(fill="x", ipady=4, pady=(0, 5))

        mbf = tk.Frame(mf, bg="#1e293b")
        mbf.pack(fill="x")

        def mab(txt, cmd):
            b = tk.Button(mbf, text=txt, bg="#475569", fg="white",
                          relief="flat", bd=0, font=("Helvetica", 9, "bold"),
                          cursor="hand2", command=cmd)
            b.pack(side="left", expand=True, fill="x", padx=1)
            return b

        mab("เลือกไฟล์แผนที่", self.browse_map_file)
        mab("สุ่มใหม่", self.use_random_maze)

        # Seed
        sf = tk.Frame(self.sidebar, bg="#1e293b")
        sf.pack(fill="x", pady=(0, 10))
        tk.Label(sf, text="Seed:", bg="#1e293b", fg="#cbd5e1",
                 font=("Helvetica", 10, "bold")).pack(side="left", padx=(0, 5))
        self.seed_entry = tk.Entry(sf, bg="#0f172a", fg="#f8fafc",
                                   insertbackground="white",
                                   bd=1, relief="flat",
                                   font=("Helvetica", 10), width=10)
        self.seed_entry.insert(0, str(self.seed_val))
        self.seed_entry.pack(side="left", padx=5, ipady=3)

        def btn(txt, bg, abg, cmd):
            b = tk.Button(self.sidebar, text=txt, bg=bg, fg="white",
                          activebackground=abg, activeforeground="white",
                          relief="flat", bd=0,
                          font=("Helvetica", 10, "bold"),
                          cursor="hand2", command=cmd, height=2)
            b.bind("<Enter>", lambda e: b.config(bg=abg))
            b.bind("<Leave>", lambda e: b.config(bg=bg))
            b.pack(fill="x", pady=4)
            return b

        btn("สร้างเขาวงกตใหม่", "#2563eb", "#1d4ed8", self.generate_new_maze)
        self.play_btn = btn("เล่นอัตโนมัติ", "#059669", "#047857", self.toggle_run)
        btn("ขยับทีละก้าว (Step)", "#d97706", "#b45309", self.step_simulation)
        btn("รีเซ็ตตัวหนู (Reset)", "#dc2626", "#b91c1c", self.reset_sim)

        # HUD Dashboard
        hud = tk.LabelFrame(self.sidebar, text=" รายงานสถานะเรียลไทม์ ",
                            bg="#1e293b", fg="#94a3b8",
                            font=("Helvetica", 10, "bold"),
                            bd=2, relief="groove", padx=10, pady=10)
        hud.pack(fill="both", expand=True, pady=(12, 0))
        hud.columnconfigure(0, weight=1)
        hud.columnconfigure(1, weight=1)

        self.action_var   = tk.StringVar(value="-")
        self.position_var = tk.StringVar(value=str(self.rat.position))
        self.walk_var     = tk.StringVar(value="0")
        self.turn_var     = tk.StringVar(value="0")
        self.total_var    = tk.StringVar(value="0")
        self.smell_var    = tk.StringVar(value=f"{self.rat.get_distance_to_cheese():.1f} cm")
        self.time_var     = tk.StringVar(value="0.00 ms")

        def card(row, col, title, var, color, span=1):
            bg = "#0f172a"
            f = tk.Frame(hud, bg=bg, padx=8, pady=8, bd=1, relief="ridge")
            f.grid(row=row, column=col, columnspan=span,
                   sticky="nsew", padx=3, pady=3)
            tk.Label(f, text=title, bg=bg, fg="#64748b",
                     font=("Helvetica", 8, "bold")).pack(anchor="w")
            tk.Label(f, textvariable=var, bg=bg, fg=color,
                     font=("Consolas", 12, "bold")).pack(anchor="w", pady=(2, 0))

        card(0, 0, "ตำแหน่ง (Row, Col)", self.position_var, "#38bdf8")
        card(0, 1, "การตัดสินใจล่าสุด",  self.action_var,   "#c084fc")
        card(1, 0, "เดินจริง (Walks)",   self.walk_var,     "#60a5fa")
        card(1, 1, "หมุนตัว (Turns)",    self.turn_var,     "#fbbf24")
        card(2, 0, "สเต็ปรวม",           self.total_var,    "#cbd5e1")
        card(2, 1, "ระยะชีส (ตรง)",      self.smell_var,    "#f43f5e")

        # Time card full width
        bg = "#0f172a"
        tf = tk.Frame(hud, bg=bg, padx=8, pady=8, bd=1, relief="ridge")
        tf.grid(row=3, column=0, columnspan=2, sticky="ew", padx=3, pady=3)
        tk.Label(tf, text="เวลาคำนวณสะสม",
                 bg=bg, fg="#64748b", font=("Helvetica", 8, "bold")).pack(side="left")
        tk.Label(tf, textvariable=self.time_var,
                 bg=bg, fg="#34d399", font=("Consolas", 11, "bold")).pack(side="right")

    # ---------------------------------------------------------------- Resize / Draw
    def on_resize(self, event):
        margin = 10
        w = max(50, event.width  - margin * 2)
        h = max(50, event.height - margin * 2)
        sz = min(w, h)
        cs = max(5, sz // self.grid_size)
        self.cell_size = cs
        self.offset_x = (event.width  - self.grid_size * cs) // 2
        self.offset_y = (event.height - self.grid_size * cs) // 2
        self.draw_maze()

    def draw_maze(self):
        self.canvas.delete("all")
        cs = self.cell_size
        ox = self.offset_x
        oy = self.offset_y

        self.visited_cells.add(self.rat.position)

        sx, sy = self.stage.start
        fx, fy = self.stage.finish

        # Cells
        for r in range(self.grid_size):
            for c in range(self.grid_size):
                x1, y1 = c * cs + ox, r * cs + oy
                x2, y2 = x1 + cs, y1 + cs

                color, outline = "#ffffff", "#e2e8f0"
                if (r, c) in self.visited_cells:
                    color, outline = "#dbeafe", "#93c5fd"
                if (c, r) == (sx, sy):
                    color, outline = "#2ecc71", "#27ae60"
                elif (c, r) == (fx, fy):
                    color, outline = "#e74c3c", "#c0392b"

                self.canvas.create_rectangle(x1, y1, x2, y2,
                                             fill=color, outline=outline, width=1)
                if (c, r) == (sx, sy):
                    self.canvas.create_text(x1 + cs/2, y1 + cs/2, text="S",
                                            fill="#fff", font=("Helvetica", 10, "bold"))
                elif (c, r) == (fx, fy):
                    self.canvas.create_text(x1 + cs/2, y1 + cs/2, text="F",
                                            fill="#fff", font=("Helvetica", 10, "bold"))

        # Actual walls (thin gray)
        for r in range(self.grid_size + 1):
            for c in range(self.grid_size):
                # Horizontal
                has_wall = (r == 0 or r == self.grid_size or
                            not self.stage.is_open((c, r-1), (c, r)))
                if has_wall:
                    x1 = c * cs + ox;  y1 = r * cs + oy
                    x2 = (c+1) * cs + ox
                    self.canvas.create_line(x1, y1, x2, y1, fill="#94a3b8", width=1.5)

        for r in range(self.grid_size):
            for c in range(self.grid_size + 1):
                # Vertical
                has_wall = (c == 0 or c == self.grid_size or
                            not self.stage.is_open((c-1, r), (c, r)))
                if has_wall:
                    x1 = c * cs + ox;  y1 = r * cs + oy
                    y2 = (r+1) * cs + oy
                    self.canvas.create_line(x1, y1, x1, y2, fill="#94a3b8", width=1.5)

        # Known walls (thick red) — draw on top
        for r in range(self.grid_size + 1):
            for c in range(self.grid_size):
                v = self.rat.known_h_walls[r, c]
                if v is True:
                    x1 = c * cs + ox;  y1 = r * cs + oy
                    x2 = (c+1) * cs + ox
                    self.canvas.create_line(x1, y1, x2, y1,
                                            fill="#ef4444", width=3.5, capstyle="projecting")

        for r in range(self.grid_size):
            for c in range(self.grid_size + 1):
                v = self.rat.known_v_walls[r, c]
                if v is True:
                    x1 = c * cs + ox;  y1 = r * cs + oy
                    y2 = (r+1) * cs + oy
                    self.canvas.create_line(x1, y1, x1, y2,
                                            fill="#ef4444", width=3.5, capstyle="projecting")

        # Rat
        rr, rc = self.rat.position
        rx = rc * cs + cs/2 + ox
        ry = rr * cs + cs/2 + oy
        rad = cs * 0.38
        self.canvas.create_oval(rx - rad, ry - rad, rx + rad, ry + rad,
                                fill="#f59e0b", outline="#d97706", width=2)
        dr, dc = self.rat.facing
        self.canvas.create_line(rx, ry,
                                rx + dc * rad * 1.3,
                                ry + dr * rad * 1.3,
                                fill="#ffffff", width=2.5)


if __name__ == "__main__":
    root = tk.Tk()
    root.resizable(True, True)
    root.geometry("1100x680")
    FloodSimulator(root)
    root.mainloop()

import time
import tkinter as tk
from tkinter import messagebox
from tkinter import filedialog
import numpy as np
import sys
import pathlib

from rat import Rat, NORTH, SOUTH, WEST, EAST

# Add man_version to path so we can import from its modules
sys.path.append(str(pathlib.Path(__file__).parent / "man_version"))

from stage.stage import Stage
from config import StageConfig

class ManMapSimulator:
    def __init__(self, root):
        self.root = root
        self.root.title("Micromouse Simulator - Map Version & Rat.py integration")
        self.root.configure(bg="#0f172a") # Slate 900
        
        # Grid settings
        self.cell_size = 20
        self.offset_x = 0
        self.offset_y = 0
        
        # Simulation control
        self.seed_val = 70
        self.running = False
        self.delay_ms = 100
        self.step_count = 0
        self.total_compute_time_ms = 0.0
        
        # Default map settings
        self.use_saved_map = False
        self.map_file_path = "stage/maps/map_example.json"
        
        # Initialize stage and rat
        self.generate_new_maze(init=True)
        
        # Build layout
        self.setup_layout()

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

        # Load stage from man_version using configuration
        cfg = StageConfig(
            seed=self.seed_val,
            use_saved_map=self.use_saved_map,
            map_file=self.map_file_path
        )
        self.stage = Stage(cfg)
        
        self.grid_size = self.stage.width
        
        # Convert start/finish coordinate (x, y) in stage -> (row, col) in Rat
        # stage.start is (x, y) = (col, row)
        start_pos = (self.stage.start[1], self.stage.start[0])
        goal_pos = (self.stage.finish[1], self.stage.finish[0])
        
        # Initialize Rat from rat.py
        self.rat = Rat(start_pos=start_pos, goal_pos=goal_pos, grid_size=self.grid_size)
        self.visited_cells = set()
        
        # Sense the initial cell front wall
        self.sense_current_front()

        if not init:
            self.draw_maze()
            self.log_output(f"\n[System] โหลดด่าน/แผนที่สำเร็จ แหล่งที่มา: {self.stage.source}")
            self.log_output(f" - ขนาดสนาม: {self.grid_size}x{self.grid_size} ช่อง")
            self.log_output(f" - พิกัดเริ่มต้นของหนู: {self.rat.position} (หันหน้าไปทางขวา)")

        # Update HUD/Dashboard values if initialized
        if hasattr(self, 'action_var'):
            self.action_var.set("-")
            self.position_var.set(str(self.rat.position))
            self.walk_steps_var.set("0")
            self.turn_steps_var.set("0")
            self.total_steps_var.set("0")
            self.cheese_smell_var.set(f"{self.rat.get_distance_to_cheese():.1f} cm")
            self.compute_time_var.set("0.00 ms")

    def sense_current_front(self):
        """Senses the wall in front of the rat and updates its memory"""
        r, c = self.rat.position
        dr, dc = self.rat.facing
        
        # In man_version, cell is (x, y) = (col, row)
        curr_cell = (c, r)
        next_cell = (c + dc, r + dr)
        
        wall_in_front = False
        if not self.stage.in_bounds(next_cell):
            wall_in_front = True
        else:
            # If not open, there is a wall between them
            wall_in_front = not self.stage.is_open(curr_cell, next_cell)
            
        self.rat.sense_and_update(wall_in_front)

    def step_simulation(self):
        """Executes one step of the simulation"""
        # Convert finish point (x, y) -> (row, col)
        goal_r, goal_c = self.stage.finish[1], self.stage.finish[0]
        if self.rat.position == (goal_r, goal_c):
            self.running = False
            dist_cheese = self.rat.get_distance_to_cheese()
            self.log_output(f"\n[Victory!] หนูเดินถึงเป้าหมาย (ชีส) สำเร็จ!")
            self.log_output(f" - ระยะทางตรงถึงชีสที่ได้กลิ่น (Euclidean): {dist_cheese:.1f} cm")
            self.log_output(f" - จำนวนการเดินเปลี่ยนที่จริง (Walk Steps): {self.walk_count} ช่อง")
            self.log_output(f" - จำนวนการหมุนตัวตรวจจับกำแพง (Turn Steps): {self.turn_count} รอบ")
            self.log_output(f" - รอบการตัดสินใจทั้งหมด (Total Steps): {self.step_count} รอบ")
            self.log_output(f" - เวลาประมวลผลคำนวณทั้งหมด: {self.total_compute_time_ms:.4f} ms")
            messagebox.showinfo("สำเร็จ", 
                                f"หนูถึงเส้นชัยแล้ว!\n"
                                f"เดินจริง (Walks): {self.walk_count} ช่อง\n"
                                f"หมุนตัว (Turns): {self.turn_count} รอบ\n"
                                f"รอบตัดสินใจรวม: {self.step_count} รอบ\n"
                                f"เวลาคำนวณสะสม: {self.total_compute_time_ms:.4f} ms\n"
                                f"ระยะห่างตรงจากเป้าหมาย: {dist_cheese:.1f} cm")
            return False

        # Measure computation time
        start_time = time.perf_counter()
        
        # Step 1: Sense
        self.sense_current_front()
        
        # Step 2: Decide
        action = self.rat.decide_next_action()
        
        elapsed_ms = (time.perf_counter() - start_time) * 1000.0
        self.total_compute_time_ms += elapsed_ms
        
        if action is None:
            self.running = False
            self.log_output("[Error] อัลกอริทึมไม่สามารถหาเส้นทางได้")
            return False
            
        # Apply the action
        self.rat.apply_action(action)
        self.step_count += 1
        if action == "FORWARD":
            self.walk_count += 1
        else:
            self.turn_count += 1
            
        dist_cheese = self.rat.get_distance_to_cheese()
        
        # Update HUD/Dashboard values
        if hasattr(self, 'action_var'):
            self.action_var.set(action)
            self.position_var.set(str(self.rat.position))
            self.walk_steps_var.set(str(self.walk_count))
            self.turn_steps_var.set(str(self.turn_count))
            self.total_steps_var.set(str(self.step_count))
            self.cheese_smell_var.set(f"{dist_cheese:.1f} cm")
            self.compute_time_var.set(f"{self.total_compute_time_ms:.2f} ms")
            
        self.draw_maze()
        return True

    def run_loop(self):
        if self.running:
            success = self.step_simulation()
            if success:
                self.root.after(self.delay_ms, self.run_loop)
            else:
                self.running = False
                self.play_btn.config(text="เล่นอัตโนมัติ")

    def toggle_run(self):
        if self.running:
            self.running = False
            self.play_btn.config(text="เล่นอัตโนมัติ")
            self.log_output("[System] หยุดการจำลองชั่วคราว")
        else:
            goal_r, goal_c = self.stage.finish[1], self.stage.finish[0]
            if self.rat.position == (goal_r, goal_c):
                self.generate_new_maze()
            self.running = True
            self.play_btn.config(text="หยุดชั่วคราว")
            self.log_output("[System] เริ่มการจำลองการเดิน...")
            self.run_loop()

    def reset_sim(self):
        self.generate_new_maze(init=True)
        self.draw_maze()
        self.log_output("\n[System] รีเซ็ตตำแหน่งหนูและล้างความจำแผนที่เรียบร้อย")

    def browse_map_file(self):
        initial_dir = pathlib.Path(__file__).parent / "man_version" / "stage" / "maps"
        file_path = filedialog.askopenfilename(
            initialdir=str(initial_dir),
            title="เลือกไฟล์แผนที่ JSON",
            filetypes=(("JSON files", "*.json"), ("All files", "*.*"))
        )
        if file_path:
            # Keep path relative to man_version/stage/ if possible, otherwise absolute
            p = pathlib.Path(file_path)
            try:
                rel = p.relative_to(pathlib.Path(__file__).parent / "man_version" / "stage")
                self.map_file_path = str(rel).replace("\\", "/")
            except ValueError:
                self.map_file_path = str(p).replace("\\", "/")
            
            self.use_saved_map = True
            self.map_file_label.config(text=pathlib.Path(self.map_file_path).name)
            self.generate_new_maze(init=True)

    def use_random_maze(self):
        self.use_saved_map = False
        self.map_file_label.config(text="สร้างแบบสุ่ม (Perfect Maze)")
        self.generate_new_maze(init=False)

    def setup_layout(self):
        self.main_frame = tk.Frame(self.root, bg="#0f172a", padx=15, pady=15)
        self.main_frame.pack(fill="both", expand=True)

        canvas_frame = tk.LabelFrame(
            self.main_frame, text=" แผนที่จาก Map Version (เส้นดำ = กำแพงจริง | เส้นแดงหนา = กำแพงที่หนูตรวจเจอแล้ว) ",
            bg="#0f172a", fg="#94a3b8", font=("Helvetica", 10, "bold"),
            bd=2, relief="groove", labelanchor="nw"
        )
        canvas_frame.pack(side="left", padx=(0, 15), fill="both", expand=True)

        # Legend Frame
        legend_frame = tk.Frame(canvas_frame, bg="#0f172a")
        legend_frame.pack(side="bottom", fill="x", padx=10, pady=(5, 10))

        def create_legend_item(parent, color, text, label_text, is_circle=False, border_color=None):
            item_frame = tk.Frame(parent, bg="#0f172a")
            item_frame.pack(side="left", expand=True)
            
            symbol_canvas = tk.Canvas(item_frame, width=20, height=20, bg="#0f172a", bd=0, highlightthickness=0)
            symbol_canvas.pack(side="left", padx=5)
            
            b_color = border_color if border_color else color
            if is_circle:
                symbol_canvas.create_oval(2, 2, 18, 18, fill=color, outline=b_color, width=1.5)
                symbol_canvas.create_text(10, 10, text=text, fill="#ffffff", font=("Helvetica", 8, "bold"))
            else:
                symbol_canvas.create_rectangle(2, 2, 18, 18, fill=color, outline=b_color, width=1.5)
                symbol_canvas.create_text(10, 10, text=text, fill="#ffffff", font=("Helvetica", 10, "bold"))
                
            lbl = tk.Label(item_frame, text=label_text, bg="#0f172a", fg="#cbd5e1", font=("Helvetica", 9, "bold"))
            lbl.pack(side="left")

        # Start and finish points
        start_x, start_y = self.stage.start
        finish_x, finish_y = self.stage.finish
        
        create_legend_item(legend_frame, "#2ecc71", "S", "Start (จุดเริ่ม)")
        create_legend_item(legend_frame, "#e74c3c", "F", "Finish (ชีส)")
        create_legend_item(legend_frame, "#3b82f6", "R", "Rat (หนู)", is_circle=True, border_color="#1d4ed8")
        create_legend_item(legend_frame, "#e0f2fe", "", "ช่องที่เคยเดิน", is_circle=False, border_color="#bae6fd")
        create_legend_item(legend_frame, "#ef4444", "", "กำแพงที่หนูรู้แล้ว", is_circle=False)
        create_legend_item(legend_frame, "#cbd5e1", "", "กำแพงจริงในสนาม", is_circle=False)

        # Canvas
        self.canvas = tk.Canvas(canvas_frame, bg="#ffffff", bd=0, highlightthickness=0)
        self.canvas.pack(padx=10, pady=10, fill="both", expand=True)
        self.canvas.bind("<Configure>", self.on_resize)

        # Sidebar
        self.sidebar = tk.Frame(self.main_frame, bg="#1e293b", width=320, padx=15, pady=15)
        self.sidebar.pack(side="right", fill="both", expand=False)
        self.sidebar.pack_propagate(False)

        # Title
        title_lbl = tk.Label(
            self.sidebar, text="Rat - Map Version",
            bg="#1e293b", fg="#f8fafc", font=("Helvetica", 14, "bold")
        )
        title_lbl.pack(anchor="w", pady=(0, 2))
        
        subtitle_lbl = tk.Label(
            self.sidebar, text="Integration with man_version maps",
            bg="#1e293b", fg="#64748b", font=("Helvetica", 9, "italic")
        )
        subtitle_lbl.pack(anchor="w", pady=(0, 15))

        # Map source selection
        map_select_frame = tk.LabelFrame(
            self.sidebar, text=" แหล่งข้อมูลแผนที่ ",
            bg="#1e293b", fg="#94a3b8", font=("Helvetica", 9, "bold"),
            padx=10, pady=10
        )
        map_select_frame.pack(fill="x", pady=(0, 15))

        self.map_file_label = tk.Label(
            map_select_frame, text="สร้างแบบสุ่ม (Perfect Maze)",
            bg="#0f172a", fg="#f8fafc", font=("Helvetica", 9), anchor="w", padx=5
        )
        self.map_file_label.pack(fill="x", ipady=4, pady=(0, 5))

        # Buttons inside map selection
        map_btn_frame = tk.Frame(map_select_frame, bg="#1e293b")
        map_btn_frame.pack(fill="x")
        
        browse_btn = tk.Button(
            map_btn_frame, text="เลือกไฟล์แผนที่", bg="#475569", fg="white",
            relief="flat", bd=0, font=("Helvetica", 9, "bold"), cursor="hand2",
            command=self.browse_map_file
        )
        browse_btn.pack(side="left", expand=True, fill="x", padx=(0, 2))

        rand_map_btn = tk.Button(
            map_btn_frame, text="สร้างแบบสุ่ม", bg="#475569", fg="white",
            relief="flat", bd=0, font=("Helvetica", 9, "bold"), cursor="hand2",
            command=self.use_random_maze
        )
        rand_map_btn.pack(side="right", expand=True, fill="x", padx=(2, 0))

        # Seed configuration
        seed_frame = tk.Frame(self.sidebar, bg="#1e293b")
        seed_frame.pack(fill="x", pady=(0, 15))
        
        seed_lbl = tk.Label(
            seed_frame, text="Seed สำหรับสุ่ม:",
            bg="#1e293b", fg="#cbd5e1", font=("Helvetica", 10, "bold")
        )
        seed_lbl.pack(side="left", padx=(0, 5))

        self.seed_entry = tk.Entry(
            seed_frame, bg="#0f172a", fg="#f8fafc",
            insertbackground="white", bd=1, relief="flat",
            font=("Helvetica", 10), width=10
        )
        self.seed_entry.insert(0, str(self.seed_val))
        self.seed_entry.pack(side="left", padx=5, ipady=3)

        # Helper buttons function
        def create_btn(text, bg, active_bg, cmd):
            btn = tk.Button(
                self.sidebar, text=text, bg=bg, fg="white",
                activebackground=active_bg, activeforeground="white",
                relief="flat", bd=0, font=("Helvetica", 10, "bold"),
                cursor="hand2", command=cmd, height=2
            )
            btn.bind("<Enter>", lambda e: btn.configure(bg=active_bg))
            btn.bind("<Leave>", lambda e: btn.configure(bg=bg))
            btn.pack(fill="x", pady=5)
            return btn

        # Buttons
        self.play_btn = create_btn("เล่นอัตโนมัติ", "#059669", "#047857", self.toggle_run)
        create_btn("ขยับทีละก้าว (Step)", "#d97706", "#b45309", self.step_simulation)
        create_btn("รีเซ็ตตัวหนู (Reset)", "#dc2626", "#b91c1c", self.reset_sim)

        # Output Log Dashboard (HUD)
        dashboard_frame = tk.LabelFrame(
            self.sidebar, text=" รายงานสถานะเรียลไทม์ (HUD) ",
            bg="#1e293b", fg="#94a3b8", font=("Helvetica", 10, "bold"),
            bd=2, relief="groove", padx=10, pady=10
        )
        dashboard_frame.pack(fill="both", expand=True, pady=(15, 0))
        
        dashboard_frame.columnconfigure(0, weight=1)
        dashboard_frame.columnconfigure(1, weight=1)
        
        self.action_var = tk.StringVar(value="-")
        self.position_var = tk.StringVar(value="-")
        self.walk_steps_var = tk.StringVar(value="0")
        self.turn_steps_var = tk.StringVar(value="0")
        self.total_steps_var = tk.StringVar(value="0")
        self.cheese_smell_var = tk.StringVar(value="0.0 cm")
        self.compute_time_var = tk.StringVar(value="0.00 ms")
        
        def make_card(parent, title, string_var, row, col, val_color="#f8fafc", bg_color="#0f172a"):
            card = tk.Frame(parent, bg=bg_color, padx=8, pady=8, bd=1, relief="ridge")
            card.grid(row=row, column=col, sticky="nsew", padx=3, pady=3)
            
            lbl_title = tk.Label(card, text=title, bg=bg_color, fg="#64748b", font=("Helvetica", 8, "bold"))
            lbl_title.pack(anchor="w")
            
            lbl_val = tk.Label(card, textvariable=string_var, bg=bg_color, fg=val_color, font=("Consolas", 12, "bold"))
            lbl_val.pack(anchor="w", pady=(2, 0))
            return card

        # Row 0
        make_card(dashboard_frame, "ตำแหน่ง (Row, Col)", self.position_var, 0, 0, "#38bdf8")
        make_card(dashboard_frame, "การตัดสินใจล่าสุด", self.action_var, 0, 1, "#c084fc")
        
        # Row 1
        make_card(dashboard_frame, "เดินจริง (Walks)", self.walk_steps_var, 1, 0, "#60a5fa")
        make_card(dashboard_frame, "หมุนตัว (Turns)", self.turn_steps_var, 1, 1, "#fbbf24")
        
        # Row 2
        make_card(dashboard_frame, "สเต็ปรวม", self.total_steps_var, 2, 0, "#cbd5e1")
        make_card(dashboard_frame, "ระยะชีส (ตรง)", self.cheese_smell_var, 2, 1, "#f43f5e")
        
        # Row 3 (Span 2)
        card_time = tk.Frame(dashboard_frame, bg="#0f172a", padx=8, pady=8, bd=1, relief="ridge")
        card_time.grid(row=3, column=0, columnspan=2, sticky="ew", padx=3, pady=3)
        
        lbl_time_title = tk.Label(card_time, text="เวลาคำนวณสะสม", bg="#0f172a", fg="#64748b", font=("Helvetica", 8, "bold"))
        lbl_time_title.pack(side="left")
        
        lbl_time_val = tk.Label(card_time, textvariable=self.compute_time_var, bg="#0f172a", fg="#34d399", font=("Consolas", 11, "bold"))
        lbl_time_val.pack(side="right")

    def log_output(self, text):
        print(text)

    def on_resize(self, event):
        margin = 10
        available_w = max(50, event.width - margin * 2)
        available_h = max(50, event.height - margin * 2)
        
        new_size = min(available_w, available_h)
        new_cell_size = max(5, new_size // self.grid_size)
        
        self.cell_size = new_cell_size
        self.offset_x = (event.width - (self.grid_size * self.cell_size)) // 2
        self.offset_y = (event.height - (self.grid_size * self.cell_size)) // 2
        self.draw_maze()

    def draw_maze(self):
        self.canvas.delete("all")
        cs = self.cell_size
        ox = self.offset_x
        oy = self.offset_y
        
        # Track visited positions
        self.visited_cells.add(self.rat.position)

        start_x, start_y = self.stage.start
        finish_x, finish_y = self.stage.finish

        # Draw cells
        for r in range(self.grid_size):
            for c in range(self.grid_size):
                x1 = c * cs + ox
                y1 = r * cs + oy
                x2 = x1 + cs
                y2 = y1 + cs
                
                color = "#ffffff"
                outline_color = "#e2e8f0"
                
                if (r, c) in self.visited_cells:
                    color = "#e0f2fe"
                    outline_color = "#bae6fd"
                
                if (c, r) == (start_x, start_y):
                    color = "#2ecc71"
                    outline_color = "#27ae60"
                elif (c, r) == (finish_x, finish_y):
                    color = "#e74c3c"
                    outline_color = "#c0392b"
                    
                self.canvas.create_rectangle(
                    x1, y1, x2, y2,
                    fill=color, outline=outline_color, width=1
                )
                
                if (c, r) == (start_x, start_y):
                    self.canvas.create_text(x1 + cs/2, y1 + cs/2, text="S", fill="#ffffff", font=("Helvetica", 10, "bold"))
                elif (c, r) == (finish_x, finish_y):
                    self.canvas.create_text(x1 + cs/2, y1 + cs/2, text="F", fill="#ffffff", font=("Helvetica", 10, "bold"))

        # Draw walls
        # Boundary walls and cell-to-cell walls
        for r in range(self.grid_size + 1):
            for c in range(self.grid_size):
                x1 = c * cs + ox
                y1 = r * cs + oy
                x2 = (c + 1) * cs + ox
                y2 = r * cs + oy
                
                # Check horizontal wall
                has_actual_wall = False
                if r == 0 or r == self.grid_size:
                    has_actual_wall = True
                else:
                    has_actual_wall = not self.stage.is_open((c, r-1), (c, r))
                    
                if has_actual_wall:
                    self.canvas.create_line(x1, y1, x2, y2, fill="#94a3b8", width=1.5)
                
                if self.rat.known_h_walls[r, c]:
                    self.canvas.create_line(x1, y1, x2, y2, fill="#ef4444", width=3.5, capstyle="projecting")

        for r in range(self.grid_size):
            for c in range(self.grid_size + 1):
                x1 = c * cs + ox
                y1 = r * cs + oy
                x2 = c * cs + ox
                y2 = (r + 1) * cs + oy
                
                # Check vertical wall
                has_actual_wall = False
                if c == 0 or c == self.grid_size:
                    has_actual_wall = True
                else:
                    has_actual_wall = not self.stage.is_open((c-1, r), (c, r))
                    
                if has_actual_wall:
                    self.canvas.create_line(x1, y1, x2, y2, fill="#94a3b8", width=1.5)
                
                if self.rat.known_v_walls[r, c]:
                    self.canvas.create_line(x1, y1, x2, y2, fill="#ef4444", width=3.5, capstyle="projecting")

        # Draw Rat
        rr, rc = self.rat.position
        rx = rc * cs + cs/2 + ox
        ry = rr * cs + cs/2 + oy
        radius = cs * 0.35
        
        self.canvas.create_oval(
            rx - radius, ry - radius,
            rx + radius, ry + radius,
            fill="#3b82f6", outline="#1d4ed8", width=1.5
        )
        
        dr, dc = self.rat.facing
        nx = rx + dc * radius * 1.2
        ny = ry + dr * radius * 1.2
        self.canvas.create_line(rx, ry, nx, ny, fill="#ffffff", width=2.5)

if __name__ == "__main__":
    root = tk.Tk()
    root.resizable(True, True)
    app = ManMapSimulator(root)
    root.mainloop()

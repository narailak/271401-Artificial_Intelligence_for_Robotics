import time
import tkinter as tk
from tkinter import messagebox
import numpy as np

# Import functions and configurations from field.py
from field import generate_maze, START, FINISH, GRID_SIZE, CELL_SIZE_CM, FREE, WALL
# Import Rat class from rat.py
from rat import Rat, NORTH, SOUTH, WEST, EAST

class RatMazeSimulator:
    def __init__(self, root):
        self.root = root
        self.root.title("Micromouse Maze Simulator (Rat & Controller) - CMU 271401")
        self.root.configure(bg="#0f172a") # Slate 900
        
        # Grid settings
        self.cell_size = 20
        self.offset_x = 0
        self.offset_y = 0
        
        # Simulation control
        self.seed_val = 70
        self.running = False
        self.delay_ms = 100 # Delay between steps in ms
        self.step_count = 0
        self.total_compute_time_ms = 0.0
        
        # Initialize maze
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

        # Stop any active simulation
        self.running = False
        self.step_count = 0
        self.total_compute_time_ms = 0.0
        self.visited_cells = set()

        # Generate actual maze walls
        self.h_walls, self.v_walls = generate_maze(size=GRID_SIZE, start=START, goal=FINISH, seed=self.seed_val)
        
        # Initialize Rat from rat.py
        self.rat = Rat(start_pos=START, goal_pos=FINISH, grid_size=GRID_SIZE)
        
        # Sense the initial cell front wall
        self.sense_current_front()

        if not init:
            self.draw_maze()
            self.log_output(f"\n[System] สร้างเขาวงกตและตัวหนูสำเร็จ (Seed: {self.seed_val})")
            self.log_output(f" - พิกัดเริ่มต้นของหนู: {self.rat.position} (หันหน้าไปทางขวา)")

    def sense_current_front(self):
        """Senses the wall in front of the rat and updates its memory"""
        r, c = self.rat.position
        dr, dc = self.rat.facing
        
        # Determine if there is a wall in front in the actual maze
        wall_in_front = False
        if self.rat.facing == NORTH:
            wall_in_front = self.h_walls[r, c]
        elif self.rat.facing == SOUTH:
            wall_in_front = self.h_walls[r + 1, c]
        elif self.rat.facing == WEST:
            wall_in_front = self.v_walls[r, c]
        elif self.rat.facing == EAST:
            wall_in_front = self.v_walls[r, c + 1]
            
        self.rat.sense_and_update(wall_in_front)

    def step_simulation(self):
        """Executes one step of the simulation"""
        if self.rat.position == FINISH:
            self.running = False
            self.log_output(f"\n[Victory!] หนูเดินถึงเป้าหมาย (ชีส) สำเร็จ!")
            self.log_output(f" - ใช้เวลาในการประมวลผลคำนวณทั้งหมด: {self.total_compute_time_ms:.4f} ms")
            self.log_output(f" - จำนวนขั้นตอน (Decision Steps): {self.step_count} รอบ")
            messagebox.showinfo("สำเร็จ", f"หนูถึงเส้นชัยแล้ว!\nใช้เวลาตัดสินใจ: {self.step_count} รอบ\nเวลาประมวลผลเฉลี่ยต่อรอบ: {self.total_compute_time_ms/max(1, self.step_count):.4f} ms")
            return False

        # Measure computation time for this single decision step
        start_time = time.perf_counter()
        
        # Step 1: Sense the front
        self.sense_current_front()
        
        # Step 2: Make decision
        action = self.rat.decide_next_action()
        
        # End timer
        elapsed_ms = (time.perf_counter() - start_time) * 1000.0
        self.total_compute_time_ms += elapsed_ms
        
        if action is None:
            self.running = False
            self.log_output("[Error] อัลกอริทึมไม่สามารถหาเส้นทางได้")
            return False
            
        # Apply the action
        self.rat.apply_action(action)
        self.step_count += 1
        
        # Print logs occasionally
        dist_cheese = self.rat.get_distance_to_cheese()
        self.log_output(
            f"รอบที่ {self.step_count:3d}: ตัดสินใจ = {action:8s} | "
            f"ตำแหน่ง = {str(self.rat.position):8s} | "
            f"ระยะถึงชีส = {dist_cheese:6.1f} cm | "
            f"เวลาคิด = {elapsed_ms:.4f} ms"
        )
        
        # Sense again after moving/turning
        self.sense_current_front()
        
        self.draw_maze()
        return True

    def run_loop(self):
        """Continuous simulation loop"""
        if self.running:
            success = self.step_simulation()
            if success:
                self.root.after(self.delay_ms, self.run_loop)
            else:
                self.running = False
                self.play_btn.config(text="เล่นอัตโนมัติ")

    def toggle_run(self):
        """Toggles play/pause state of simulation"""
        if self.running:
            self.running = False
            self.play_btn.config(text="เล่นอัตโนมัติ")
            self.log_output("[System] หยุดการจำลองชั่วคราว")
        else:
            if self.rat.position == FINISH:
                self.generate_new_maze()
            self.running = True
            self.play_btn.config(text="หยุดชั่วคราว")
            self.log_output("[System] เริ่มการจำลองการเดิน...")
            self.run_loop()

    def reset_sim(self):
        """Resets the simulation on the current maze"""
        self.generate_new_maze(init=True)
        self.draw_maze()
        self.log_output("\n[System] รีเซ็ตตำแหน่งหนูและล้างความจำแผนที่เรียบร้อย")

    def setup_layout(self):
        # Create main container frame
        self.main_frame = tk.Frame(self.root, bg="#0f172a", padx=15, pady=15)
        self.main_frame.pack(fill="both", expand=True)

        # Canvas Frame
        canvas_frame = tk.LabelFrame(
            self.main_frame, text=" สนามจำลอง (เส้นดำ = กำแพงจริง | เส้นแดงหนา = กำแพงที่หนูตรวจเจอแล้ว | ช่องสีฟ้า = เส้นทางที่หนูเคยเดิน) ",
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

        create_legend_item(legend_frame, "#2ecc71", "S", "Start (จุดเริ่ม)")
        create_legend_item(legend_frame, "#e74c3c", "F", "Finish (ชีส)")
        create_legend_item(legend_frame, "#3b82f6", "R", "Rat (หนู)", is_circle=True, border_color="#1d4ed8")
        create_legend_item(legend_frame, "#e0f2fe", "", "ช่องที่หนูเคยเดินผ่าน", is_circle=False, border_color="#bae6fd")
        create_legend_item(legend_frame, "#ef4444", "", "กำแพงที่หนูรู้แล้ว", is_circle=False)
        create_legend_item(legend_frame, "#cbd5e1", "", "กำแพงจริงในสนาม", is_circle=False)

        # Canvas
        self.canvas = tk.Canvas(
            canvas_frame, bg="#ffffff", bd=0, highlightthickness=0
        )
        self.canvas.pack(padx=10, pady=10, fill="both", expand=True)
        self.canvas.bind("<Configure>", self.on_resize)

        # Sidebar Frame
        self.sidebar = tk.Frame(self.main_frame, bg="#1e293b", width=320, padx=15, pady=15)
        self.sidebar.pack(side="right", fill="both", expand=False)
        self.sidebar.pack_propagate(False)

        # Title
        title_lbl = tk.Label(
            self.sidebar, text="Rat Simulator",
            bg="#1e293b", fg="#f8fafc", font=("Helvetica", 16, "bold")
        )
        title_lbl.pack(anchor="w", pady=(0, 2))
        
        subtitle_lbl = tk.Label(
            self.sidebar, text="Phase 2: Online Pathfinding & Sensing",
            bg="#1e293b", fg="#64748b", font=("Helvetica", 9, "italic")
        )
        subtitle_lbl.pack(anchor="w", pady=(0, 20))

        # Seed configuration
        seed_frame = tk.Frame(self.sidebar, bg="#1e293b")
        seed_frame.pack(fill="x", pady=(0, 15))
        
        seed_lbl = tk.Label(
            seed_frame, text="Seed เขาวงกต:",
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
        create_btn("สร้างเขาวงกตใหม่", "#2563eb", "#1d4ed8", self.generate_new_maze)
        self.play_btn = create_btn("เล่นอัตโนมัติ", "#059669", "#047857", self.toggle_run)
        create_btn("ขยับทีละก้าว (Step)", "#d97706", "#b45309", self.step_simulation)
        create_btn("รีเซ็ตตัวหนู (Reset)", "#dc2626", "#b91c1c", self.reset_sim)

        # Output Log Box
        log_lbl = tk.Label(
            self.sidebar, text="บันทึกการนำทางออนไลน์:",
            bg="#1e293b", fg="#cbd5e1", font=("Helvetica", 10, "bold")
        )
        log_lbl.pack(anchor="w", pady=(15, 5))

        # Text area with scrollbar
        self.log_text = tk.Text(
            self.sidebar, bg="#0f172a", fg="#34d399", # Green terminal text
            insertbackground="white", bd=0, relief="flat",
            font=("Consolas", 8), height=15
        )
        self.log_text.pack(fill="both", expand=True, pady=(0, 5))
        
        scrollbar = tk.Scrollbar(self.log_text, bg="#0f172a")
        scrollbar.pack(side="right", fill="y")
        self.log_text.config(yscrollcommand=scrollbar.set)
        scrollbar.config(command=self.log_text.yview)

        # Initial info
        self.log_output("ระบบเริ่มต้นจำลองสถานการณ์\n"
                        f"เขาวงกตขนาด {GRID_SIZE}x{GRID_SIZE} ช่อง (ช่องละ {CELL_SIZE_CM} cm)")

    def log_output(self, text):
        self.log_text.insert(tk.END, text + "\n")
        self.log_text.see(tk.END)

    def on_resize(self, event):
        margin = 10
        available_w = max(50, event.width - margin * 2)
        available_h = max(50, event.height - margin * 2)
        
        new_size = min(available_w, available_h)
        new_cell_size = max(5, new_size // GRID_SIZE)
        
        self.cell_size = new_cell_size
        self.offset_x = (event.width - (GRID_SIZE * self.cell_size)) // 2
        self.offset_y = (event.height - (GRID_SIZE * self.cell_size)) // 2
        self.draw_maze()

    def draw_maze(self):
        self.canvas.delete("all")
        cs = self.cell_size
        ox = self.offset_x
        oy = self.offset_y
        
        # Track visited positions for drawing
        # We can reconstruct visited cells from the rat's path history (or we can just track visited cells in a set in main.py)
        # For simplicity, let's keep track of visited cells using a set or list in the simulation.
        # Let's check if we have a set of visited cells. Since we don't, we can add a self.visited_cells set.
        if not hasattr(self, 'visited_cells'):
            self.visited_cells = set()
        self.visited_cells.add(self.rat.position)

        # Draw cells
        for r in range(GRID_SIZE):
            for c in range(GRID_SIZE):
                x1 = c * cs + ox
                y1 = r * cs + oy
                x2 = x1 + cs
                y2 = y1 + cs
                
                color = "#ffffff" # Default white background for viewers
                outline_color = "#e2e8f0" # Light gray grid lines
                
                if (r, c) in self.visited_cells:
                    color = "#e0f2fe" # Light blue for visited cells
                    outline_color = "#bae6fd"
                
                if (r, c) == START:
                    color = "#2ecc71"
                    outline_color = "#27ae60"
                elif (r, c) == FINISH:
                    color = "#e74c3c"
                    outline_color = "#c0392b"
                    
                self.canvas.create_rectangle(
                    x1, y1, x2, y2,
                    fill=color, outline=outline_color, width=1
                )
                
                if (r, c) == START:
                    self.canvas.create_text(x1 + cs/2, y1 + cs/2, text="S", fill="#ffffff", font=("Helvetica", 10, "bold"))
                elif (r, c) == FINISH:
                    self.canvas.create_text(x1 + cs/2, y1 + cs/2, text="F", fill="#ffffff", font=("Helvetica", 10, "bold"))

        # Draw walls
        # Actual walls are drawn in thin dark gray/black so the viewer sees the entire maze.
        # Discovered walls are drawn in thick red so the viewer knows what the rat has found.
        
        # Horizontal walls
        for r in range(GRID_SIZE + 1):
            for c in range(GRID_SIZE):
                x1 = c * cs + ox
                y1 = r * cs + oy
                x2 = (c + 1) * cs + ox
                y2 = r * cs + oy
                
                if self.h_walls[r, c]:
                    self.canvas.create_line(x1, y1, x2, y2, fill="#94a3b8", width=1.5) # Thin gray actual wall
                
                if self.rat.known_h_walls[r, c]:
                    self.canvas.create_line(x1, y1, x2, y2, fill="#ef4444", width=3.5, capstyle="projecting") # Thick red discovered wall

        # Vertical walls
        for r in range(GRID_SIZE):
            for c in range(GRID_SIZE + 1):
                x1 = c * cs + ox
                y1 = r * cs + oy
                x2 = c * cs + ox
                y2 = (r + 1) * cs + oy
                
                if self.v_walls[r, c]:
                    self.canvas.create_line(x1, y1, x2, y2, fill="#94a3b8", width=1.5)
                
                if self.rat.known_v_walls[r, c]:
                    self.canvas.create_line(x1, y1, x2, y2, fill="#ef4444", width=3.5, capstyle="projecting")

        # Draw Rat
        rr, rc = self.rat.position
        rx = rc * cs + cs/2 + ox
        ry = rr * cs + cs/2 + oy
        radius = cs * 0.35
        
        # Circle representation
        self.canvas.create_oval(
            rx - radius, ry - radius,
            rx + radius, ry + radius,
            fill="#3b82f6", outline="#1d4ed8", width=1.5
        )
        
        # Direction indicator line (nose/pointer)
        dr, dc = self.rat.facing
        nx = rx + dc * radius * 1.2
        ny = ry + dr * radius * 1.2
        self.canvas.create_line(rx, ry, nx, ny, fill="#ffffff", width=2.5)

if __name__ == "__main__":
    root = tk.Tk()
    root.resizable(True, True)
    app = RatMazeSimulator(root)
    root.mainloop()

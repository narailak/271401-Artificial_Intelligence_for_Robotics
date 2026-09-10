"""
Micromouse Maze Simulation - Phase 1 (Map + Mouse only)
========================================================
- สร้างสนามเขาวงกต (Maze) ขนาด 30x30 ช่อง (16x16 cm ต่อช่อง)
- ใช้อัลกอริทึม Perfect Maze (Recursive Backtracker) -> ทางเดินแคบ 1 ช่อง
  คดเคี้ยวจริงจัง ยากกว่าการสุ่มวางกำแพงแบบกระจายจุดมาก
  และการันตี 100% ว่ามีเส้นทางจาก Start ไป Finish ได้จริง (ตรวจซ้ำด้วย A*)
- สร้างโครงสร้างตัวหนู (Mouse) รู้ตำแหน่งตัวเองและเป้าหมาย พร้อมคำนวณ
  ระยะทางตรงทางทฤษฎี (Manhattan/Euclidean แบบมองทะลุกำแพง)
- แสดงผลด้วย Matplotlib

หมายเหตุ: เฟสนี้ยังไม่คำนวณ/แสดง Shortest Path หรือให้หนูเดินจริง
          (ฟังก์ชัน A* และ mouse.plan_path() เตรียมไว้พร้อมใช้ในเฟสถัดไป)
"""

import numpy as np
import random
import heapq
import time
import tkinter as tk
from tkinter import messagebox
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.colors import ListedColormap

# =========================================================
# 1. ค่าคงที่ของสนาม (Maze Configuration)
# =========================================================
GRID_SIZE = 30          # 30x30 ช่อง
CELL_SIZE_CM = 16       # แต่ละช่อง 16x16 cm (ใช้ตอนคำนวณระยะทางจริงถ้าต้องการ)

START = (0, 0)                              # จุดเริ่มต้น (แถว, คอลัมน์)
FINISH = (GRID_SIZE - 1, GRID_SIZE - 1)     # จุดเป้าหมาย (มีชีสอยู่)

# ค่าคงที่แทนการมีกำแพง (สำหรับเก็บใน arrays และ visualization)
FREE = False
WALL = True


# =========================================================
# 2. A* Algorithm สำหรับหาเส้นทางที่สั้นที่สุดที่หลบสิ่งกีดขวางจริง
#    (ยังไม่ถูกเรียกใช้แสดงผลในเฟสนี้ แต่เตรียมพร้อมไว้)
# =========================================================
def manhattan_distance(a, b):
    """ระยะทางแบบ Manhattan (เส้นตรงทางทฤษฎี ไม่หลบสิ่งกีดขวาง)"""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def euclidean_distance(a, b):
    """ระยะทางแบบเส้นตรง (Euclidean) - ใช้เป็นเส้นนำสายตาทางทฤษฎีเท่านั้น"""
    return np.hypot(a[0] - b[0], a[1] - b[1])


def get_neighbors(pos, h_walls, v_walls):
    """คืนช่องข้างเคียงที่เดินได้ (4 ทิศ: บน ล่าง ซ้าย ขวา) โดยเช็คจากกำแพงกั้น"""
    r, c = pos
    rows, cols = v_walls.shape[0], h_walls.shape[1]
    result = []
    # Up: (r - 1, c)
    if r > 0 and not h_walls[r, c]:
        result.append((r - 1, c))
    # Down: (r + 1, c)
    if r < rows - 1 and not h_walls[r + 1, c]:
        result.append((r + 1, c))
    # Left: (r, c - 1)
    if c > 0 and not v_walls[r, c]:
        result.append((r, c - 1))
    # Right: (r, c + 1)
    if c < cols - 1 and not v_walls[r, c + 1]:
        result.append((r, c + 1))
    return result


def a_star_search(h_walls, v_walls, start, goal):
    """
    A* search: หาเส้นทางที่สั้นที่สุดที่หลบกำแพงจริง
    คืนค่า: list ของ (row, col) จาก start -> goal หรือ None ถ้าไม่มีเส้นทาง
    """
    open_set = []
    heapq.heappush(open_set, (0 + manhattan_distance(start, goal), 0, start))
    came_from = {}
    g_score = {start: 0}
    visited = set()

    while open_set:
        _, current_g, current = heapq.heappop(open_set)

        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            return path

        if current in visited:
            continue
        visited.add(current)

        for neighbor in get_neighbors(current, h_walls, v_walls):
            tentative_g = current_g + 1
            if tentative_g < g_score.get(neighbor, float("inf")):
                g_score[neighbor] = tentative_g
                came_from[neighbor] = current
                f_score = tentative_g + manhattan_distance(neighbor, goal)
                heapq.heappush(open_set, (f_score, tentative_g, neighbor))

    return None  # ไม่มีเส้นทาง


def count_turns(path):
    """นับจำนวนจุดเลี้ยว (decision points) บนเส้นทาง"""
    if len(path) < 3:
        return 0
    turns = 0
    prev_direction = None
    for i in range(1, len(path)):
        dr = path[i][0] - path[i - 1][0]
        dc = path[i][1] - path[i - 1][1]
        direction = (dr, dc)
        if prev_direction is not None and direction != prev_direction:
            turns += 1
        prev_direction = direction
    return turns


# =========================================================
# 3. สร้างเขาวงกต (Maze Generation) แบบ "Perfect Maze"
#    ใช้ Recursive Backtracker (DFS แบบสุ่ม) บนเซลล์ -> ผนังบางกั้นระหว่างช่อง
#    มีทางเดียวเท่านั้นระหว่างจุดสองจุดใด ๆ
#    (การันตี 100% ว่ามีทางเดินถึงกันเสมอ)
# =========================================================
def generate_perfect_maze(rows, cols, rng):
    """
    สร้าง Perfect Maze บนโครงสร้างผนังบางระหว่างช่อง (h_walls และ v_walls)
    - เริ่มต้นด้วยกำแพงกั้นในทุกด้านของทุกช่อง (True)
    - ใช้ Recursive Backtracker (DFS) วิ่งผ่านเซลล์ทั้งหมดแล้วเจาะกำแพงออก (False)
    - คืนค่า h_walls และ v_walls
    """
    h_walls = np.ones((rows + 1, cols), dtype=bool) * WALL
    v_walls = np.ones((rows, cols + 1), dtype=bool) * WALL

    visited = np.zeros((rows, cols), dtype=bool)
    stack = [(0, 0)]
    visited[0, 0] = True

    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]

    while stack:
        r, c = stack[-1]
        unvisited_neighbors = []
        for dr, dc in directions:
            nr, nc = r + dr, c + dc
            if 0 <= nr < rows and 0 <= nc < cols and not visited[nr, nc]:
                unvisited_neighbors.append((nr, nc))

        if unvisited_neighbors:
            nr, nc = rng.choice(unvisited_neighbors)
            # เจาะกำแพงระหว่างเซลล์ (r, c) และ (nr, nc)
            if r == nr:
                # วิ่งซ้าย/ขวา -> เจาะกำแพงแนวตั้ง
                v_walls[r, max(c, nc)] = FREE
            else:
                # วิ่งขึ้น/ลง -> เจาะกำแพงแนวนอน
                h_walls[max(r, nr), c] = FREE
            visited[nr, nc] = True
            stack.append((nr, nc))
        else:
            stack.pop()

    return h_walls, v_walls


def generate_maze(size=GRID_SIZE, start=START, goal=FINISH, seed=None):
    """
    สร้างเขาวงกตแบบ Perfect Maze ที่มีผนังกั้นระหว่างเซลล์ และการันตีเส้นทางจาก start ไป goal
    คืนค่า: h_walls, v_walls
    """
    rng = random.Random(seed)
    h_walls, v_walls = generate_perfect_maze(size, size, rng)

    check_path = a_star_search(h_walls, v_walls, start, goal)
    if check_path is None:
        raise RuntimeError("เกิดข้อผิดพลาด: ไม่สามารถการันตีเส้นทางได้ กรุณาลอง seed อื่น")

    print("[Maze Generator] สร้าง Perfect Maze สำเร็จ (ผนังแบบบางกั้นระหว่างช่อง, การันตีเส้นทางเชื่อมถึงกัน 100%)")

    return h_walls, v_walls


# =========================================================
# 4. โครงสร้างตัวหนู (Mouse)
# =========================================================
class Mouse:
    """
    ตัวแทนหนู (Micromouse)
    - รู้พิกัดปัจจุบันและพิกัดเป้าหมาย (Finish/Cheese)
    - คำนวณระยะทางตรงทางทฤษฎี (Manhattan/Euclidean) ได้ทันที
      แต่ระยะนี้ "มองทะลุกำแพง" ใช้เป็นแค่เส้นนำสายตา (heuristic) เท่านั้น
    - หาเส้นทางจริงที่หลบสิ่งกีดขวางด้วย A* ผ่าน plan_path()
    """

    def __init__(self, start_pos, goal_pos, h_walls, v_walls):
        self.start_pos = start_pos
        self.position = start_pos          # ตำแหน่งปัจจุบันของหนู
        self.goal_pos = goal_pos           # ตำแหน่งเป้าหมาย (ชีส)
        self.h_walls = h_walls             # ผนังแนวนอนที่หนู "รู้"
        self.v_walls = v_walls             # ผนังแนวตั้งที่หนู "รู้"
        self.planned_path = None
        self.direction = "NONE"

    def theoretical_distance(self, method="manhattan"):
        """ระยะทางตรงทางทฤษฎีจากตำแหน่งปัจจุบันไปเป้าหมาย (ไม่หลบกำแพง)"""
        if method == "euclidean":
            return euclidean_distance(self.position, self.goal_pos)
        return manhattan_distance(self.position, self.goal_pos)

    def plan_path(self):
        """ใช้ A* คำนวณเส้นทางจริงที่หลบสิ่งกีดขวาง (เตรียมไว้สำหรับเฟสถัดไป)"""
        path = a_star_search(self.h_walls, self.v_walls, self.position, self.goal_pos)
        self.planned_path = path
        return path

    def __repr__(self):
        return (f"Mouse(pos={self.position}, goal={self.goal_pos}, "
                f"planned_path_len={len(self.planned_path) if self.planned_path else None})")


# =========================================================
# 5. Visualization ด้วย Matplotlib
# =========================================================
def visualize_maze(h_walls, v_walls, start, goal, mouse_pos=None, path=None,
                    show_path=False, title="Micromouse Maze - Initial State"):
    """
    วาดสนามเขาวงกตด้วย Matplotlib
    - พื้นหลัง: สีขาว (ทางเดิน)
    - กำแพง: เส้นสีดำกั้นระหว่างช่อง
    - Start: สีเขียว | Finish (มีชีส): สีแดง
    - หนู: วงกลมสีน้ำเงิน
    """
    rows, cols = v_walls.shape[0], h_walls.shape[1]
    fig, ax = plt.subplots(figsize=(9, 9))

    # กำหนดอาณาเขตและสีพื้นหลังขาวสำหรับทางเดินทั้งหมด
    ax.set_xlim(-0.5, cols - 0.5)
    ax.set_ylim(rows - 0.5, -0.5) # origin="upper"
    ax.set_facecolor("white")

    # กำหนด grid lines จางๆ
    ax.set_xticks(np.arange(-0.5, cols, 1), minor=True)
    ax.set_yticks(np.arange(-0.5, rows, 1), minor=True)
    ax.grid(which="minor", color="#e2e8f0", linewidth=0.5)
    ax.tick_params(which="both", bottom=False, left=False,
                    labelbottom=False, labelleft=False)

    # วาดกำแพง
    # Horizontal walls
    for r in range(rows + 1):
        for c in range(cols):
            if h_walls[r, c]:
                ax.plot([c - 0.5, c + 0.5], [r - 0.5, r - 0.5], color="black", linewidth=3)
    # Vertical walls
    for r in range(rows):
        for c in range(cols + 1):
            if v_walls[r, c]:
                ax.plot([c - 0.5, c - 0.5], [r - 0.5, r + 0.5], color="black", linewidth=3)

    if show_path and path:
        ys = [p[0] for p in path]
        xs = [p[1] for p in path]
        ax.plot(xs, ys, color="orange", linewidth=2.5, alpha=0.8, zorder=2,
                 label="Shortest Path (A*)")

    ax.add_patch(patches.Rectangle((start[1] - 0.5, start[0] - 0.5), 1, 1,
                                    facecolor="limegreen", edgecolor="darkgreen", alpha=0.6, zorder=1))
    ax.text(start[1], start[0], "S", ha="center", va="center",
            fontsize=10, fontweight="bold", color="black", zorder=4)

    ax.add_patch(patches.Rectangle((goal[1] - 0.5, goal[0] - 0.5), 1, 1,
                                    facecolor="tomato", edgecolor="darkred", alpha=0.6, zorder=1))
    ax.text(goal[1], goal[0], "F", ha="center", va="center",
            fontsize=10, fontweight="bold", color="black", zorder=4)

    if mouse_pos is not None:
        ax.plot(mouse_pos[1], mouse_pos[0], marker="o", markersize=16,
                 markerfacecolor="royalblue", markeredgecolor="navy", zorder=5)
        ax.text(mouse_pos[1], mouse_pos[0], "M", ha="center", va="center",
                fontsize=9, fontweight="bold", color="white", zorder=6)

    legend_elements = [
        patches.Patch(facecolor="white", edgecolor="gray", label="Free Space"),
        plt.Line2D([0], [0], color="black", linewidth=3, label="Wall / Obstacle"),
        patches.Patch(facecolor="limegreen", edgecolor="darkgreen", alpha=0.6, label="Start"),
        patches.Patch(facecolor="tomato", edgecolor="darkred", alpha=0.6, label="Finish (Cheese)"),
    ]
    if mouse_pos is not None:
        legend_elements.append(
            plt.Line2D([0], [0], marker="o", color="w", markerfacecolor="royalblue",
                       markersize=10, label="Mouse"))
    if show_path and path:
        legend_elements.append(
            plt.Line2D([0], [0], color="orange", linewidth=2.5, label="Shortest Path (A*)"))

    ax.legend(handles=legend_elements, loc="upper center",
              bbox_to_anchor=(0.5, -0.02), ncol=3, frameon=False, fontsize=9)

    ax.set_title(title, fontsize=13, fontweight="bold")
    plt.tight_layout()
    return fig, ax


# =========================================================
# 6. GUI Class (Tkinter implementation)
# =========================================================
class MazeGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Micromouse Maze Simulator - CMU 271401")
        self.root.configure(bg="#0f172a") # Slate 900
        
        # Grid settings
        self.cell_size = 20 # Initial cell size
        self.canvas_size = GRID_SIZE * self.cell_size
        self.offset_x = 0
        self.offset_y = 0
        
        # Initial maze config
        self.seed_val = 70
        self.show_path = False
        
        # Set window initial size and minsize to allow resizing
        self.root.geometry("950x650")
        self.root.minsize(800, 550)

        # Initialize maze and mouse
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

        # Generate maze & initialize mouse
        self.h_walls, self.v_walls = generate_maze(size=GRID_SIZE, start=START, goal=FINISH, seed=self.seed_val)
        self.mouse = Mouse(start_pos=START, goal_pos=FINISH, h_walls=self.h_walls, v_walls=self.v_walls)
        self.planned_path = None
        
        # Pre-plan path
        self.planned_path = self.mouse.plan_path()
        
        if not init:
            self.draw_maze()
            self.log_output(f"\n[System] สร้างเขาวงกตสำเร็จ (Seed: {self.seed_val})")
            h_count = np.sum(self.h_walls)
            v_count = np.sum(self.v_walls)
            total_walls = h_count + v_count
            total_possible = (GRID_SIZE + 1) * GRID_SIZE + GRID_SIZE * (GRID_SIZE + 1)
            self.log_output(f" - กำแพง: {total_walls} ส่วน จากทั้งหมด {total_possible} ({total_walls/total_possible*100:.1f}%)")

    def measure_computation_time(self):
        self.log_output("\n=== เริ่มวัดเวลาประมวลผล (Computation Time) 5 ครั้ง ===")
        times = []
        path = None
        for i in range(5):
            start = time.perf_counter()
            path = self.mouse.plan_path()
            end = time.perf_counter()
            elapsed_ms = (end - start) * 1000
            times.append(elapsed_ms)
            self.log_output(f"ครั้งที่ {i+1}: {elapsed_ms:.4f} ms | ความยาวเส้นทาง: {len(path)} ช่อง")
        
        avg_time = sum(times) / len(times)
        self.log_output(f"เวลาประมวลผลเฉลี่ย: {avg_time:.4f} ms")
        self.log_output("=====================================================")
        
        self.planned_path = path
        self.draw_maze()

    def toggle_path(self):
        self.show_path = not self.show_path
        self.draw_maze()
        state = "แสดง" if self.show_path else "ซ่อน"
        self.log_output(f"[System] เปลี่ยนสถานะเส้นทาง: {state}")

    def setup_layout(self):
        # Create a container frame
        self.main_frame = tk.Frame(self.root, bg="#0f172a", padx=15, pady=15)
        self.main_frame.pack(fill="both", expand=True)

        # Canvas Frame
        canvas_frame = tk.LabelFrame(
            self.main_frame, text=" แผนที่เขาวงกต (30x30 ช่อง, ช่องละ 16 cm) ",
            bg="#0f172a", fg="#94a3b8", font=("Helvetica", 10, "bold"),
            bd=2, relief="groove", labelanchor="nw"
        )
        canvas_frame.pack(side="left", padx=(0, 15), fill="both", expand=True)

        # Legend Frame under canvas (packed first with side="bottom" so it stays at the bottom)
        legend_frame = tk.Frame(canvas_frame, bg="#0f172a")
        legend_frame.pack(side="bottom", fill="x", padx=10, pady=(5, 10))

        # Helper to create legend item
        def create_legend_item(parent, color, text, label_text, is_circle=False):
            item_frame = tk.Frame(parent, bg="#0f172a")
            item_frame.pack(side="left", expand=True)
            
            symbol_canvas = tk.Canvas(item_frame, width=20, height=20, bg="#0f172a", bd=0, highlightthickness=0)
            symbol_canvas.pack(side="left", padx=5)
            
            if is_circle:
                symbol_canvas.create_oval(2, 2, 18, 18, fill=color, outline="#1d4ed8", width=1.5)
                symbol_canvas.create_text(10, 10, text=text, fill="#ffffff", font=("Helvetica", 8, "bold"))
            else:
                symbol_canvas.create_rectangle(2, 2, 18, 18, fill=color, outline="", width=0)
                symbol_canvas.create_text(10, 10, text=text, fill="#ffffff", font=("Helvetica", 10, "bold"))
                
            lbl = tk.Label(item_frame, text=label_text, bg="#0f172a", fg="#cbd5e1", font=("Helvetica", 9, "bold"))
            lbl.pack(side="left")

        create_legend_item(legend_frame, "#2ecc71", "S", "Start (จุดเริ่ม)")
        create_legend_item(legend_frame, "#e74c3c", "F", "Finish / Cheese (ชีส)")
        create_legend_item(legend_frame, "#3b82f6", "M", "Mouse (หนู)", is_circle=True)

        # Canvas occupying the remaining space
        self.canvas = tk.Canvas(
            canvas_frame, bg="#1e293b", bd=0, highlightthickness=0
        )
        self.canvas.pack(padx=10, pady=10, fill="both", expand=True)
        self.canvas.bind("<Configure>", self.on_resize)

        # Sidebar Frame
        self.sidebar = tk.Frame(self.main_frame, bg="#1e293b", width=300, padx=15, pady=15)
        self.sidebar.pack(side="right", fill="both", expand=False)
        self.sidebar.pack_propagate(False) # Keep width fixed

        # Title
        title_lbl = tk.Label(
            self.sidebar, text="Micromouse Maze",
            bg="#1e293b", fg="#f8fafc", font=("Helvetica", 16, "bold")
        )
        title_lbl.pack(anchor="w", pady=(0, 2))
        
        subtitle_lbl = tk.Label(
            self.sidebar, text="Phase 1: Simulation & GUI",
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
            # Hover animations
            btn.bind("<Enter>", lambda e: btn.configure(bg=active_bg))
            btn.bind("<Leave>", lambda e: btn.configure(bg=bg))
            btn.pack(fill="x", pady=5)
            return btn

        # Buttons
        create_btn("สร้างเขาวงกตใหม่", "#2563eb", "#1d4ed8", self.generate_new_maze)
        create_btn("วัดเวลา A* (5 ครั้ง)", "#059669", "#047857", self.measure_computation_time)
        create_btn("แสดง/ซ่อน เส้นทาง A*", "#d97706", "#b45309", self.toggle_path)

        # Output Log Box
        log_lbl = tk.Label(
            self.sidebar, text="ประวัติการประมวลผล:",
            bg="#1e293b", fg="#cbd5e1", font=("Helvetica", 10, "bold")
        )
        log_lbl.pack(anchor="w", pady=(15, 5))

        # Text area with scrollbar
        self.log_text = tk.Text(
            self.sidebar, bg="#0f172a", fg="#10b981", # Green terminal text
            insertbackground="white", bd=0, relief="flat",
            font=("Consolas", 9), height=15
        )
        self.log_text.pack(fill="both", expand=True, pady=(0, 5))
        
        # Scrollbar for log text
        scrollbar = tk.Scrollbar(self.log_text, bg="#0f172a")
        scrollbar.pack(side="right", fill="y")
        self.log_text.config(yscrollcommand=scrollbar.set)
        scrollbar.config(command=self.log_text.yview)

        # Initial info
        self.log_output("ระบบเริ่มต้นสำเร็จ\n"
                        f"เขาวงกตขนาด {GRID_SIZE}x{GRID_SIZE} ช่อง (ช่องละ {CELL_SIZE_CM} cm)")

    def log_output(self, text):
        self.log_text.insert(tk.END, text + "\n")
        self.log_text.see(tk.END)

    def on_resize(self, event):
        # Calculate maximum possible size for GRID_SIZE cells
        # We leave a small margin (e.g. 10px on each side) to avoid clipping
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
        
        # Draw cells
        for r in range(GRID_SIZE):
            for c in range(GRID_SIZE):
                x1 = c * cs + ox
                y1 = r * cs + oy
                x2 = x1 + cs
                y2 = y1 + cs
                
                # All cells are walkable space by default
                color = "#ffffff" # White for paths
                outline_color = "#e2e8f0" # Light gray for cell grid lines
                
                # Custom colors for START and FINISH
                if (r, c) == START:
                    color = "#2ecc71" # Neon Green
                    outline_color = "#27ae60"
                elif (r, c) == FINISH:
                    color = "#e74c3c" # Tomato Red
                    outline_color = "#c0392b"
                    
                self.canvas.create_rectangle(
                    x1, y1, x2, y2,
                    fill=color, outline=outline_color, width=1
                )
                
                # Draw text identifier for Start/Finish
                if (r, c) == START:
                    self.canvas.create_text(
                        x1 + cs/2, y1 + cs/2,
                        text="S", fill="#ffffff",
                        font=("Helvetica", 10, "bold")
                    )
                elif (r, c) == FINISH:
                    self.canvas.create_text(
                        x1 + cs/2, y1 + cs/2,
                        text="F", fill="#ffffff",
                        font=("Helvetica", 10, "bold")
                    )

        # Draw walls as border lines between cells
        # Horizontal walls
        for r in range(GRID_SIZE + 1):
            for c in range(GRID_SIZE):
                if self.h_walls[r, c]:
                    self.canvas.create_line(
                        c * cs + ox, r * cs + oy, (c + 1) * cs + ox, r * cs + oy,
                        fill="#1e293b", width=3, capstyle="projecting"
                    )
        # Vertical walls
        for r in range(GRID_SIZE):
            for c in range(GRID_SIZE + 1):
                if self.v_walls[r, c]:
                    self.canvas.create_line(
                        c * cs + ox, r * cs + oy, c * cs + ox, (r + 1) * cs + oy,
                        fill="#1e293b", width=3, capstyle="projecting"
                    )

        # Draw Planned A* Path if toggle is on
        if self.show_path and self.planned_path:
            for i in range(len(self.planned_path) - 1):
                r1, c1 = self.planned_path[i]
                r2, c2 = self.planned_path[i+1]
                # Line coordinates (centers of cells)
                lx1 = c1 * cs + cs/2 + ox
                ly1 = r1 * cs + cs/2 + oy
                lx2 = c2 * cs + cs/2 + ox
                ly2 = r2 * cs + cs/2 + oy
                self.canvas.create_line(
                    lx1, ly1, lx2, ly2,
                    fill="#fd9644", width=3, capstyle="round" # Vibrant orange path
                )

        # Draw Mouse
        mr, mc = self.mouse.position
        mx = mc * cs + cs/2 + ox
        my = mr * cs + cs/2 + oy
        r_offset = cs * 0.35 # Mouse radius
        self.canvas.create_oval(
            mx - r_offset, my - r_offset,
            mx + r_offset, my + r_offset,
            fill="#3b82f6", outline="#1d4ed8", width=1.5 # Blue mouse
        )
        self.canvas.create_text(
            mx, my,
            text="M", fill="#ffffff",
            font=("Helvetica", 8, "bold")
        )


# =========================================================
# 7. Main - รันระบบ GUI
# =========================================================
if __name__ == "__main__":
    root = tk.Tk()
    root.resizable(True, True)
    app = MazeGUI(root)
    root.mainloop()
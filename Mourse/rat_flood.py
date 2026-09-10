"""
rat_flood.py - Micromouse Rat using Online Flood Fill Algorithm

กฎที่ปฏิบัติตาม:
  R8: ไม่รู้แผนที่ล่วงหน้า (เรียนรู้จากการวิ่งเท่านั้น)
  R9: ไม่มี LIDAR ต้องหันหน้าไปทิศนั้นก่อนถึงจะตรวจกำแพงได้
  R10: ตัดสินใจ 1 action ต่อรอบ (FORWARD / LEFT / RIGHT / BACKWARD)
  R7: รู้ระยะทางตรงทะลุกำแพงถึงชีสได้ตลอดเวลา

กลยุทธ์ Flood Fill:
  - สร้างตาราง flood[] เก็บระยะห่างจากชีสของแต่ละช่อง
  - ทุกครั้งที่ค้นพบกำแพงใหม่ ให้ re-flood ใหม่ทั้งหมด
  - เลือกเดินไปช่องที่มีค่า flood น้อยที่สุดเสมอ

กลยุทธ์ลดสเต็ป:
  - เมื่อเข้าช่องใหม่ หนูจะสำรวจรอบด้าน (ซ้าย, ขวา, หน้า)
    ก่อนตัดสินใจเดิน เพื่อให้ Flood Fill มีข้อมูลแม่นยำขึ้น
  - หลีกเลี่ยงช่องที่เคยสำรวจครบแล้ว ถ้ามีตัวเลือกที่ดีกว่า
"""

from collections import deque
import numpy as np

# Directions
NORTH = (-1, 0)
SOUTH = (1, 0)
WEST = (0, -1)
EAST = (0, 1)

DIR_ORDER = [NORTH, EAST, SOUTH, WEST]  # Clockwise

# Unknown / Free / Wall states for known_walls
UNKNOWN = -1
FREE_WALL = False
WALL = True

INF = 9999


class RatFlood:
    def __init__(self, start_pos, goal_pos, grid_size=30):
        self.position = start_pos
        self.goal_pos = goal_pos
        self.grid_size = grid_size

        # Start facing EAST
        self.facing = EAST

        # ---------- Known wall memory ----------
        # None = unknown, True = wall, False = free
        self.known_h_walls = np.full((grid_size + 1, grid_size), None, dtype=object)
        self.known_v_walls = np.full((grid_size, grid_size + 1), None, dtype=object)

        # Boundaries are always walls
        self.known_h_walls[0, :] = True
        self.known_h_walls[grid_size, :] = True
        self.known_v_walls[:, 0] = True
        self.known_v_walls[:, grid_size] = True

        # ---------- Flood Fill table ----------
        # Initialize with Manhattan distance from goal
        self.flood = np.full((grid_size, grid_size), INF, dtype=int)
        self._init_flood()

        # ---------- Exploration state ----------
        # Track how many sides of each cell we have sensed (max 4)
        self.sensed_sides = np.zeros((grid_size, grid_size), dtype=int)

        # Phase: "SENSE" = still turning to sense sides, "MOVE" = ready to move
        self._sense_queue = []      # List of target facings to sense before moving
        self._build_sense_queue()   # Sense all sides on start cell

        self.visited = set()
        self.visited.add(self.position)

    # ------------------------------------------------------------------ helpers
    def _init_flood(self):
        """BFS flood from goal using only known-free passages."""
        gr, gc = self.goal_pos
        self.flood[:, :] = INF
        self.flood[gr, gc] = 0
        q = deque()
        q.append((gr, gc))
        while q:
            r, c = q.popleft()
            for dr, dc in DIR_ORDER:
                nr, nc = r + dr, c + dc
                if 0 <= nr < self.grid_size and 0 <= nc < self.grid_size:
                    if self._is_open((r, c), (nr, nc)):
                        new_val = self.flood[r, c] + 1
                        if new_val < self.flood[nr, nc]:
                            self.flood[nr, nc] = new_val
                            q.append((nr, nc))

    def _is_open(self, a, b):
        """
        Check if there is NO wall between cell a and cell b
        using currently known walls. If unknown, treat as FREE (optimistic).
        """
        ar, ac = a
        br, bc = b
        dr, dc = br - ar, bc - ac
        if (dr, dc) == NORTH:     # a is below b
            val = self.known_h_walls[ar, ac]
        elif (dr, dc) == SOUTH:   # a is above b
            val = self.known_h_walls[ar + 1, ac]
        elif (dr, dc) == WEST:    # a is right of b
            val = self.known_v_walls[ar, ac]
        elif (dr, dc) == EAST:    # a is left of b
            val = self.known_v_walls[ar, ac + 1]
        else:
            return False
        # None = unknown = assume open (optimistic)
        return val is not True

    def _wall_exists(self, from_pos, facing):
        """Check known wall in facing direction from from_pos."""
        r, c = from_pos
        dr, dc = facing
        if facing == NORTH:
            val = self.known_h_walls[r, c]
        elif facing == SOUTH:
            val = self.known_h_walls[r + 1, c]
        elif facing == WEST:
            val = self.known_v_walls[r, c]
        elif facing == EAST:
            val = self.known_v_walls[r, c + 1]
        else:
            return True
        return val is True

    def _update_wall(self, from_pos, facing, has_wall: bool):
        """Record sensed wall into memory and re-flood if changed."""
        r, c = from_pos
        changed = False
        if facing == NORTH:
            if self.known_h_walls[r, c] is None:
                self.known_h_walls[r, c] = has_wall
                changed = True
        elif facing == SOUTH:
            if self.known_h_walls[r + 1, c] is None:
                self.known_h_walls[r + 1, c] = has_wall
                changed = True
        elif facing == WEST:
            if self.known_v_walls[r, c] is None:
                self.known_v_walls[r, c] = has_wall
                changed = True
        elif facing == EAST:
            if self.known_v_walls[r, c + 1] is None:
                self.known_v_walls[r, c + 1] = has_wall
                changed = True
        if changed:
            self._init_flood()      # Re-flood with new wall info

    def _build_sense_queue(self):
        """
        Build list of facings to turn to in order to sense
        the 3 remaining sides (left, front, right relative to current facing).
        We skip BACKWARD since we know we just came from there (no wall).
        """
        idx = DIR_ORDER.index(self.facing)
        # We will sense: LEFT, FRONT, RIGHT  (relative to current facing)
        left  = DIR_ORDER[(idx - 1) % 4]
        right = DIR_ORDER[(idx + 1) % 4]
        front = self.facing

        self._sense_queue = []
        for d in [left, front, right]:
            # Only queue if still unknown
            r, c = self.position
            if self._get_wall_known(d) is None:
                self._sense_queue.append(d)

    def _get_wall_known(self, facing):
        """Return True/False/None for the wall in facing direction."""
        r, c = self.position
        if facing == NORTH:
            return self.known_h_walls[r, c]
        elif facing == SOUTH:
            return self.known_h_walls[r + 1, c]
        elif facing == WEST:
            return self.known_v_walls[r, c]
        elif facing == EAST:
            return self.known_v_walls[r, c + 1]
        return None

    # -------------------------------------------------------- Public API
    def get_distance_to_cheese(self):
        """R7: Straight-line distance to cheese in CM (1 unit = 16 cm)."""
        r1, c1 = self.position
        r2, c2 = self.goal_pos
        return np.hypot(r1 - r2, c1 - c2) * 16.0

    def sense_and_update(self, wall_in_front: bool):
        """
        Called by the simulator each round BEFORE decide_next_action().
        Records the wall status directly in front of current facing.
        """
        self._update_wall(self.position, self.facing, wall_in_front)

    def decide_next_action(self):
        """
        Main decision function. Returns exactly one of:
        'FORWARD' / 'LEFT' / 'RIGHT' / 'BACKWARD'

        Strategy:
          Phase A - SENSE SWEEP: If there are unknown sides to sense on this
                    cell, turn toward them one by one (1 action per round).
          Phase B - MOVE: Pick the open neighbor with the lowest flood value
                    and move toward it.
        """
        if self.position == self.goal_pos:
            return None

        # ---- Phase A: Sense sweep ----
        # Pop next side to sense (we will face it; simulator will sense it)
        while self._sense_queue:
            target_facing = self._sense_queue[0]
            if self._get_wall_known(target_facing) is not None:
                # Already known (maybe sensed by previous sense_and_update)
                self._sense_queue.pop(0)
                continue
            # Need to turn toward target_facing and sense it
            action = self._turn_toward(target_facing)
            if action == "FORWARD" and self.facing == target_facing:
                # We're already facing it; the sense_and_update at top of round
                # already recorded it, so pop and continue
                self._sense_queue.pop(0)
                continue
            return action  # Turn to face that direction (sense happens next round)

        # ---- Phase B: Flood Fill move ----
        r, c = self.position
        best_val = INF + 1
        best_dir = None

        for dr, dc in DIR_ORDER:
            nr, nc = r + dr, c + dc
            if 0 <= nr < self.grid_size and 0 <= nc < self.grid_size:
                # Must not have a known wall
                if not self._wall_exists(self.position, (dr, dc)):
                    if self.flood[nr, nc] < best_val:
                        best_val = self.flood[nr, nc]
                        best_dir = (dr, dc)

        if best_dir is None:
            # Surrounded by walls - should not happen in valid maze
            return None

        # Turn or move toward best_dir
        action = self._turn_toward(best_dir)
        return action

    def _turn_toward(self, target_dir):
        """
        Return the single action needed to face target_dir,
        or 'FORWARD' if already facing it.
        """
        if self.facing == target_dir:
            return "FORWARD"
        curr_idx = DIR_ORDER.index(self.facing)
        tgt_idx = DIR_ORDER.index(target_dir)
        diff = (tgt_idx - curr_idx) % 4
        if diff == 1:
            return "RIGHT"
        elif diff == 3:
            return "LEFT"
        else:
            return "BACKWARD"

    def apply_action(self, action):
        """
        Apply one action. When moving FORWARD into a new cell,
        automatically queue a sense sweep for that cell.
        """
        prev_pos = self.position

        if action == "FORWARD":
            dr, dc = self.facing
            r, c = self.position
            nr, nc = r + dr, c + dc
            if 0 <= nr < self.grid_size and 0 <= nc < self.grid_size:
                self.position = (nr, nc)
                self.visited.add(self.position)
                # Mark passage we just came through as FREE
                self._update_wall(prev_pos, self.facing, False)
                # Queue sense sweep for the new cell
                self._build_sense_queue()
        elif action == "LEFT":
            idx = DIR_ORDER.index(self.facing)
            self.facing = DIR_ORDER[(idx - 1) % 4]
            # When we face a new direction, pop it from sense queue if it's next
            if self._sense_queue and self._sense_queue[0] == self.facing:
                self._sense_queue.pop(0)
        elif action == "RIGHT":
            idx = DIR_ORDER.index(self.facing)
            self.facing = DIR_ORDER[(idx + 1) % 4]
            if self._sense_queue and self._sense_queue[0] == self.facing:
                self._sense_queue.pop(0)
        elif action == "BACKWARD":
            idx = DIR_ORDER.index(self.facing)
            self.facing = DIR_ORDER[(idx + 2) % 4]
            if self._sense_queue and self._sense_queue[0] == self.facing:
                self._sense_queue.pop(0)

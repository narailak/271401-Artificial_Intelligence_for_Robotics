import numpy as np
import heapq
from collections import deque

# Directions
NORTH = (-1, 0)
SOUTH = (1, 0)
WEST = (0, -1)
EAST = (0, 1)

DIR_ORDER = [NORTH, EAST, SOUTH, WEST] # Clockwise order

class Rat:
    # How many most-recent steps to remember and penalize
    HISTORY_LEN   = 20     # จำ 20 ช่องล่าสุด
    PENALTY_BASE  = 8      # ค่า penalty สูงสุดสำหรับช่องที่เพิ่งเดิน
    
    def __init__(self, start_pos, goal_pos, grid_size=30):
        self.position = start_pos
        self.goal_pos = goal_pos
        self.grid_size = grid_size
        
        # Facing direction (starts facing EAST by default)
        self.facing = EAST
        
        # Internal memory of walls: True = Wall, False = Free/Unknown
        # We start by initializing all walls to False (assuming no walls)
        # except the outer boundaries of the maze which we know are walls.
        self.known_h_walls = np.zeros((grid_size + 1, grid_size), dtype=bool)
        self.known_v_walls = np.zeros((grid_size, grid_size + 1), dtype=bool)
        
        # Initialize boundaries as walls
        self.known_h_walls[0, :] = True
        self.known_h_walls[grid_size, :] = True
        self.known_v_walls[:, 0] = True
        self.known_v_walls[:, grid_size] = True

        # History of recent positions (newest = right end)
        # Used to add recency penalty in A* and discourage backtracking
        self.visit_history: deque = deque(maxlen=self.HISTORY_LEN)
        self.visit_history.append(start_pos)

    def _recency_penalty(self, cell) -> float:
        """
        Returns extra cost for visiting a recently-visited cell.
        The more recent the visit, the higher the penalty.
        A cell visited 1 step ago gets PENALTY_BASE penalty.
        A cell visited HISTORY_LEN steps ago gets 0 penalty.
        """
        history = list(self.visit_history)
        for i, past in enumerate(reversed(history)):
            if past == cell:
                # i=0  means just visited → full penalty
                # i=HISTORY_LEN-1 means oldest → near-zero penalty
                ratio = 1.0 - (i / self.HISTORY_LEN)
                return self.PENALTY_BASE * ratio
        return 0.0

    def get_distance_to_cheese(self):
        """R7: straight line distance to cheese in CM (1 unit = 16 cm)"""
        r1, c1 = self.position
        r2, c2 = self.goal_pos
        euclidean_dist = np.hypot(r1 - r2, c1 - c2)
        return euclidean_dist * 16.0

    def sense_and_update(self, wall_in_front: bool):
        """Update internal wall memory based on the sensor reading directly in front"""
        r, c = self.position
        dr, dc = self.facing
        
        if self.facing == NORTH:
            self.known_h_walls[r, c] = wall_in_front
        elif self.facing == SOUTH:
            self.known_h_walls[r + 1, c] = wall_in_front
        elif self.facing == WEST:
            self.known_v_walls[r, c] = wall_in_front
        elif self.facing == EAST:
            self.known_v_walls[r, c + 1] = wall_in_front

    def plan_path(self):
        """
        Run A* with recency penalty to find the best path.
        Recently-visited cells cost more, so the rat avoids
        immediately backtracking through the same corridor.
        """
        start = self.position
        goal = self.goal_pos
        
        def manhattan_distance(a, b):
            return abs(a[0] - b[0]) + abs(a[1] - b[1])
            
        def get_neighbors(pos):
            r, c = pos
            result = []
            # Up
            if r > 0 and not self.known_h_walls[r, c]:
                result.append((r - 1, c))
            # Down
            if r < self.grid_size - 1 and not self.known_h_walls[r + 1, c]:
                result.append((r + 1, c))
            # Left
            if c > 0 and not self.known_v_walls[r, c]:
                result.append((r, c - 1))
            # Right
            if c < self.grid_size - 1 and not self.known_v_walls[r, c + 1]:
                result.append((r, c + 1))
            return result

        open_set = []
        heapq.heappush(open_set, (0 + manhattan_distance(start, goal), 0.0, start))
        came_from = {}
        g_score = {start: 0.0}
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

            for neighbor in get_neighbors(current):
                # Base cost = 1 step + recency penalty for entering that neighbor
                step_cost = 1.0 + self._recency_penalty(neighbor)
                tentative_g = current_g + step_cost
                if tentative_g < g_score.get(neighbor, float("inf")):
                    g_score[neighbor] = tentative_g
                    came_from[neighbor] = current
                    f_score = tentative_g + manhattan_distance(neighbor, goal)
                    heapq.heappush(open_set, (f_score, tentative_g, neighbor))

        return None # No path found (should not happen in a valid solvable maze)

    def decide_next_action(self):
        """
        Determines the next action: "FORWARD", "LEFT", "RIGHT", or "BACKWARD"
        based on the A* path (with recency penalty) using current map memory.
        """
        if self.position == self.goal_pos:
            return None # Already at goal
            
        path = self.plan_path()
        if not path or len(path) < 2:
            return None
            
        # The next cell we want to go to
        next_cell = path[1]
        req_dir = (next_cell[0] - self.position[0], next_cell[1] - self.position[1])
        
        # Calculate turn action needed to match req_dir
        if self.facing == req_dir:
            return "FORWARD"
            
        # Determine the relative direction
        curr_idx = DIR_ORDER.index(self.facing)
        req_idx = DIR_ORDER.index(req_dir)
        diff = (req_idx - curr_idx) % 4
        
        if diff == 1:
            return "RIGHT"
        elif diff == 3:
            return "LEFT"
        else:
            return "BACKWARD"

    def apply_action(self, action):
        """Applies the decided action to update position or facing direction"""
        if action == "FORWARD":
            r, c = self.position
            dr, dc = self.facing
            self.position = (r + dr, c + dc)
            # Record new position in visit history
            self.visit_history.append(self.position)
        elif action == "LEFT":
            curr_idx = DIR_ORDER.index(self.facing)
            self.facing = DIR_ORDER[(curr_idx - 1) % 4]
        elif action == "RIGHT":
            curr_idx = DIR_ORDER.index(self.facing)
            self.facing = DIR_ORDER[(curr_idx + 1) % 4]
        elif action == "BACKWARD":
            curr_idx = DIR_ORDER.index(self.facing)
            self.facing = DIR_ORDER[(curr_idx + 2) % 4]


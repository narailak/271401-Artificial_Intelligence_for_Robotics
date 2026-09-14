import numpy as np

# กำหนดขนาดตาราง
ROWS, COLS = 5, 5

# กำหนดตำแหน่งพิเศษ
START = (0, 0)
GOAL = (0, 4)
OBSTACLES = {(0, 3), (1, 1), (1, 3), (3, 0), (3, 1), (3, 3), (3, 4)}

# Actions ที่ทำได้
ACTIONS = {
    'UP': (-1, 0),
    'DOWN': (1, 0),
    'LEFT': (0, -1),
    'RIGHT': (0, 1)
}

#  Policy Map
ARROW_MAP = {
    'UP': ' ↑ ',
    'DOWN': ' ↓ ',
    'LEFT': ' ← ',
    'RIGHT': ' → '
}

# พารามิเตอร์ของ MDP
GAMMA = 0.9
THETA = 1e-4

def get_transitions(s, a):
    """
    Deterministic Transition Model:
    การเลือก Action 'a' จะนำไปสู่ State ปลายทางด้วยความน่าจะเป็น P(s' | s, a) = 1.0 เสมอ
    หากชนขอบตารางหรือชนสิ่งกีดขวาง จะอยู่ที่ตำแหน่งเดิม (s' = s) ด้วยความน่าจะเป็น 1.0 เช่นกัน
    Return: list ของ tuple [(s_prime, prob)]
    """
    dr, dc = ACTIONS[a]
    nr, nc = s[0] + dr, s[1] + dc
    
    # ตรวจสอบการชนขอบ หรือการชนกำแพง
    if nr < 0 or nr >= ROWS or nc < 0 or nc >= COLS or (nr, nc) in OBSTACLES:
        s_prime = s
    else:
        s_prime = (nr, nc)
        
    return [(s_prime, 1.0)]

def value_iteration():
    """
    Value Iteration ตาม Bellman Optimality Equation
    V*(s) = max_a sum_{s'} P(s'|s,a) * [R(s,a,s') + gamma * V*(s')]
    """
    V = np.zeros((ROWS, COLS))
    
    # 1. วนรอบปรับปรุงค่า Value Function
    while True:
        delta = 0
        new_V = np.copy(V)
        
        for r in range(ROWS):
            for c in range(COLS):
                s = (r, c)
                if s == GOAL or s in OBSTACLES:
                    continue
                
                action_values = []
                for a in ACTIONS:
                    q_val = 0.0
                    for s_prime, prob in get_transitions(s, a):
                        reward = 10.0 if s_prime == GOAL else -1.0
                        q_val += prob * (reward + GAMMA * V[s_prime])
                    action_values.append(q_val)
                
                new_V[s] = max(action_values)
                delta = max(delta, abs(new_V[s] - V[s]))
                
        V = new_V
        if delta < THETA:
            break
            
    # 2. ทำ Policy Extraction เพื่อหา Actionที่ดีที่สุดสำหรับแต่ละ State
    policy = {}
    for r in range(ROWS):
        for c in range(COLS):
            s = (r, c)
            if s == GOAL or s in OBSTACLES:
                continue
            
            best_action = None
            best_q = -float('inf')
            for a in ACTIONS:
                q_val = 0.0
                for s_prime, prob in get_transitions(s, a):
                    reward = 10.0 if s_prime == GOAL else -1.0
                    q_val += prob * (reward + GAMMA * V[s_prime])
                if q_val > best_q:
                    best_q = q_val
                    best_action = a
            policy[s] = best_action
            
    return V, policy

def trace_path(policy, max_steps=100):
    #เดินตาม Optimal Policy จาก START ไป GOAL

    curr = START
    path = [curr]
    visited = {curr}
    
    for _ in range(max_steps):
        if curr == GOAL:
            return path, True
            
        action = policy.get(curr)
        if not action:
            break
            
        # การเปลี่ยนสถานะแบบ Deterministic (prob = 1.0)
        curr = get_transitions(curr, action)[0][0]
        
        if curr in visited:
            print("[Warning] ตรวจพบลูป: กลับมาที่สถานะเดิมที่เคยเดินผ่านแล้ว")
            path.append(curr)
            return path, False
            
        visited.add(curr)
        path.append(curr)
        
    return path, False

def print_policy_grid(policy):
    print("\n" + "="*35)
    print("      OPTIMAL POLICY MAP (π*)")
    print("="*35)
    for r in range(ROWS):
        row_str = []
        for c in range(COLS):
            s = (r, c)
            if s == START:
                row_str.append(" S ")
            elif s == GOAL:
                row_str.append(" G ")
            elif s in OBSTACLES:
                row_str.append(" ■ ")
            else:
                row_str.append(ARROW_MAP.get(policy.get(s), " . "))
        print(" | ".join(row_str))
        if r < ROWS - 1:
            print("-" * 35)

# --- รันโปรแกรม ---
V, policy = value_iteration()
path, success = trace_path(policy)

# print_Action ของทุก State
print_policy_grid(policy)

print("\n" + "="*35)
print("สรุป Optimal Pathway:")
print("="*35)
path_str = " -> ".join([f"{p} ({policy[p]})" if p in policy else f"{p} (GOAL)" for p in path])
print(path_str)
print(f"สถานะ: {'ถึงเป้าหมายสำเร็จ' if success else 'ไม่ถึงเป้าหมาย'} (จำนวน {len(path)-1} ก้าว)")
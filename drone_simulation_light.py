"""
Drone-assisted Firefighter Simulation
Save as: drone_fire_sim.py
Requires: numpy, matplotlib

Run: python drone_fire_sim.py

Controls:
 - Click START to run the simulation (click again to pause).
 - Simulation shows searching Drone 1, guide Drone 2, rescuers (R), victims (V),
   fire (red), smoke (gray), walls (black), entrance (green).
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Button
import heapq
import random
import math
import time


GRID = 30
CELL = 1
FPS = 8  
FIRE_SPREAD_PROB = 0.08  
SMOKE_DECAY = 0.02
INITIAL_FIRES = 3
INITIAL_VICTIMS = 3
NUM_RESCUERS = 2

FREE = 0
WALL = 1
FIRE = 2
SMOKE = 3
VICTIM = 4
DRONE1 = 5
DRONE2 = 6
RESCUER = 7
ENTRANCE = 8
EXIT = 9
EXPLORED = 10


COLORS = {
    FREE: '#ffffff',
    WALL: '#333333',
    FIRE: '#e23b3b',
    SMOKE: '#bdbdbd',
    VICTIM: '#ffd966',
    DRONE1: '#2b7bff',
    DRONE2: '#a64dff',
    RESCUER: '#00a86b',
    ENTRANCE: '#0f9d58',
    EXIT: '#f4b400',
    EXPLORED: '#f0f8ff',
}

def heuristic(a, b):
    return abs(a[0]-b[0]) + abs(a[1]-b[1])

def neighbors(node, grid):
    x, y = node
    for dx, dy in ((1,0),(-1,0),(0,1),(0,-1)):
        nx, ny = x+dx, y+dy
        if 0 <= nx < GRID and 0 <= ny < GRID:
            if grid[nx, ny] != WALL and grid[nx, ny] != FIRE:
                yield (nx, ny)

def astar(start, goal, grid):
    if start == goal:
        return [start]
    frontier = []
    heapq.heappush(frontier, (0+heuristic(start, goal), 0, start, None))
    came_from = {}
    cost_so_far = {start: 0}
    while frontier:
        _, cost, current, parent = heapq.heappop(frontier)
        if current in came_from:
            continue
        came_from[current] = parent
        if current == goal:
             

            path = []
            node = current
            while node is not None:
                path.append(node)
                node = came_from[node]
            path.reverse()
            return path
        for nxt in neighbors(current, grid):
            new_cost = cost_so_far[current] + 1
            if nxt not in cost_so_far or new_cost < cost_so_far[nxt]:
                cost_so_far[nxt] = new_cost
                priority = new_cost + heuristic(nxt, goal)
                heapq.heappush(frontier, (priority, new_cost, nxt, current))
    return None




np.random.seed(2)

grid = np.zeros((GRID, GRID), dtype=int)
smoke = np.zeros_like(grid, dtype=float)
explored = np.zeros_like(grid, dtype=bool)


def add_room(x, y, w, h):
    grid[x:x+w, y:y+h] = WALL
    

    if w > 2 and h > 2:
        px = random.randint(x+1, x+w-2)
        py = random.randint(y+1, y+h-2)
        grid[px, py] = FREE


for _ in range(5):
    w, h = random.randint(4,8), random.randint(4,8)
    x, y = random.randint(1, GRID-w-1), random.randint(1, GRID-h-1)
    add_room(x, y, w, h)


entrance = (GRID-1, random.randint(1, GRID-2))
exit_cell = (0, random.randint(1, GRID-2))
grid[entrance] = FREE
grid[exit_cell] = FREE



free_cells = list(zip(*np.where(grid == FREE)))
for _ in range(INITIAL_FIRES):
    c = random.choice(free_cells)
    grid[c] = FIRE


victim_positions = []
for _ in range(INITIAL_VICTIMS):
    choices = [c for c in free_cells if grid[c]==FREE and heuristic(c, entrance) > 4]
    if not choices:
        break
    v = random.choice(choices)
    grid[v] = VICTIM
    victim_positions.append(v)



grid[entrance] = ENTRANCE
grid[exit_cell] = EXIT


drone1_pos = entrance 
drone2_pos = entrance 
drone1_path = []
drone2_path = []
drone1_target = None
drone2_target = None
drone1_search_frontiers = [] 

rescuer_positions = [entrance for _ in range(NUM_RESCUERS)]
rescuer_paths = [[] for _ in range(NUM_RESCUERS)]



victim_found = False
found_victim_pos = None
simulation_running = False
paused = False
tick = 0



def recompute_frontiers():
    frontiers = []
    for x in range(GRID):
        for y in range(GRID):
            if grid[x,y] in (FREE, EXPLORED, ENTRANCE) and not explored[x,y]:
                
                
                adj_explored = False
                for nx, ny in ((x+1,y),(x-1,y),(x,y+1),(x,y-1)):
                    if 0 <= nx < GRID and 0 <= ny < GRID:
                        if explored[nx, ny] or grid[nx, ny] == ENTRANCE:
                            adj_explored = True
                if adj_explored:
                    frontiers.append((x,y))
    return frontiers


def pick_search_target(drone_pos):
    frontiers = recompute_frontiers()
    if not frontiers:
        
        frontiers = [(x,y) for x in range(GRID) for y in range(GRID)
                     if grid[x,y] in (FREE, EXPLORED) and not explored[x,y]]
    if not frontiers:
        return None
    

    frontiers.sort(key=lambda c: heuristic(c, drone_pos))
    for f in frontiers:
        path = astar(drone_pos, f, grid)
        if path:
            return f, path
    return None


def victim_step(pos):
    x, y = pos
    

    if grid[pos] == FIRE:
        return pos
    

    fire_near = False
    total_fire_dir = np.array([0.0, 0.0])
    for dx, dy in ((1,0),(-1,0),(0,1),(0,-1)):
        nx, ny = x+dx, y+dy
        if 0 <= nx < GRID and 0 <= ny < GRID:
            if grid[nx, ny] == FIRE:
                fire_near = True
                total_fire_dir += np.array([dx,dy])
    if fire_near:
        

        best = pos
        best_score = -1e9
        for dx, dy in ((1,0),(-1,0),(0,1),(0,-1),(0,0)):
            nx, ny = x+dx, y+dy
            if 0 <= nx < GRID and 0 <= ny < GRID and grid[nx, ny] in (FREE, EXPLORED, ENTRANCE):
               

                score = - (dx*total_fire_dir[0] + dy*total_fire_dir[1])
               

                score -= sum(1 for ddx,ddy in ((1,0),(-1,0),(0,1),(0,-1))
                             if 0 <= nx+ddx < GRID and 0 <= ny+ddy < GRID and grid[nx+ddx, ny+ddy]==FIRE)
                if score > best_score:
                    best_score = score
                    best = (nx, ny)
        return best
    

    if random.random() < 0.05:
        

        options = []
        for dx, dy in ((1,0),(-1,0),(0,1),(0,-1)):
            nx, ny = x+dx, y+dy
            if 0 <= nx < GRID and 0 <= ny < GRID and grid[nx, ny] in (FREE, EXPLORED, ENTRANCE):
                options.append((nx, ny))
        if options:
            return random.choice(options)
    return pos



fig, ax = plt.subplots(figsize=(7,7))
plt.subplots_adjust(bottom=0.12)
ax.set_xticks([])
ax.set_yticks([])
ax.set_title("Drone-assisted Firefighter Simulation")
im = ax.imshow(np.zeros((GRID,GRID,3)), interpolation='nearest', vmin=0, vmax=1)


text_tick = ax.text(0.01, 1.02, "", transform=ax.transAxes, fontsize=10)
text_status = ax.text(0.5, 1.02, "", transform=ax.transAxes, fontsize=10, ha='center')


ax_button = plt.axes([0.4, 0.03, 0.2, 0.06])
btn = Button(ax_button, 'START', hovercolor='#a6a6a6')

def toggle_run(event):
    global simulation_running, paused
    if not simulation_running:
        simulation_running = True
        btn.label.set_text('PAUSE')
    else:
        paused = not paused
        btn.label.set_text('PAUSE' if not paused else 'RESUME')

btn.on_clicked(toggle_run)


def grid_to_rgb(grid, smoke):
    arr = np.ones((GRID, GRID, 3))
    for x in range(GRID):
        for y in range(GRID):
            c = grid[x, y]
            color = COLORS.get(c, '#ffffff')
            

            try:
                hexv = color.lstrip('#')
                r = int(hexv[0:2],16)/255.0
                g = int(hexv[2:4],16)/255.0
                b = int(hexv[4:6],16)/255.0
                base = np.array([r,g,b])
            except:
                base = np.array([1.0,1.0,1.0])
            arr[x,y] = base
            
            if (grid[x,y] in (FREE, EXPLORED)) and explored[x,y]:
                arr[x,y] = arr[x,y]*0.9 + np.array([0.95,0.97,1.0])*0.1
        

            s = min(1.0, smoke[x,y])
            if s > 0:
                arr[x,y] = arr[x,y]*(1-s) + np.array([0.6,0.6,0.6])*s
    
    return arr

victim_visible_toggle = True
victim_blink_counter = 0

def simulation_step(frame):
    global tick, grid, smoke, drone1_pos, drone2_pos, drone1_path, drone2_path
    global drone1_target, drone2_target, victim_found, found_victim_pos, explored
    global victim_positions, rescuer_positions, rescuer_paths, victim_visible_toggle, victim_blink_counter

    if not simulation_running or paused:
     
        draw()
        return

    tick += 1

    
    new_fire = []
    for x in range(GRID):
        for y in range(GRID):
            if grid[x,y] == FIRE:
    

                for dx, dy in ((1,0),(-1,0),(0,1),(0,-1)):
                    nx, ny = x+dx, y+dy
                    if 0 <= nx < GRID and 0 <= ny < GRID:
                        if grid[nx, ny] in (FREE, EXPLORED, VICTIM, ENTRANCE, EXIT) and random.random() < FIRE_SPREAD_PROB:
                            new_fire.append((nx, ny))
    

                        if grid[nx, ny] != WALL:
                            smoke[nx, ny] = min(1.0, smoke[nx, ny] + 0.4)
    for c in new_fire:
        if grid[c] != FIRE:
            grid[c] = FIRE

    
    smoke *= (1 - SMOKE_DECAY)
    
    s2 = smoke.copy()
    for x in range(1, GRID-1):
        for y in range(1, GRID-1):
            s2[x,y] = (smoke[x,y] + smoke[x+1,y] + smoke[x-1,y] + smoke[x,y+1] + smoke[x,y-1]) / 5.0
    smoke = s2

    
    updated_victims = []
    for v in victim_positions:
    
        if grid[v] == FIRE:
    
            continue
        newv = victim_step(v)
        if newv != v:
    
            if grid[newv] in (FREE, EXPLORED, ENTRANCE):
    
                if grid[v] == VICTIM:
                    grid[v] = FREE
    
                if grid[newv] in (FREE, EXPLORED):
                    grid[newv] = VICTIM
                updated_victims.append(newv)
            else:
                updated_victims.append(v)
        else:
            updated_victims.append(v)
    victim_positions = updated_victims

    
    x1,y1 = drone1_pos
    for nx, ny in ((x1,y1),(x1+1,y1),(x1-1,y1),(x1,y1+1),(x1,y1-1)):
        if 0 <= nx < GRID and 0 <= ny < GRID and grid[nx,ny] in (FREE, EXPLORED):
            explored[nx,ny] = True
            if grid[nx,ny] == FREE:
                grid[nx,ny] = EXPLORED

    
    if not victim_found:
    

        if not drone1_path:
            pick = pick_search_target(drone1_pos)
            if pick:
                target, path = pick
                drone1_target = target
    
    
                drone1_path = astar(drone1_pos, target, grid) or []
        if drone1_path:
            
            if len(drone1_path) > 1:
                drone1_pos = drone1_path[1]
                drone1_path = drone1_path[1:]
            else:
                drone1_path = []
        
        for v in victim_positions:
            if heuristic(drone1_pos, v) <= 1:
                victim_found = True
                found_victim_pos = v
                print(f"[{time.strftime('%X')}] Drone1 found victim at {found_victim_pos}")
                
                drone2_path = astar(drone2_pos, found_victim_pos, grid) or []
                
                path_rescue = astar(entrance, found_victim_pos, grid)
                if path_rescue:
                    for i in range(NUM_RESCUERS):
                        rescuer_paths[i] = path_rescue.copy()
                break

    
    if victim_found and found_victim_pos:
        
        if drone2_path and not drone2_path:  
            pass
        if drone2_path:
            if len(drone2_path) > 1:
                drone2_pos = drone2_path[1]
                drone2_path = drone2_path[1:]
            else:
                drone2_path = []
        else:
            
            if drone2_pos != found_victim_pos:
                
                p = astar(drone2_pos, found_victim_pos, grid)
                if p:
                    drone2_path = p
            else:
                
                p = astar(found_victim_pos, entrance, grid)
                if p:
                    for cell in p:
                        if grid[cell] == FREE:
                            grid[cell] = EXPLORED
                    
                    for i in range(NUM_RESCUERS):
                        rescuer_paths[i] = p.copy()

    
    for i in range(NUM_RESCUERS):
        path = rescuer_paths[i]
        if path:
            
            if len(path) > 1:
                nxt = path[1]
                if grid[nxt] != FIRE:
                    rescuer_positions[i] = nxt
                    rescuer_paths[i] = path[1:]
                else:
                    
                    if victim_found:
                        p = astar(rescuer_positions[i], found_victim_pos, grid)
                        if p:
                            rescuer_paths[i] = p
            else:
                
                rescuer_paths[i] = []

    
    for i in range(NUM_RESCUERS):
        if found_victim_pos and rescuer_positions[i] == found_victim_pos:
            print(f"[{time.strftime('%X')}] Rescuer {i} reached victim {found_victim_pos} -> RESCUED")
            
            if found_victim_pos in victim_positions:
                victim_positions.remove(found_victim_pos)
            grid[found_victim_pos] = EXPLORED
            found_victim_pos = None
            victim_found = False
            
            drone2_path = astar(drone2_pos, entrance, grid) or []
            
            for j in range(NUM_RESCUERS):
                rescuer_paths[j] = []

    
    victim_blink_counter += 1
    if victim_blink_counter % (FPS//2 + 1) == 0:
        victim_visible_toggle = not victim_visible_toggle

    
    if grid[drone1_pos] == FIRE:
        drone1_pos = entrance
        drone1_path = []
    if grid[drone2_pos] == FIRE:
        drone2_pos = entrance
        drone2_path = []

    for i in range(NUM_RESCUERS):
        if grid[rescuer_positions[i]] == FIRE:
            rescuer_positions[i] = entrance
            rescuer_paths[i] = []

    draw()

def draw():
    
    disp = grid.copy()
    
    rgb = grid_to_rgb(disp, smoke)
    
    ax.clear()
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_title("Drone-assisted Firefighter Simulation")
    ax.imshow(rgb, interpolation='nearest')
    
    for v in victim_positions:
        x,y = v
        
        if found_victim_pos == v:
            if victim_visible_toggle:
                ax.text(y, x, "V", va='center', ha='center', fontsize=10, fontweight='bold')
        else:
            ax.text(y, x, "V", va='center', ha='center', fontsize=10)
    
    x1,y1 = drone1_pos
    ax.text(y1, x1, "D1", va='center', ha='center', fontsize=9, fontweight='bold', color='white')
    x2,y2 = drone2_pos
    ax.text(y2, x2, "D2", va='center', ha='center', fontsize=9, fontweight='bold', color='white')
    
    for i, r in enumerate(rescuer_positions):
        x,y = r
        ax.text(y, x, "R", va='center', ha='center', fontsize=8, fontweight='bold', color='black')
    
    status = f"Tick: {tick}  | Victim found: {victim_found}  | Victims remaining: {len(victim_positions)}"
    ax.text(0.02, 1.02, status, transform=ax.transAxes, fontsize=9, va='bottom', ha='left')
    
    ax.text(0.02, -0.03, "Legend: D1=Searcher  D2=Guide  R=Rescuer  V=Victim", transform=ax.transAxes, fontsize=8, va='top')


ani = animation.FuncAnimation(fig, simulation_step, interval=1000//FPS)


draw()

plt.show()



from controller import Supervisor
import math
from Path_Planning_Aastrix_Code import astar
import itertools
from itertools import permutations

robot = Supervisor()

#___________Drawing
def draw_path1(path, col):
    root = robot.getRoot()
    children = root.getField("children")



    for r, c in path:

        
        SCALE = 1
        wx = SCALE*((c * t) - (w / 2) + (t / 2))
        wy = SCALE*((r * t) - (h / 2) + (t / 2))


        marker = f"""
        Transform {{
          translation {wx} {wy} 0.01
          children [
            Shape {{
              appearance Appearance {{
                material Material {{
                  diffuseColor 1 {col} 0.5
                  emissiveColor 1 {col} 0.5
                }}
              }}
              geometry Sphere {{
                radius 0.03
              }}
            }}
          ]
        }}
        """

        children.importMFNodeFromString(-1, marker)
        
        
def draw_bonuses(bonus_dict):
    root = robot.getRoot()
    children = root.getField("children")

    for (r, c), value in bonus_dict.items():

        wx = (c * t) - (w / 2) + (t / 2)
        wy =(r * t) - (h / 2) + (t / 2)

       
        radius = 0.001 + 0.003 * value

        marker = f"""
        Transform {{
          translation {wx} {wy} 0.001
          children [
            Shape {{
              appearance Appearance {{
                material Material {{
                  diffuseColor 0 0.5 1
                  emissiveColor 0 0.5 1
                  shininess 1
                }}
              }}
              geometry Sphere {{
                radius {radius}
              }}
            }}
          ]
        }}
        """

        children.importMFNodeFromString(-1, marker)

def draw_start_goal(start, goal):
    root = robot.getRoot()
    children = root.getField("children")

    sx = (start[1] * t) - (w / 2) + (t / 2)
    sy =(start[0] * t) - (h / 2) + (t / 2)

    gx = (goal[1] * t) - (w / 2) + (t / 2)
    gy =(goal[0] * t) - (h / 2) + (t / 2)

    
    # start - green
    start_node = f"""
    Transform {{
      translation {sx} {sy} 0.0005
      children [
        Shape {{
          appearance Appearance {{
            material Material {{
              diffuseColor 0 1 0
              emissiveColor 0 1 0
            }}
          }}
          geometry Box {{
            size {t*0.9} {t*0.9} 0.001
          }}
        }}
      ]
    }}
    """

    #goal - red
    goal_node = f"""
    Transform {{
      translation {gx} {gy} 0.0005
      children [
        Shape {{
          appearance Appearance {{
            material Material {{
              diffuseColor 1 0 0
              emissiveColor 1 0 0
            }}
          }}
          geometry Box {{
            size {t*0.9} {t*0.9} 0.001
          }}
        }}
      ]
    }}
    """

    children.importMFNodeFromString(-1, start_node)
    children.importMFNodeFromString(-1, goal_node)


def get_dir(r1, c1, r2, c2):
    if r2 == r1 + 1: return (1, 0)  
    if r2 == r1 - 1: return (-1, 0) 
    if c2 == c1 + 1: return (0, -1) 
    if c2 == c1 - 1: return (0, 1)  


def draw_path(path, col1=1, col2=1, col3=0):

    root = robot.getRoot()
    children = root.getField("children")

    for i in range(len(path)-1):

        r1, c1 = path[i]
        r2, c2 = path[i+1]

        y1 = (r1 * t) - (w/2) + (t/2)
        x1 = (c1 * t) - (h/2) + (t/2)

        y2 = (r2 * t) - (w/2) + (t/2)
        x2 = (c2 * t) - (h/2) + (t/2)

        mx = (x1 + x2) / 2
        my = (y1 + y2) / 2

      
        if r1 == r2:          
            sx = t
            sy = 0.05

        else:               
            sx = 0.05
            sy = t

        segment = f"""
        Transform {{
          translation {mx} {my} 0.0005
          children [
            Shape {{
              appearance Appearance {{
                material Material {{
                  diffuseColor {col1} {col2} {col3}
                  emissiveColor 0 0 0
                }}
              }}
              geometry Box {{
                size {sx} {sy} 0.02
              }}
            }}
          ]
        }}
        """

        children.importMFNodeFromString(-1, segment)

      
        if i < len(path)-2:

            r3, c3 = path[i+2]

            d1 = (r2-r1, c2-c1)
            d2 = (r3-r2, c3-c2)

            if d1 != d2:

                corner = f"""
                Transform {{
                  translation {x2} {y2} 0.0005
                  children [
                    Shape {{
                      appearance Appearance {{
                        material Material {{
                          diffuseColor {col1} {col2} {col3}
                          emissiveColor {col1} {col2} {col3}
                        }}
                      }}
                      geometry Box {{
                        size 0.05 0.05 0.021
                      }}
                    }}
                  ]
                }}
                """

                children.importMFNodeFromString(-1, corner)





#__ constadnts adn enabling motors and sensors

timestep = int(robot.getBasicTimeStep())
l_wheel = robot.getDevice('left wheel motor')
r_wheel = robot.getDevice('right wheel motor')
l_wheel.setPosition(float('inf'))
r_wheel.setPosition(float('inf'))

l_wheel.setVelocity(1)
r_wheel.setVelocity(1)

l_sensor = robot.getDevice('left wheel sensor')
r_sensor = robot.getDevice('right wheel sensor')
l_sensor.enable(timestep)
r_sensor.enable(timestep)

lidar = robot.getDevice('LDS-01')
#gyro = robot.getDevice('gyro')
compass = robot.getDevice('compass')
lidar.enable(timestep)
#gyro.enable(timestep)
compass.enable(timestep)

robot.step(timestep)
prev_left = l_sensor.getValue()
prev_right = r_sensor.getValue()

keyboard = robot.getKeyboard()
keyboard.enable(timestep)

total_points = 0 

w = 1.75
h = 1.75
t = 0.35

x = 0
y = 0
dir = 1 
theta = 0
# north = 0
# east = 1
# south = 2
# west = 3

cener = 0.0075

xtarget = 0
ytarget = 0
dtarget = 0
target =()

rtarget = 0
ctarget = 0
time_done = 0

state  = "MAPPING"
motion = 0
stack = []
path_index = 0


start = None
goal = None
bonus = {}
waypoints = []
time = 0
start_time = 0

start_path_generated = False
goal_path_generated = False

stime = 0
etime = 0


greedy_start = 0
greedy_end = 0
time_left = 0

grid_change = False

bonus_collected = 0

grad = 1
grad_dif = 0
"""

1 = forward
2 = left
3 = right
4 = backwards

5 = mapping
6 = going to start position
7 = plannign for goal
8 = going to goal
9 = end

"""




#______________LOcalisation

def at_cell_center():
    global x, y, xtarget, ytarget, cener
    return abs(x - xtarget) < cener and abs(y - ytarget) < cener




def map(w, h, t_size):
    grid = []

    w_num = int(w / t_size)
    h_num = int(h / t_size)

    for y in range(h_num):
        row = []
        for x in range(w_num):
            row.append([None, None, None, None, False])  # [top, right, bottom, left]
        grid.append(row)
    return grid
    

def world_to_grid(x, y, t, w, h):
    
   
    r = int((y + (h/2))/ t)
    c = int((x+(w/2))/t)
    
    return r, c


def odometry():
    global x, y, prev_left, prev_right, dir, theta
    
    c_val = compass.getValues()
    theta = math.atan2(c_val[1], c_val[0])
    left = l_sensor.getValue()
    right = r_sensor.getValue()
 
    
    d_left = left - prev_left
    d_right = right - prev_right
    
    prev_left = left
    prev_right = right
    
    wheelr = 0.033
    left_dist = d_left * wheelr
    right_dist = d_right * wheelr
    d_center = (left_dist + right_dist) / 2
        
    x += d_center * math.sin(theta)
    y += d_center * math.cos(theta)
    
    theta = (theta + 2*math.pi) % (2*math.pi)
    if  theta < math.pi/4 or theta >= 7*math.pi/4:
        dir = 0 # north

    elif math.pi/4 <= theta < 3*math.pi/4:
        dir = 1 #east
       
    elif 3*math.pi/4 <= theta < 5*math.pi/4:
        dir = 2 # south

    else:
        dir = 3 #wesr

   # print(x,y)        
    return x, y, theta
    
    
def lidar_to_grid(grid, t):


    global grid_change
    ranges = lidar.getRangeImage()
    points = []
    down = ranges[-22:] + ranges[:22]
    left = ranges[67:112]
    front = ranges[157:202]
    right = ranges[247:292]
    r, c = world_to_grid(x, y, t, w, h)
    hit = [0,0,0,0]
    prev_hit = [0, 0, 0, 0]
    new_hit = [0,0,0,0]
    
    if grid[r][c][4]== 1:
       # print("grid", grid[r][c])
        for world_dir in range(4):   
           
            prev_hit = grid[r][c][0:4]
       # print("robot hits" ,prev_hit)
    for val in front:
       
        if val == float('inf'):
            continue
        if val < t*0.7 :
            hit[0]= 1
            break
      
            
    for val in left:
        if val == float('inf'):
            continue
        if val < t*0.7:
            hit[3]= 1
            break
     
            
    for val in right:
        if val == float('inf'):
            break
        if val < t*0.7:
            hit[1]= 1
            break
      
    for val in down:
        if val == float('inf'):
            continue
        if val < t*0.7:
            hit[2]= 1
            break
    
    if grid[r][c][4]== 1:
        for i in range(len(hit)):
            world_direction = (i+dir)%4
            new_hit[world_direction]=hit[i]
        
        #print(grid[r][c])
  
        #print("Previous his", prev_hit, r,c)
        #print("Hit:", hit)
        if  prev_hit != new_hit:
            grid_change = True
            
    for i in range(len(hit)):
        world_direction = (i+dir)%4

        grid[r][c][world_direction]=hit[i]


   # print("At lidar", hit
        
    return  grid[r][c], r, c, hit    
    
    
    
    
    
    
    
#________________________mapping
     
    
def get_accessible_neighbours(grid, r, c):
    neighbours = []

   
    options = []

    if grid[r][c][0] == 0:
        options.append((r+1, c, 0))  

    if grid[r][c][1] == 0:
        options.append((r, c+1, 1))  

    if grid[r][c][2] == 0:
        options.append((r-1, c, 2))  

    if grid[r][c][3] == 0:
        options.append((r, c-1, 3))  


    
    ordered = []

    for nr, nc, d in options:
        if d == dir:
            ordered.append((nr, nc))
        else:
            ordered.append((nr, nc))


    return ordered


def dfs_step(grid, r, c):

    global stack
 
    
    neighbours = get_accessible_neighbours(grid, r, c)

   
    for nr, nc in neighbours:
        if grid[nr][nc][4]==0:

            stack.append((r, c))   
            

   
            if nr == r + 1:
                move_to_neighbor(0)

            elif nr == r - 1:
                move_to_neighbor(2) 
 
            elif nc == c + 1:
                move_to_neighbor(1)

            elif nc == c - 1:
                move_to_neighbor(3)

            return
                
    if stack:
        pr, pc = stack.pop()

        if pr == r + 1:
            move_to_neighbor(0)
        elif pr == r - 1:
            move_to_neighbor(2)
        elif pc == c + 1:
            move_to_neighbor(1)
        elif pc == c - 1:
            move_to_neighbor(3)
 

        return
            
def exploration_done(grid, stack, r, c):
    
    neighbours = get_accessible_neighbours(grid, r, c)

    for nr, nc in neighbours:
        if grid[nr][nc][4] == 0:
            return False

    
   
    if len(stack) == 0:
        return True

    return False
    

    


def move_to_neighbor(direction):

    global xtarget, ytarget, motion

    if direction == 0:      
        xtarget = x
        ytarget = y + t

    elif direction == 1:   
        xtarget = x + t
        ytarget = y

    elif direction == 2:  
        xtarget = x
        ytarget = y - t

    elif direction == 3:    
        xtarget = x - t
        ytarget = y

    diff = (direction - dir) % 4

    if diff == 0:
        motion = 1

    elif diff == 1:
        right()

    elif diff == 3:
        left()

    elif diff == 2:
        motion = 4 
    return diff, rtarget, ctarget, state
        
    
        
def left():
    global dtarget, theta, motion
    dtarget = theta - math.pi/2
    dtarget = dtarget % (2*math.pi)
    motion = 2
    
def right():
    global dtarget, theta, motion
    dtarget = theta + math.pi/2
    dtarget = dtarget % (2*math.pi)
    motion = 3
    
    

#-----------------------------   printing 



    
def print_grid(grid):
  

    h = len(grid)
    w = len(grid[0])


    print("+" + "-----+" * w)

    for r in range(h-1, -1, -1):

 
        row = "|"

        for c in range(w):
            cell = grid[r][c]

            if cell[4]:
                row += "  V  "
            else:
                row += "  .  "

 
            if cell[1] == 1:
                row += "|"
            else:
                row += " "

        print(row)

 
        bottom = "+"

        for c in range(w):
            cell = grid[r][c]

            if cell[2] == 1:
                bottom += "-----+"
            else:
                bottom += "     +"

        print(bottom)














def follow_to_start():

    global stime, path_index, state, motion, path,  goal_path_generated, waypoints, greedy_start
    if path == None:
        print("Cannot reach the start point")
        state = "DONE"
        return
    
    if path_index >= len(path)-1:
        print("At Start point")
      
        
        motion = 0
  #      path, b_score, b_time = pathing()

                     
        if len(bonus)<7:
        
            path, b, tm = build_best_path(bonus.keys())
            motion = 0
    
    
              
            path_index = 0
            goal_path_generated = True
            if path != None:
                draw_path(path, 0.75,1,0)
                        
            print("Path: ", path)
            print("Points gonna be collected: ", b)
            print("time: ", tm)
            stime = robot.getTime()
            greedy_start = robot.getTime()
            print("Start time = ", stime)
            state = "GO_TO_GOAL"

            
        else:   
            state = "PLAN GREEDY"

        draw_bonuses(bonus) 
                

        return
        
    
    r1, c1 = path[path_index]
    r2, c2 = path[path_index + 1]
    
    if r2 == r1 + 1:
        move_to_neighbor(0)  

    elif r2 == r1 - 1:
        move_to_neighbor(2) 

    elif c2 == c1 + 1:
        move_to_neighbor(1)  

    elif c2 == c1 - 1:
        move_to_neighbor(3)  

    path_index += 1

def follow_to_goal():

    global path_index, state, motion, bonus_collected, time_left
    if path == None:
        print("No path in given time")
        state = "DONE"
        return
    
    if path_index >= len(path)-1:

        print("Reached goal!!")
  
        etime = robot.getTime()
        print("End Time: ", etime)
        finalt = etime - stime
        #print("Final Time: ",finalt)
        time_left = time - finalt

        l_wheel.setVelocity(0)
        r_wheel.setVelocity(0)

        motion = 0
        state = "DONE"


        return
    
    r1, c1 = path[path_index]
    r2, c2 = path[path_index + 1]
    
    if r2 == r1 + 1:
        move_to_neighbor(0)   # north

    elif r2 == r1 - 1:
        move_to_neighbor(2)   # south

    elif c2 == c1 + 1:
        move_to_neighbor(1)   # east

    elif c2 == c1 - 1:
        move_to_neighbor(3)   # west

    path_index += 1
    
    for i in range(len(bonus)):
        if (r, c) in bonus:
            print("Reached bonus:", (r,c), "With bonus points: ", bonus[(r,c)])
            bonus_collected+= bonus[(r, c)]
            del bonus[(r,c)]
            
            
    
    
    


def map_done():
    for i in range(len(grid)):
        for j in range(len(grid[i])):
            if grid[i][j][4] == 0:
                return False
            
    return True
 
def go_to_goal():
    print()
    #print("start_time: ", robot.getTime())

    global path
    global path_index
    global state

    path = astar(grid,
                 start,
                 goal)

    path_index = 0
  #  draw_path(path, 1)



def go_to_start():

    global path
    global path_index
    global state

    current_r, current_c = world_to_grid(x, y, t, w, h)
 
    path = astar(grid,
                 (current_r, current_c),
                 start)
   # print(path)
    path_index = 0
    
    
    
    
    
    

#-----------brute force stuff    
    
def build_full_path(waypoints):

    if len(waypoints) == 0:
        return astar(start, goal)
    full_path = []

    for i in range(len(waypoints) - 1):
        segment = astar(grid, waypoints[i], waypoints[i+1])

        if segment is None:
            print("No path between", waypoints[i], waypoints[i+1])
            return None

        
        if i > 0:
            segment = segment[1:]

        full_path += segment


    return full_path
    
    
    
    
    

    
import itertools

def total_bonus(selected):
    score = 0
    for point in selected:
        score += bonus[point]
    return score


def get_bonus_subsets():
    points = list(bonus.keys())
    subsets = []

    for r in range(len(points) + 1):
        for subset in itertools.combinations(points, r):
            for order in itertools.permutations(subset):

                subsets.append(order)

    return subsets
    

    for i in range(1, len(path) - 1):

        nr, nc = path[i+1]

        new_dr = nr - curr_r
        new_dc = nc - curr_c

        total += straight

 
        if (new_dr, new_dc) != (dr, dc):
            total += turn

        dr, dc = new_dr, new_dc
        curr_r, curr_c = nr, nc

    return total
    
          
def build_best_path(waypoints):

    best_path = None
    f_cost = float('inf')
    best_bonus = -1
    best_time = float("inf")

    for order in get_bonus_subsets():

        candidate = [start] + list(order) + [goal]

        full_path = build_full_path(candidate)

        if full_path is None:
            continue

        path_time = get_cost(full_path, dir)

        if path_time > time:
            continue
                 
            
        bonus = total_bonus(list(order))
     #   print(round(path_time,2), bonus) 

        if (bonus > best_bonus) or (bonus == best_bonus and path_time < best_time):
            best_bonus = bonus
            best_path = full_path
            best_time = path_time
            

    return best_path, best_bonus, best_time
    
def get_cost(path, start_dir):
    straight = 1.8
    turn = 1.45

    if not path or len(path) < 2:
        return 0

    def get_dir(a, b):
        dr = b[0] - a[0]
        dc = b[1] - a[1]

        if dr == -1 and dc == 0:
            return 0  # North
        elif dr == 0 and dc == 1:
            return 1  # East
        elif dr == 1 and dc == 0:
            return 2  # South
        elif dr == 0 and dc == -1:
            return 3  # West

    total = 0

    prev_dir = start_dir

    for i in range(len(path) - 1):

        curr_dir = get_dir(path[i], path[i + 1])

 
        total += straight

        diff = abs(curr_dir - prev_dir)

     
        if diff == 3:
            diff = 1

        
        if diff == 2:
            diff = 0

        total += diff * turn

        prev_dir = curr_dir

    return total
    
    
    
   
#--------------------------------

def greedy(cur, bonus, goal):
    global target, greedy_start, grad
    target = goal
    
    best_score = 0
    best_time = float('inf')

    original_path = astar(grid, cur, goal)

    if original_path is None:
        print("Shoot, no path to end possible")
        return None

    original_time = get_cost(original_path, dir)
    #print("time_left =", time_left)
   # print("original_time =", original_time)


    if original_time > time_left:
        print("oof! Cant get to goal in givven time")
        return None
        

    best_path = original_path

    for c, b in bonus.items():

        path1 = astar(grid, cur, c)
        path2 = astar(grid, c, goal)

        if path1 is None or path2 is None:
            continue

        b_path = path1 + path2[1:]

        b_time = get_cost(b_path, dir)

        if b_time > time_left:
            continue

        e_time = b_time - original_time

        if e_time <= 0:
            score = float('inf')
        else:
            score = b / (e_time+1)

        if score > best_score or (score == best_score and e_time < best_time):

            best_score = score
            best_time = e_time
            best_path = path1
            target = best_path[-1]
            
    for i in bonus.keys():
        if i == target:
            continue
        if i in best_path:
            target = i
            best_path = astar(grid, cur, i)
            best_time = get_cost(best_path, dir)
            
    if grad > 0:
        grad = grad - grad_diff
        draw_path(best_path, 1, grad, 0)
 
        
    else:
        draw_path(best_path, 1, 0, 0.7)
    
    greedy_start = robot.getTime()
 #   print(greedy_start)


    return best_path, best_score, best_time
        



def follow_to_bonus():
    global path_index, state, target, time_left, path, greedy_start, bonus_collected
   
    if (path_index >= len(path) -1):
        
        # reached bonus
        if target != goal:
            print("Reached bonus - ", target, "With bonus points - ", bonus[target])

        # remove collected bonus
        if target in bonus:
            bonus_collected+= bonus[target]
            del bonus[target]

        current_target = None
        path_index = 0
        greedy_end = robot.getTime()
        time_taken = greedy_end - greedy_start
        time_left = time_left - time_taken
     #   print("tiem left: ",time_left)
        
        
        
        if r == goal[0] and c == goal[1]:

            state = "DONE"
        
        else:
            state = "PLAN GREEDY"
            
         #   greedy_start = robot.getTime()
        

        return

    r1, c1 = path[path_index]
    r2, c2 = path[path_index + 1]

    if r2 == r1 + 1:
        move_to_neighbor(0)
    elif r2 == r1 - 1:
        move_to_neighbor(2)
    elif c2 == c1 + 1:
        move_to_neighbor(1)
    elif c2 == c1 - 1:
        move_to_neighbor(3)

    path_index += 1

    
    
    #------------------
    
    
    
def change_check(r1, c1, r2, c2, grid):
    dr = r2 - r1
    dc = c2 - c1
    ndir = None

    if dr == -1 and dc == 0:
        ndir = 2  # north
    elif dr == 0 and dc == 1:
        ndir = 1  # east
    elif dr == 1 and dc == 0:
        ndir =0  # south
    elif dr == 0 and dc == -1:
        ndir = 3  # west 
    if ndir is None:
        print("Invalid move:", (r1,c1), " to ", (r2,c2))
        return False
    
    if grid[r1][c1][ndir] == 1:
        return True
    return False  
    
    
"""
Motion
1 = Forward
2 = Tutn left
3 = right
4 = REVERSE

State
0 = desicion
1 = MAPPING
2 = GO_TO_START
3 = AT_START
4 = GO_TO_GOAL
5 = STOP

"""    









start = (0,0)
goal  = (3,4)
print("Start: ",start)
print("End: ", goal)

bonus = {(2,1): 15, (4,0):20, (4,3):10, (4,0): 25, (1,2): 10,(0,4):30}
time = 60

print("Time: ", time)
print("Bonuses:", bonus)

draw_start_goal(start, goal)


time_left = time
grid = map(w,h,t)
draw_start_goal(start, goal)
odometry()
r, c = world_to_grid(x, y, t, w, h)
g, row, col, hit = lidar_to_grid(grid, t)
dfs_step(grid, r, c)

grid = [[[0, 0, 1, 1, 1], [0, 0, 1, 0, 1], [0, 0, 1, 0, 1], [0, 1, 1, 0, 1], [0, 1, 1, 1, 1]], [[0, 0, 0, 1, 1], [0, 1, 0, 0, 1], [0, 0, 0, 1, 1], [0, 0, 0, 0, 1], [0, 1, 0, 0, 1]], [[0, 0, 0, 1, 1], [1, 1, 0, 0, 1], [1, 1, 0, 1, 1], [1, 0, 0, 1, 1], [0, 1, 0, 0, 1]], [[1, 0, 0, 1, 1], [0, 0, 1, 0, 1], [0, 0, 1, 0, 1], [0, 1, 1, 0, 1], [0, 1, 0, 1, 1]], [[1, 0, 1, 1, 1], [1, 0, 0, 0, 1], [1, 0, 0, 0, 1], [1, 0, 0, 0, 1], [1, 1, 0, 0, 1]]]
 
 
while robot.step(timestep) != -1:
    
    if 0 <= r < len(grid) and 0 <= c < len(grid[0]):
        grid[r][c][4] = 1
 



    x, y, theta = odometry()    
    r, c = world_to_grid(x, y, t, w, h)
 
    


    if motion == 1:
        l_wheel.setVelocity(6)
        r_wheel.setVelocity(6)

        dist = math.sqrt( (x - xtarget)**2 + (y - ytarget)**2)

        if dist < 0.0085:
            l_wheel.setVelocity(0)
            r_wheel.setVelocity(0)
            motion = 0
                
    elif motion == 2:
        l_wheel.setVelocity(-3)
        r_wheel.setVelocity(3)

        error = abs(theta - dtarget)

        if error > math.pi:
            error = 2*math.pi - error

        if error < 0.01:

            l_wheel.setVelocity(0)
            r_wheel.setVelocity(0)

            motion = 1   
            
    elif motion == 3:
        l_wheel.setVelocity(3)
        r_wheel.setVelocity(-3)

        error = abs(theta - dtarget)

        if error > math.pi:
            error = 2*math.pi - error

        if error < 0.01:

            l_wheel.setVelocity(0)
            r_wheel.setVelocity(0)

            motion = 1   
    
    
    
    elif motion == 4:
        l_wheel.setVelocity(-6)
        r_wheel.setVelocity(-6)
        
        dist = math.sqrt( (x - xtarget)**2 + (y - ytarget)**2)

        if dist < 0.0075:
            l_wheel.setVelocity(0)
            r_wheel.setVelocity(0)
            motion = 0
            
        
    if motion == 0:    
    
    
        
        if state == "MAPPING":
            
           
                  
            if at_cell_center():
                lidar_to_grid(grid, t)
                       
            if exploration_done(grid, stack, r, c) or map_done():
                l_wheel.setVelocity(0)
                r_wheel.setVelocity(0)
                grad_diff = 1/4
                print_grid(grid)
                print(grid)
                
   
                if not start_path_generated:

                    if ((r != goal[0] or c != goal[1])) and not start_path_generated:
                        print("Mapping is done!! Proceeding to start point: ", start) 

                        go_to_start()
                        motion = 0
                        
                        state = "GO_TO_START"
                       # draw_path(path)
                        
        
                elif not goal_path_generated:


                    if len(bonus)<6:
                        
                        path, b, tm = build_best_path(bonus.keys())
                        motion = 0
      #      path, b_score, b_time = pathing()
    
                         
                        path_index = 0
                        goal_path_generated = True
    
                        draw_path(path, 0.75, 1, 1)
                        
                        print("Path: ", path)
                        print("Points gonna be collected: ", b)
                        print("time: ", tm)
                        stime = robot.getTime()
   
                        state = "GO_TO_GOAL"
                    else:
                        state = "GREEDY"
                        print(state)
                        cur = (r, c)
                        path, score, extra = greedy(cur, bonus, goal)
                        greedy_start = robot.getTime()                            
                            
                    
                    #path, b_score, b_time = pathing()
                    
                   # print(path, b_score, b_time)
                    
                    
            
            if state == "MAPPING":
                dfs_step(grid,r,c)
            
            
        elif state == "GO_TO_START":
           
            
           
            follow_to_start()
          
            
        elif state == "GO_TO_GOAL":
                 

            if at_cell_center():
                lidar_to_grid(grid, t)
            if grid_change and motion == 0:

                if path_index < len(path):

                    nr, nc = path[path_index+1]   # next cell robot wants to go to
    
                    if change_check(r, c, nr, nc, grid):
                        
        
                        print("Next move blocked! ", (nr, nc),". Recaliberating...")
                                         
                        state = "PLAN GREEDY"
                        greedy_start = stime
                        greedy_end = robot.getTime()
                        time_taken = greedy_end-greedy_start
                        time_left -= time_taken
                        

                grid_change = False
                
            if state == "GO_TO_GOAL":
                   
            
                follow_to_goal()   
            
     
      
            
        elif state == "GREEDY":
            if at_cell_center():
               lidar_to_grid(grid, t)
            if grid_change and motion == 0:

                if path_index < len(path)-1:
   

                    nr, nc = path[path_index+1] 
    
                    if change_check(r, c, nr, nc, grid):
                        
        
                        print("Next move blocked!", (nr, nc), ". recaliberating...")
                                         
                        state = "PLAN GREEDY"
                        
                        greedy_end = robot.getTime()
                        time_taken = greedy_end-greedy_start
                        time_left -= time_taken
                        print(greedy_start,
                        greedy_end,
                        time_taken ,
                        time_left)
          

                grid_change = False
                
            if state == "GREEDY":
                   
            
                follow_to_bonus()   
            
     
        elif state == "PLAN GREEDY":
     
            cur = (r, c)
            result = greedy(cur, bonus, goal)
            if result is None:
                print("No path possible. Exiting.")
                state = "DONE"
                continue
            else:
                
                path, score, extra = result
           # if target!= goal:
             #   bonus_collected += bonus[target]
            path_index = 0
            state = "GREEDY"
           
            greedy_start = robot.getTime() 
            
            
            
            
            
            
              
        elif state == "DONE":
            l_wheel.setVelocity(0)
            r_wheel.setVelocity(0)
            print("Time Taken: ", time - time_left)
            print("Bonus Points collected: ",bonus_collected)
            print(state)
                     
            break
            






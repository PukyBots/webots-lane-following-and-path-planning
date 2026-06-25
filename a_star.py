def get_neighbour(grid, r, c):

    top, right, bottom, left, visit = grid[r][c]

    neighbours = []

  
    if top == 0 and r < len(grid)-1:
        neighbours.append((r+1, c))

    
    if right == 0 and c < len(grid[0])-1:
        neighbours.append((r, c+1))

   
    if bottom == 0 and r > 0:
        neighbours.append((r-1, c))

    
    if left == 0 and c > 0:
        neighbours.append((r, c-1))

    return neighbours    
def heuristic( current, goal):
    return abs(goal[0] - current[0]) + abs(goal[1] - current[1])
    
def astar(grid, start, goal):

    open_set = [start]

    g_score = {
        start: 0
    }

    came_from = {}
    
    closed_set = set()

    while open_set:
        

        current = open_set[0]

        for cell in open_set:

            current_f = heuristic(current, goal)+ g_score[current]
            cell_f = heuristic(cell, goal)+g_score[cell]

            if cell_f < current_f:
                current = cell
        
                
        if current == goal:
            #print("Goal found!")
            curr = goal
            path = [goal]
    
            while curr!=start:
                path.append(came_from[curr])
                curr = came_from[curr]

            path.reverse()
            #print( path)
            return path
    
        open_set.remove(current)
        closed_set.add(current)

        neighbours = get_neighbour(
            grid,
            current[0],
            current[1]
        )

        for neighbour in neighbours:
            if neighbour in closed_set:
                continue

            new_g = g_score[current]+1

            if (
                neighbour not in g_score
                or
                new_g < g_score[neighbour]
            ):

                g_score[neighbour] = new_g

                came_from[neighbour] = current

                if neighbour not in open_set:
                    open_set.append(neighbour) 
       # print("Current:", current)
      #  print("Open set:", open_set)



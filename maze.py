# Michael Sharp and Emmet Hoverston

'''
To execute code:

Navigate to terminal and use command:

Python3 maze.py

'''

from pyamaze import maze, agent, textLabel
from queue import PriorityQueue
import time

'''
Search algorithm code was derived from Stuart Russell and Peter Norvig, 
"Artificial Intelligence. A modern approach. 4th Edition" Pearson, 2020.
and Abeer Asif with The Medium, 
Asif, Abeer. “Genetic Algorithm for Maze Search.” Medium, Medium, 2 Apr. 2023, 
medium.com/@abeerasif232/genetic-algorithm-for-maze-search-5ced3f5035e7. 
'''

# Depth-First Search
def DFS(m):
    start = (m.rows, m.cols)
    explored = [start]
    frontier = [start]
    path = {}
    while len(frontier) > 0:
        currCell = frontier.pop()
        if currCell == (1, 1):
            break
        for direction in 'ESNW':
            if m.maze_map[currCell][direction]: 
                if direction == 'E':
                    nextCell = (currCell[0], currCell[1] + 1)
                elif direction == 'W':
                    nextCell = (currCell[0], currCell[1] - 1)
                elif direction == 'N':
                    nextCell = (currCell[0] - 1, currCell[1])
                elif direction == 'S':
                    nextCell = (currCell[0] + 1, currCell[1])
                if nextCell not in explored:
                    explored.append(nextCell)
                    frontier.append(nextCell)
                    path[nextCell] = currCell
    fwdPath = {}
    cell = (1, 1)
    while cell != start:
        fwdPath[path[cell]] = cell
        cell = path[cell]
    return fwdPath

# Breadth-First Search
def BFS(m):
    start = (m.rows, m.cols)
    explored = [start]
    queue = [start]
    path = {}
    while len(queue) > 0:
        currCell = queue.pop(0)
        if currCell == (1, 1):
            break
        for direction in 'ESNW':
            if m.maze_map[currCell][direction]:
                if direction == 'E':
                    nextCell = (currCell[0], currCell[1] + 1)
                elif direction == 'W':
                    nextCell = (currCell[0], currCell[1] - 1)
                elif direction == 'N':
                    nextCell = (currCell[0] - 1, currCell[1])
                elif direction == 'S':
                    nextCell = (currCell[0] + 1, currCell[1])
                if nextCell not in explored:
                    explored.append(nextCell)
                    queue.append(nextCell)
                    path[nextCell] = currCell
    fwdPath = {}
    cell = (1, 1)
    while cell != start:
        fwdPath[path[cell]] = cell
        cell = path[cell]
    return fwdPath

# A* Search
def AStar(m):
    start = (m.rows, m.cols)
    g_score = {cell: float('inf') for cell in m.grid}
    g_score[start] = 0
    f_score = {cell: float('inf') for cell in m.grid}
    f_score[start] = heuristic(start, (1, 1))

    openList = PriorityQueue()
    openList.put((f_score[start], start))
    path = {}

    while not openList.empty():
        currCell = openList.get()[1]
        if currCell == (1, 1):  
            break
        for direction in 'ESNW':
            if m.maze_map[currCell][direction]:
                if direction == 'E':
                    nextCell = (currCell[0], currCell[1] + 1)
                elif direction == 'W':
                    nextCell = (currCell[0], currCell[1] - 1)
                elif direction == 'N':
                    nextCell = (currCell[0] - 1, currCell[1])
                elif direction == 'S':
                    nextCell = (currCell[0] + 1, currCell[1])

                temp_g_score = g_score[currCell] + 1
                if temp_g_score < g_score[nextCell]:
                    g_score[nextCell] = temp_g_score
                    f_score[nextCell] = temp_g_score + heuristic(nextCell, (1, 1))
                    openList.put((f_score[nextCell], nextCell))
                    path[nextCell] = currCell

    fwdPath = {}
    cell = (1, 1)
    while cell != start:
        fwdPath[path[cell]] = cell
        cell = path[cell]
    return fwdPath

# Heuristic function for A*
def heuristic(cell1, cell2):
    x1, y1 = cell1
    x2, y2 = cell2
    return abs(x1 - x2) + abs(y1 - y2)

# Depth-Limited Search used in IDDFS
def DLS(m, start, goal, depth):
    explored = set()
    path = {}

    def recursive_dls(cell, current_depth):
        if cell == goal: 
            return True
        if current_depth == 0:  
            return False
        for direction in 'ESNW':
            if m.maze_map[cell][direction]:  
                if direction == 'E':
                    nextCell = (cell[0], cell[1] + 1)
                elif direction == 'W':
                    nextCell = (cell[0], cell[1] - 1)
                elif direction == 'N':
                    nextCell = (cell[0] - 1, cell[1])
                elif direction == 'S':
                    nextCell = (cell[0] + 1, cell[1])
                if nextCell not in explored:
                    explored.add(nextCell)
                    path[nextCell] = cell
                    if recursive_dls(nextCell, current_depth - 1):
                        return True
        return False

    explored.add(start)
    if recursive_dls(start, depth):
        fwdPath = {}
        cell = goal
        while cell != start: 
            fwdPath[path[cell]] = cell
            cell = path[cell]
        return fwdPath
    return None

# Iterative Deepening Depth-First Search
def IDDFS(m):
    start = (m.rows, m.cols)
    goal = (1, 1)
    for depth in range(1, 1000): 
        result = DLS(m, start, goal, depth)
        if result is not None: 
            return result
    return None

if __name__ == '__main__':

    m = maze(30, 30) # Change this for different size mazes
    m.CreateMaze(loopPercent=70) # Loop percent at 70 allows different paths to goal state

    # Time and run search algorithms
    start_time = time.time()
    dfsPath = DFS(m)
    dfs_time = time.time() - start_time

    start_time = time.time()
    bfsPath = BFS(m)
    bfs_time = time.time() - start_time

    start_time = time.time()
    aStarPath = AStar(m)
    aStar_time = time.time() - start_time

    start_time = time.time()
    iddfsPath = IDDFS(m)
    iddfs_time = time.time() - start_time

    # Show paths
    a1 = agent(m, footprints=True, color='red') 
    a2 = agent(m, footprints=True, color='blue')  
    a3 = agent(m, footprints=True, color='green')  
    a4 = agent(m, footprints=True, color='yellow')  

    m.tracePath({a1: dfsPath}, delay=100)
    m.tracePath({a2: bfsPath}, delay=100)
    m.tracePath({a3: aStarPath}, delay=100)
    m.tracePath({a4: iddfsPath}, delay=100)

    # Show results
    textLabel(m, 'DFS Runtime (s)', round(dfs_time, 4))
    textLabel(m, 'BFS Runtime (s)', round(bfs_time, 4))
    textLabel(m, 'A* Runtime (s)', round(aStar_time, 4))
    textLabel(m, 'IDDFS Runtime (s)', round(iddfs_time, 4))
    textLabel(m, 'DFS Path Length', len(dfsPath))
    textLabel(m, 'BFS Path Length', len(bfsPath))
    textLabel(m, 'A* Path Length', len(aStarPath))
    textLabel(m, 'IDDFS Path Length', len(iddfsPath))

    m.run()
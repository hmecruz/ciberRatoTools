from collections import deque

def bfs(agent):
    start_cell = agent.robot.cell
    queue = deque([start_cell])
    came_from = {start_cell: None}

    while queue:
        current_cell = queue.popleft()

        if agent.maze.has_neighbours_to_explore(current_cell):
            agent.robot.pathfinding_path = reconstruct_path(current_cell, came_from)
            return True

        for neighbour in agent.maze.get_neighbours(current_cell):
            if neighbour not in came_from:
                queue.append(neighbour)
                came_from[neighbour] = current_cell

    print("Map exploration complete.")
    return False

def reconstruct_path(current_cell, came_from):
    total_path = [current_cell]
    while current_cell in came_from and came_from[current_cell] is not None:
        current_cell = came_from[current_cell]
        total_path.append(current_cell)
    total_path.reverse()
    return total_path
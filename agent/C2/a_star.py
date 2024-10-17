from heapq import heappop, heappush
from constants import *

def a_star(agent):
    start_cell = agent.robot.cell # Current Cell
    goal_cell = find_closest_cell_with_neighbours(agent)

    if not goal_cell:
        print("No cells with neighbours to explore found.")
        return

    open_set = []
    heappush(open_set, (0, start_cell))
    came_from = {}
    g_score = {start_cell: 0}
    f_score = {start_cell: heuristic(start_cell, goal_cell)}

    while open_set:
        current = heappop(open_set)

        if current == goal_cell:
            agent.robot.a_star_path = reconstruct_path(came_from, current)
            return

        for neighbour in get_neighbours(agent, current):
            tentative_g_score = g_score[current] + 1  # Assuming uniform cost for simplicity

            if neighbour not in g_score or tentative_g_score < g_score[neighbour]:
                came_from[neighbour] = current
                g_score[neighbour] = tentative_g_score
                f_score[neighbour] = tentative_g_score + heuristic(neighbour, goal_cell)
                heappush(open_set, (f_score[neighbour], neighbour))


def find_closest_cell_with_neighbours(agent):
    visited_cells = agent.maze.get_visited_cells() # List of visited cells
    closest_cell = None
    min_distance = float('inf')

    for cell in visited_cells:
        if agent.maze.has_neighbours_to_explore(cell):
            distance = heuristic(agent.robot.cell, cell)
            if distance < min_distance:
                min_distance = distance
                closest_cell = cell

    return closest_cell


def heuristic(cell1, cell2):
    return abs(cell1.bl_x - cell2.bl_x) + abs(cell1.bl_y - cell2.bl_y)


def get_neighbours(robot, cell):
    directions = [MOVE_NORTH, MOVE_WEST, MOVE_EAST, MOVE_SOUTH]
    neighbours = []
    for direction in directions:
        neighbour_coords = (cell.bl_x + direction, cell.bl_y + direction)
        if neighbour_coords in robot.maze.map and robot.maze.map[neighbour_coords]:
            neighbours.append(robot.maze.map[neighbour_coords])
    return neighbours


def reconstruct_path(came_from, current):
    total_path = [current]
    while current in came_from:
        current = came_from[current]
        total_path.append(current)
    total_path.reverse()
    return total_path

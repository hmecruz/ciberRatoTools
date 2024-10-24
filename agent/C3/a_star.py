from heapq import heappop, heappush
from constants import *

class PriorityCell:
    def __init__(self, priority, cell):
        self.priority = priority
        self.cell = cell

    def __lt__(self, other):
        return self.priority < other.priority

def a_star(agent):
    start_cell = agent.robot.cell # Current cell
    goal_cell = agent.maze.get_cell(agent.robot.initial_position)  # Goal is the initial position

    open_set = []
    heappush(open_set, PriorityCell(0, start_cell))
    came_from = {}
    g_score = {start_cell: 0}
    f_score = {start_cell: heuristic(start_cell, goal_cell)}

    while open_set:
        current_priority_cell = heappop(open_set)
        current_cell = current_priority_cell.cell # Extract the cell from the wrapper

        if current_cell == goal_cell:
            agent.robot.pathfinding_path = reconstruct_path(current_cell, came_from)
            return True

        for neighbour in agent.maze.get_neighbours(current_cell):
            tentative_g_score = g_score[current_cell] + 1

            if neighbour not in g_score or tentative_g_score < g_score[neighbour]:
                came_from[neighbour] = current_cell
                g_score[neighbour] = tentative_g_score
                f_score[neighbour] = tentative_g_score + heuristic(neighbour, goal_cell)
                heappush(open_set, PriorityCell(f_score[neighbour], neighbour))

    print("Failed to find path to initial position.")
    return False

def heuristic(cell1, cell2):
    return abs(cell1.bl_x - cell2.bl_x) + abs(cell1.bl_y - cell2.bl_y)


def reconstruct_path(current_cell, came_from):
    total_path = [current_cell]
    while current_cell in came_from:
        current_cell = came_from[current_cell]
        total_path.append(current_cell)
    total_path.reverse()
    return total_path

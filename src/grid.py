import matplotlib.pyplot as plt
import math
import random

class Node:
    def __init__(self, position, parent=None):
        # init node with a position and (optional) parent
        self.position = position # tuple (row, colum)    
        self.parent = parent # reference to the parent nod (where we came from)  

        # init costs for A* cost function f(n)=g(n)+h(n)
        self.g = 0 # cost from start node to current node
        self.h = 0 # estimate cost from current node to the goal
        self.f = 0 # total cost (g + h)

# A* pathfinding function
def a_star(grid, start, goal):
    # init start node and goal node
    start_node = Node(position=start)
    goal_node = Node(position=goal)

    # init node lists
    open_list = [] # nodes to be evaluated
    closed_list = [] # Nodes already evaluated
    open_list.append(start_node) # adds the start node to the open list as it's the first we'll evaluate

    # while there are nodes to evaluate in the open list
    while open_list:
        current_node = min(open_list, key=lambda node: node.f) # find the node in the open list with the lowest f cost

        # if goal is reached, reconstruct path
        if current_node.position == goal_node.position:
            path = []
            while current_node is not None:
                path.append(current_node.position)
                current_node = current_node.parent
            return path[::-1] # path reversed here will go from start to goal
        
        # remove current node from open list and add to closed list
        open_list.remove(current_node)
        closed_list.append(current_node)

        # generate neighbors to current node (up, down, left, right)
        neighbors = [
            (current_node.position[0] - 1, current_node.position[1]),  # Up
            (current_node.position[0] + 1, current_node.position[1]),  # Down
            (current_node.position[0], current_node.position[1] - 1),  # Left
            (current_node.position[0], current_node.position[1] + 1)   # Right
        ]

        # looping for neighbor node costs
        for next_position in neighbors:
            # only eval neighbors within grid (skip if not)
            if next_position[0] < 0 or next_position[0] >= len(grid) or next_position[1] < 0 or next_position[1] >= len(grid[0]):
                continue

            # create new node for each new neighbor position
            neighbor_node = Node(position=next_position, parent=current_node)

            # skip if neighbor already in closed list (already evaluated)
            if any(closed_node.position == neighbor_node.position for closed_node in closed_list):
                continue

            # calculate costs for neighbor node
            neighbor_node.g = current_node.g + 1 # assumes uniform cost to move to a neighbor
            #neighbor_node.h = math.sqrt((neighbor_node.position[0] - goal_node.position[0]) ** 2 + (neighbor_node.position[1] - goal_node.position[1]) ** 2)  # euclidean distance from neighbor node to goal node
            neighbor_node.h = abs(neighbor_node.position[0] - goal_node.position[0]) + abs(neighbor_node.position[1] - goal_node.position[1]) # manhattan distance from neighbor node to goal node
            neighbor_node.f = neighbor_node.g + neighbor_node.h

            # skip if neighbor is already in open list with lower g cost 
            if any(open_node.position == neighbor_node.position and neighbor_node.g >= open_node.g for open_node in open_list):
                continue

            # add neighbor to open list
            open_list.append(neighbor_node)

    # if open list is exhausted, there is no path
    return None

# create 2D grid
grid = []
grid_size = 100
for i in range(grid_size):
    grid.append([0] * grid_size)

# define start and goal (end) points
start = (0, 0)
#goal = (grid_size-1, grid_size-1)
goal = (random.randint(0, grid_size - 1), random.randint(0, grid_size - 1)) # random goal point

# function to visualize grid
def vis_grid(grid, start, goal, path=None):
    grid_size = len(grid)
    
    plt.figure()
    plt.xlim(-0.5, grid_size - 0.5)  # Set x-axis limits to cover the grid range
    plt.ylim(-0.5, grid_size - 0.5)  # Set y-axis limits to cover the grid range
    plt.gca().invert_yaxis()  # Make (0,0) the top-left of the plot
    
    
    # add faint grey grid lines at every integer coordinate
    plt.xticks(range(grid_size))
    plt.yticks(range(grid_size))
    plt.grid(color='lightgrey', linestyle='-', linewidth=0.5) # add faint grey gridlines
    
    # plot relevant points (start, goal, current, obstacle, etc.)
    plt.plot(start[1], start[0], 'go', label='Start')
    plt.plot(goal[1], goal[0], 'ro', label='Goal')

    # plot path (if one is found)
    if path:
        path_x = [position[1] for position in path]  # Extract x-coordinates
        path_y = [position[0] for position in path]  # Extract y-coordinates
        plt.plot(path_x, path_y, color='blue', linewidth=2, label='Path') 

    plt.legend(bbox_to_anchor=(1.2, 1), loc='upper right', fancybox=True, shadow=True)
    plt.title("2D Grid Environment")
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    path = a_star(grid, start, goal) # run A* function to find the optimal path
    vis_grid(grid, start, goal, path) # visualize the grid space with start, end, and path


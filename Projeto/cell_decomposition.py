import numpy as np
import matplotlib.pyplot as plt
import sys

class Cell():
    def __init__(self, values, height, weight, origin, target, g, num_cell, image_size):
        self.origin = origin
        self.center = self.calculateCenter(origin, height, weight)
        self.movement_cost = g
        self.heuristic = self.calculateHeuristic(target)
        self.total_cost = self.movement_cost + self.heuristic
        self.is_obstacle = self.isObstacle(values, height, weight)
        self.id = num_cell
        self.max_row_cells = image_size//weight
        self.max_cells = image_size//height * self.max_row_cells
        self.neighbors = self.findNeighbors()
        self.values = values
        self.parent = None

    def calculateCenter(self,origin, height, weight):
        return (origin[0]+height/2, origin[1]+weight/2)

    def calculateHeuristic(self,target):
        (x1, y1) = self.center
        (x2, y2) = target
        return abs(x1 - x2) + abs(y1 - y2)

    def isObstacle(self, values, height, weight):

        for i in range(height):
            for j in range(weight):
                if values[i][j] != False:
                    return True
        return False

    def findNeighbors(self):
        neighbors = []

        if self.id + 1 <= self.max_cells and self.id%self.max_row_cells!=0:
            neighbors.append(self.id + 1)
        if self.id - 1 > 0 and (self.id-1)%self.max_row_cells!=0:
            neighbors.append(self.id - 1)
        if self.id + self.max_row_cells <= self.max_cells:
            neighbors.append(self.id + self.max_row_cells)
        if self.id - self.max_row_cells > 0:
            neighbors.append(self.id - self.max_row_cells)
        if self.id + self.max_row_cells + 1 <= self.max_cells and (self.id%self.max_row_cells)!=0:
            neighbors.append(self.id + self.max_row_cells + 1)
        if self.id + self.max_row_cells - 1 <= self.max_cells and ((self.id-1)%self.max_row_cells)!=0 and self.id != 1:
            neighbors.append(self.id +self.max_row_cells - 1)
        if self.id - self.max_row_cells - 1 > 0 and ((self.id-1)%self.max_row_cells)!=0:
            neighbors.append(self.id - self.max_row_cells - 1)
        if self.id - self.max_row_cells + 1 > 0 and (self.id%self.max_row_cells)!=0:
            neighbors.append(self.id - self.max_row_cells + 1)

        return neighbors

    def recalculateCost(self, new_cost):
        self.movement_cost = new_cost
        self.total_cost = self.movement_cost + self.heuristic

class Graph():
    def __init__(self):
        self.graph_map = {}
        self.last_node = 0

    def addNode(self, node):
        self.graph_map[node.id] = node

    def setNodeCost(self, id, cost):
        self.graph_map[id].recalculateCost(cost)

class CellDecomposition():
    def __init__(self, grid_map, origin_point, destiny_point, size):
    
        self.graph = Graph()
        self.grid_map = grid_map
        self.size = size

        num_cell = 0
        for i in range(0, 500, self.size):
            for j in range(0, 500, self.size):
                num_cell += 1
                cell = Cell(grid_map.gridMap[i:i+self.size,j:j+self.size], self.size, self.size, (i,j), destiny_point, 0, num_cell, 500)
                self.graph.addNode(cell)
        
        path = self.a_star(origin_point, destiny_point)
        print("path", path)

        for i in range(len(path)):
            plt.scatter(path[i][1], path[i][0], s=1, color='r')

        plt.imshow(grid_map.gridMap, origin='lower')
        plt.show()

    def findInitialNode(self, start):
        initialNode = None
        for node in self.graph.graph_map.values():
            if ((abs(node.center[0] - start[1]) < self.size/2) and (abs(node.center[1] - start[0]) < self.size/2)):
                initialNode = node
        initialNode.movement_cost = 0
        initialNode.heuristic = 0
        initialNode.total_cost = 0

        return initialNode

    def findFinalNode(self, end):
        finalNode = None
        for node in self.graph.graph_map.values():
            if ((abs(node.center[0] - end[0]) < self.size/2) and (abs(node.center[1] - end[1]) < self.size/2)):
                finalNode = node

        finalNode.movement_cost = 0
        finalNode.heuristic = 0
        finalNode.total_cost = 0
        return finalNode

    def a_star(self, start, end):
        intial_node = self.findInitialNode(start)
        final_node = self.findFinalNode(end)

        open_list = []
        closed_list = {}

        open_list.append(intial_node)

        while len(open_list) > 0:
            
            open_list.sort(key=lambda n: n.total_cost, reverse=False)
            
            current_node = open_list[0]
            open_list.pop(0)

            closed_list[current_node.id] = current_node

            if current_node.id == final_node.id:
                path = []
                current = current_node
                while current is not None:
                    path.append(current.center)
                    current = current.parent
                return path[::-1]
            
            for neighbor in current_node.neighbors:
                
                #Verificacao se é obstáculo ou se está na lista fechada. Não faz nada para ambos os casos
                if (self.graph.graph_map[neighbor].is_obstacle) or (neighbor in closed_list):
                    continue

                self.graph.graph_map[neighbor].parent = current_node 

                if  neighbor == current_node.id + 1 or \
                    neighbor == current_node.id - 1 or \
                    neighbor == current_node.id + current_node.max_row_cells or \
                    neighbor == current_node.id - current_node.max_row_cells:
                    self.graph.setNodeCost(neighbor, current_node.movement_cost+10)
                else:
                    self.graph.setNodeCost(neighbor, current_node.movement_cost+14)
                
                open_list_member = False
                for open_neighbor in open_list:
                    if neighbor == open_neighbor.id:
                        if self.graph.graph_map[neighbor].movement_cost <  open_neighbor.movement_cost:
                            open_neighbor.movement_cost = self.graph.graph_map[neighbor].movement_cost
                            open_neighbor.parent = current_node
                        open_list_member = True
                if not open_list_member:
                    open_list.append(self.graph.graph_map[neighbor])    


    

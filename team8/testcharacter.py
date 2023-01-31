# This is necessary to find the main code
from queue import PriorityQueue
import sys
sys.path.insert(0, '../bomberman')
# Import necessary stuff
from entity import CharacterEntity
from colorama import Fore, Back

class TestCharacter(CharacterEntity):
    move_count = 0
    path_plan = True 
    
    def do(self, wrld):
        if self.path_plan:
            start = (self.x,self.y)
            goal = wrld.exitcell
            came_from, cost_incurred = self.A_star(wrld,start,goal)
            self.path = self.get_path(came_from, start, goal)
            self.path_plan = False

        else:
            dx, dy = self.extract_move(self.path[self.move_count])
            self.set_cell_color(self.path[self.move_count][0], self.path[self.move_count][1], Back.RED)
            self.move(dx, dy)
            self.move_count+=1

  

    def A_star(self,wrld, start, goal):
        frontier = PriorityQueue()
        frontier.put(start,0)
        came_from = {}  
        cost_incurred = {}  
        came_from[start] = None
        cost_incurred[start]=0

        while not frontier.empty():
            current  =frontier.get()
            
            if current == goal:
                break
            
            for next in self.get_neighbors_4(wrld, current):
                new_cost = cost_incurred[current] + self.get_Gn(wrld,current,next)
                
                if next not in cost_incurred or new_cost < cost_incurred[next]:
                    cost_incurred[next]= new_cost
                    priority = new_cost + self.get_Hn(goal, next)
                    frontier.put(next, priority)
                    came_from[next] = current
                    
        return came_from, cost_incurred
    
    def get_neighbors_4(self,wrld, current):
        neighbors = []
        right_x = current[0]+1
        left_x = current[0]-1
        up_y = current[1]-1
        down_y = current[1]+1
        if 0 < right_x <  wrld.width():
            if not wrld.wall_at(right_x, current[1]):
                neighbors.append((right_x, current[1]))
        if  0 < left_x <  wrld.width():
            if not wrld.wall_at(left_x, current[1]):
                neighbors.append(( left_x, current[1])) 
        if 0 < up_y <  wrld.height():
            if not wrld.wall_at(current[0], up_y):
                neighbors.append((current[0], up_y))
        if 0 < down_y < wrld.height():
            if not wrld.wall_at(current[0], down_y):
                neighbors.append((current[0], down_y))   
        return(neighbors)
    
    def get_neighbors_8(self,current):
        neighbors = self.get_neighbors_4
        right_x = current[0]+1
        left_x = current[0]-1
        up_y = current[1]-1
        down_y = current[1]+1
        if right_x and up_y > 0:
            neighbors.append((right_x, up_y))
        if right_x and down_y > 0:
            neighbors.append((right_x, down_y)) 
        if  left_x and up_y > 0:
            neighbors.append((left_x, up_y)) 
        if  left_x and down_y > 0:
            neighbors.append((left_x, down_y))  
        return(neighbors)
    
    def extract_move(self, next):
        return next[0]-self.x, next[1]-self.y

     # the G(n) is the manhattan distance or right angle distance distance
    def get_Gn(self, wrld, goal, next):
        return abs(goal[0]-next[0]) + abs(goal[1]-next[1])

    # the h(n) is the euclidean_distance or straight line distance
    def get_Hn(self, goal, next):
        return abs(pow((goal[0]-next[0]), 2) + pow((goal[1]-next[1]), 2))

    def get_path(self, came_from, start, goal):
        current = goal
        path=[]
        if goal not in came_from:
            return []
        while current != start:
            self.set_cell_color(current[0], current[1],Fore.RED + Back.GREEN)
            path.append(current)
            current= came_from[current]
        # path.append(start)
        path.reverse()
        return path

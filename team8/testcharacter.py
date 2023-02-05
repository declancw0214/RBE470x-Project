# This is necessary to find the main code
from queue import PriorityQueue
import sys
sys.path.insert(0, '../bomberman')
# Import necessary stuff
from entity import CharacterEntity
from colorama import Fore, Back
import math

class TestCharacter(CharacterEntity):
    move_count = 0
    path_plan = True 
    
    def do(self, wrld):

        status = self.check_3_spaces(wrld)

        if (status != False):
            print('running')
            self.run(wrld, status)

        elif self.path_plan:
            print('planning')
            start = (self.x,self.y)
            goal = wrld.exitcell
            #self.move_count = 0
            came_from, cost_inpycurred = self.A_star(wrld,start,goal)
            #print(came_from)
            #self.path = None
            self.path = self.get_path(came_from, start, goal)
            print(self.path)
            self.path_plan = False
            dx, dy = self.extract_move(self.path[self.move_count])
            self.set_cell_color(self.path[self.move_count][0], self.path[self.move_count][1], Back.RED)
            print('move_count: ', self.move_count)
            print('move: ', dx, dy)
            self.move(dx, dy)
            self.move_count+=1

        else:
            
            if (status == False):
                print('on path')
                dx, dy = self.extract_move(self.path[self.move_count])
                self.set_cell_color(self.path[self.move_count][0], self.path[self.move_count][1], Back.RED)
                print('move_count: ', self.move_count)
                print('move: ', dx, dy)
                self.move(dx, dy)
                self.move_count+=1
            else:
                 print('need replan')
                 #self.move_count = 0
                 self.path_plan = True

  

    def A_star(self,wrld, start, goal):
        self.move_count = 0
        frontier = PriorityQueue()
        frontier.put(start, 0)
        came_from = {}  
        cost_incurred = {}  
        came_from[start] = None
        cost_incurred[start]=0

        while not frontier.empty():
            current = frontier.get()
            #print(current)
            
            if current == goal:
                break
            
            for next in self.get_neighbors_8(wrld, current):
                if not wrld.wall_at(next[0], next[1]):
                    new_cost = cost_incurred[current] + self.get_Gn(wrld,current,next)
                
                    if next not in cost_incurred or new_cost < cost_incurred[next]:
                        cost_incurred[next] = new_cost
                        priority = new_cost + self.get_Hn(goal, next)
                        frontier.put(next, priority)
                        came_from[next] = current
                    
        return came_from, cost_incurred
    
    def get_neighbors_4(self, wrld, current):
        neighbors = []
        right_x = current[0]+1
        left_x = current[0]-1
        up_y = current[1]-1
        down_y = current[1]+1
        if right_x <  wrld.width():
            #if not wrld.wall_at(right_x, current[1]):
                neighbors.append((right_x, current[1]))
        if  0 <= left_x:
            #f not wrld.wall_at(left_x, current[1]):
                neighbors.append(( left_x, current[1])) 
        if 0 <= up_y:
            #if not wrld.wall_at(current[0], up_y):
                neighbors.append((current[0], up_y))
        if down_y < wrld.height():
            #print(current[0], down_y)
            #if not wrld.wall_at(current[0], down_y):
                neighbors.append((current[0], down_y))   
        return(neighbors)
    
    def get_neighbors_8(self, wrld, current):
        #print(current)
        neighbors = self.get_neighbors_4(wrld, current)
        right_x = current[0]+1
        left_x = current[0]-1
        up_y = current[1]-1
        down_y = current[1]+1
        if (right_x < wrld.width()) and (up_y >= 0):
            neighbors.append((right_x, up_y))
        if (right_x < wrld.width()) and (down_y < wrld.height()):
            neighbors.append((right_x, down_y)) 
        if  (left_x >= 0) and (up_y >= 0):
            neighbors.append((left_x, up_y)) 
        if  (left_x >= 0) and (down_y < wrld.height()):
            neighbors.append((left_x, down_y))  
        #print(neighbors)
        return(neighbors)
    
    def extract_move(self, next):
        print('next: ', next)
        print('self: ', self.x, self.y)
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
            current = came_from[current]
        # path.append(start)
        path.reverse()
        return path
    
    def check_3_spaces(self, wrld):
        for current in self.get_neighbors_8(wrld, (self.x, self.y)):
            for current2 in self.get_neighbors_8(wrld, (current)):
                for current3 in self.get_neighbors_8(wrld, (current2)):

                    if(wrld.monsters_at(current3[0], current3[1]) != None):
                        print('found monster')
                        monster_pos = current3
                        return monster_pos
        return False
    
    def run(self, wrld, status):
        dx, dy = self.x - status[0], self.y - status[1]
        
        #dx, dy = self.move_no_wall(wrld, dx, dy)
        print('run direction: ', dx, dy)
        self.move(dx, dy)
        self.path_plan = True

    def move_no_wall(self, wrld, dx, dy):
        pass

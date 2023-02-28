# This is necessary to find the main code
from queue import PriorityQueue
import sys
sys.path.insert(0, '../bomberman')
# Import necessary stuff
from entity import CharacterEntity
from colorama import Fore, Back
import math
import pandas as pd
import csv
import numpy as np
from numpy.linalg import norm
import numpy as np
from numpy.linalg import norm
class TestCharacter(CharacterEntity):
    move_count = 0
    path_plan = True 
    called_special_move = False
    path = []
    """ 
    depth can be set at 3 to win 100% of the time with variant 2 
    with a depth of 2 it fails once with the seed set at 234
    """
    depth = 4
    bomb_location = None
    bomb_timer = 11
    GAMMA = 0.8
    COST_OF_LIVING = -2
    ALPHA = 0.01
    WEIGHT_INDEX = 0
    need_weight_index = True
    def do(self, wrld):
        self.check_bomb()
        # print(self.bomb_location)
        self.get_index()
        
        if(wrld.scores['me'] == -4999):
            self.get_weights()
            print("w for feature 1 = ", self.weights[0])
            print("w for feature 2 = ", self.weights[1])
            print("w for feature 3 = ", self.weights[2])
            print("w for feature 4 = ", self.weights[3])
            print("w for feature 5 = ", self.weights[4])
            print("w for feature 6 = ", self.weights[5])
            print("w for feature 7 = ", self.weights[6])
            print("w for feature 8 = ", self.weights[7])
    
        
        # print(self.path)
        self.set_features(wrld)
        # print('features', self.features)

        current_q = self.calc_Q(self.features)
        move, q = self.update_weights(wrld, current_q)

        # print('move', move, 'q value', q)
        # print('post weights', self.weights)

        # if self.get_Gn(wrld.exitcell,(self.x,self.y)) <= 2:
        #     self.update_weights_in_csv(self.weights)

        if((move[0] == 0) & (move[1] == 0)):
            
            self.drop_bomb()
        else:
            self.move(move[0], move[1])
        


    def set_features(self, wrld):
        self.features = self.get_features(wrld, (self.x, self.y))

    def get_features(self, wrld, position):
        monst_path = self.get_nearest_monst_path(wrld,position)
        exit_path,distance = self.get_exit_path(wrld,position)
        print('exit path:', exit_path) 
        #  f1 = a_star distance to exit
        f1 = 1/(1+distance)
        #  f2 = a_star distance to closest monster
        f2 = len(monst_path)
        # f3 = should we drop bomb  
        f3 = self.should_drop_bomb(wrld, position,distance)
        # f4 = distance to bomb
        f4 = self.get_bomb_distance(position)
        # f5 = manhattan distance to exit
        f5 =1/(1+ self.get_Gn(position,wrld.exitcell))
        dist_r = self.get_wall_distance(wrld, position,1)
        dist_l = self.get_wall_distance(wrld, position,3)
        diff_l_r=abs(dist_r-dist_l)
        # f6 = difference between the closest wall on the left and the closest wall on the right
        f6 = 1/(1+diff_l_r)
        # f7 = difference between the closest wall on the left and the closest wall on the right
        dist_b = self.get_wall_distance(wrld, position,2)
        dist_u = self.get_wall_distance(wrld, position,4)
        diff_b_u=abs(dist_b-dist_u)
        
        # f8 = manhattan distance to closet wall on top
        f7 = 1/(1+diff_b_u)#self.get_wall_distance(wrld, position,4)
        # f8 = cosine similarity from path to monster and path to exit
        if monst_path == [] or exit_path == []:
            f8=-1
        else:
            f8 = self.get_cos_sim(wrld,monst_path,exit_path)
        return [f1, f2, f3, f4, f5, f6, f7, f8]
    
        
    def best_Q(self, wrld):
        best_q = -math.inf
        best_move = (self.x, self.y)
        neighbors = self.get_possible_moves(wrld, (self.x, self.y), True, False)
        print("neighbors ", neighbors)
        # use 0,0 move to represent placing bomb
        neighbors.append((self.x, self.y))

        for move in neighbors:
            print("getting features")
            print('check', move)
            features = self.get_features(wrld, move)
            print(features)
            q = self.calc_Q(features)
            
            # print('check', move, q)

            if(q > best_q):
                best_q = q
                best_move = move
        print("best move", best_move, " best q", best_q)
        return best_q, best_move

    def get_q_reward(self,wrld, best_move):
        if best_move ==(7,18):
            return 100
        elif self.blast_radius(self.bomb_location,best_move):
            return -500
        elif wrld.monsters_at(best_move[0],best_move[1]):
            return -500
        else:
            exit_reward = (self.get_Gn(best_move,wrld.exitcell))
            return self.COST_OF_LIVING - exit_reward
    
    def calc_Q(self, features):
        q = 0
        for i in range(len(features)):
            q += (self.weights[i]*features[i])
        return q
    
    def should_drop_bomb(self, wrld, position,distance):
        dx, dy = self.extract_move(position)

        print("dx,dy", (dx,dy))
        print(dx ==0 and dy==0 and not(self.bomb_location) and distance ==0)
        if dx ==0 and dy==0 and not(self.bomb_location):
            return 1
        else: 
            return 0

    def get_exit_path(self,wrld,position): 
        goals = [wrld.exitcell,(7,14), (7,10),(7,6),(7,2)]
        for i in range(len(goals)):
            print("trying ", goals[i])
            if goals[i]==(self.x,self.y):
                return [],0
            came_from, cost_incurred = self.A_star(wrld,(self.x,self.y),goals[i],False)
    
            path = self.get_path((self.x,self.y),came_from, goals[i])
            if path != []:
                print("GOAL = ", goals[i])
                distance = self.get_Gn(position,path[0])
                return path,distance
        else:
            print("NO GOAL FOUND")
            return [], 0
            
    def get_nearest_monst_path(self,wrld,position):
        monsters = self.get_monster_position(wrld)
        # print("monsters = ", monsters)

        if monsters != []:


            if len(monsters) ==1:
                closest_monster = monsters[0]
            else:
                monst1_dist = self.get_Gn(position,monsters[0])
                monst2_dist = self.get_Gn(position,monsters[1])
                if monst1_dist < monst2_dist:
                    closest_monster = monsters[0]
                else:
                    closest_monster = monsters[1]
            came_from, cost_incurred = self.A_star(wrld,position,closest_monster,True)
            
            path = self.get_path(position,came_from, closest_monster)
          
            # print(path)
            return path
        else:
            return []
    
    def get_wall_distance(self,wrld, position, move_direction):
        # print(position)
        if move_direction==1:
            for i in range(0, wrld.width() - position[0],1):
                x = (position[0]+i)
                y = position[1]

                if wrld.wall_at(x,y):
                    return i
            else:
                return wrld.width()-position[0]
        if move_direction==2:
            for i in range(0, wrld.height() - position[1], 1):
                x = position[0]
                y = (position[1]+i)

                if wrld.wall_at(x,y):
                    return i
            else:
                return wrld.height()-position[1]
        if move_direction==3:
            for i in range(position[0], -1, -1):
                x = i
                y = position[1]

                if wrld.wall_at(x,y):
                    return position[0]-i
            else:
                return position[0] + 1
        if move_direction==4:
            for i in range(position[1], -1, -1):
                x= position[0]
                y= i

                if wrld.wall_at(x,y):
                    return position[1]-i
            else: 
                return position[1] + 1

                
    def get_bomb_distance(self, position):
            if(self.bomb_location):
                return self.get_Hn(self.bomb_location,position)
                # return 1.0 / (self.get_Gn(self.bomb_location,position) + 1)
            else:
                return 0
    def get_cos_sim(self,wrld,monst_path,exit_path):
        
        mdx,mdy = self.extract_move(monst_path[0])
        edx,edy = self.extract_move(exit_path[0])    
        if (mdx,mdy)==(0,0) or (edx,edy)==(0,0):
            return 0
        v_m= np.array([mdx,mdy])
        v_e=np.array([edx,edy])
        # print("v_m = ", v_m)
        # print("v_e = ", v_e)
        cosine =np.dot(v_m,v_e)/(norm(v_m)*norm(v_e))#,4

        print("Cosine Similarity:", cosine)
        return cosine
    
    def get_monster_position(self, wrld):
        monsters = []
        for x in range(wrld.width()):
            for y in range(wrld.height()):
                if(wrld.monsters_at(x, y)):
                    if not(self.is_behind_wall(wrld,(x,y))):
                        monsters.append((x, y))
        return monsters
    """
    checks if a point of interest is behind a wall.
    points of interest are the exit or a monster
    """
    def is_behind_wall(self, wrld, check_loc):
        
        if self.y < 2 < check_loc[1] < 7:
            return self.check_wall(wrld,3)
        elif self.y < 6< check_loc[1] < 11:
            return self.check_wall(wrld,7)
        elif self.y < 10 < check_loc[1] < 15:
            return self.check_wall(wrld,11)
        elif self.y < 14< check_loc[1] < 19:
            return self.check_wall(wrld,15)
        else: 
            return False
        
    def check_wall(self,wrld,y):
        walls_intact = 0
        for x in range(wrld.width()):
            if wrld.wall_at(x,y):
                walls_intact +=1
        if walls_intact ==8:
            return True
        else:
            return False

   
    def update_weights(self, wrld, q_current):
        max_q, best_move= self.best_Q(wrld)
        reward = self.get_q_reward(wrld,best_move)
        print('reward:', reward)
        delta = (reward + (self.GAMMA*max_q)) - q_current
        print('delta:', delta)
        new_weights = []
        for i in range(len(self.weights)):
            w_i = self.weights[i]+(self.ALPHA*delta*self.features[i])
            new_weights.append(w_i)
        # self.weights = new_weights
        #print('weights', self.weights)
        return ((best_move[0] - self.x), (best_move[1] - self.y)), max_q

    def get_index(self):
        if self.need_weight_index:
            indexes = pd.read_csv('index.csv')
            self.WEIGHT_INDEX = indexes["index"][0]
        

    def get_weights(self):
        weights = pd.read_csv('weights.csv')
  
        w_1 = weights["dist_exit"][self.WEIGHT_INDEX]
        w_2 = weights["dist_monst"][self.WEIGHT_INDEX]
        w_3 = weights["drop_bomb"][self.WEIGHT_INDEX]
        w_4 = weights["dist_bomb"][self.WEIGHT_INDEX]
        w_5 = weights["manhat_exit"][self.WEIGHT_INDEX]
        w_6 = weights["diff_l_r_walls"][self.WEIGHT_INDEX]
        w_7 = weights["diff_b_u_walls"][self.WEIGHT_INDEX]
        w_8 = weights["cos_sim"][self.WEIGHT_INDEX]
        self.weights = [w_1,w_2,w_3,w_4,w_5,w_6,w_7,w_8]

    def update_weights_in_csv(self, w1):
        with open("weights.csv", 'a') as csvfile:
            new_weights = w1
            updater = csv.writer(csvfile)
            updater.writerow(new_weights)
            csvfile.close()

    def drop_bomb(self):
        if(not self.bomb_location) and self.bomb_timer==11:
            self.place_bomb()
            self.bomb_location = (self.x, self.y)
            # self.update_weights_in_csv(self.weights)

    def blast_radius(self, bomb_pos, move):
        if(not bomb_pos):
            return False
        for dx in range(-5, 5, 1):
            radius = (bomb_pos[0] + dx, bomb_pos[1])
            if ((move[0] == radius[0]) & (move[1] == radius[1])):
                return True
        for dy in range(-5, 5, 1):  
            radius = (bomb_pos[0], bomb_pos[1] + dy)   
            if ((move[0] == radius[0]) & (move[1] == radius[1])):
                return True
        return False
    
    def check_bomb(self):
        if(self.bomb_location):
                self.bomb_timer -= 1
        if self.bomb_timer == 0:
            self.bomb_location = None
            self.bomb_timer = 11

    def A_star(self,wrld, start, goal,toM):
      
        frontier = PriorityQueue()
        frontier.put(start,0)
        came_from = {}  
        cost_incurred = {}  
        came_from[start] = None
        cost_incurred[start]=0

        while not frontier.empty():
            current  = frontier.get()
            
            if current == goal:
                break
            
            for next in self.get_possible_moves(wrld,current, True,toM):
                new_cost = cost_incurred[current] + self.get_Gn(current,next)
                
                if next not in cost_incurred or new_cost < cost_incurred[next]:
                    cost_incurred[next]= new_cost
                    priority = new_cost + self.get_Hn( next,goal)
                    frontier.put(next, priority)
                    came_from[next] = current
                    
        return came_from, cost_incurred
    
    def get_possible_moves(self,wrld, loc, isC,toM):
        neighbors = []   

        for dx in [-1, 0, 1]:
            # Avoid out-of-bounds access
            if ((loc[0] + dx >= 0) and (loc[0]  + dx < wrld.width())):
                for dy in [-1, 0, 1]:
                    # Avoid out-of-bounds access
                    if ((loc[1] + dy >= 0) and (loc[1] + dy < wrld.height())):
                        x= loc[0]+dx
                        y = loc[1] + dy
                        if isC:
                            if toM:
                                if((wrld.exit_at(x,y) or not(wrld.wall_at(x,y)))
                                and (not self.blast_radius(self.bomb_location, (x,y)))):
                                    if(x,y) == (0,15):
                                        self.set_cell_color(x, y,Fore.RED + Back.GREEN)
                                    neighbors.append((x, y))
                                else:
                                    continue
                            else:

                                if(wrld.exit_at(x, y) or (not(wrld.monsters_at(x, y)) and not(wrld.wall_at(x,y))
                                and (not self.blast_radius(self.bomb_location, (x, y))))):
                                    if((x, y)) == (0,15):
                                        self.set_cell_color((x), (y),Fore.RED + Back.GREEN)
                                    neighbors.append((x, y))
                                else:
                                    continue
                        else:
                            if((not wrld.wall_at(x, y)) and
                                ((x, y)!= (loc[0] ,loc[1]))):
                                        neighbors.append((x, y))

        return neighbors

    def extract_move(self, next):
            return next[0]-self.x, next[1]-self.y

     # the G(n) is the manhattan distance or right angle distance distance
    def get_Gn(self, current, next):
        return abs(current[0]-next[0]) + abs(current[1]-next[1])

    # the h(n) is the euclidean_distance or straight line distance
    def get_Hn(self, next, goal ):
        return math.floor(math.sqrt(pow((goal[0]-next[0]), 2) + pow((goal[1]-next[1]), 2)))


    def get_path(self,start, came_from, goal):
        current = goal
        path=[]
        if goal not in came_from:
            return []
        while current != start:
            self.set_cell_color(current[0], current[1],Fore.RED + Back.GREEN)
            path.append(current)
            current= came_from[current]
        path.reverse()
        return path
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
    bomb_timer = 12
    GAMMA = 0.8
    COST_OF_LIVING = -1
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
            print("w for feature 9 = ", self.weights[8])
    
        
        # print(self.path)
        self.set_features(wrld)
        # print('features', self.features)

        current_q = self.calc_Q(self.features)
        move, q = self.update_weights(wrld, current_q)

        # print('move', move, 'q value', q)
        # print('post weights', self.weights)

        # if self.get_Gn(wrld.exitcell,(self.x,self.y)) <= 2:
        #     self.update_weights_in_csv(self.weights)
        # print("move = ", move)
        # print(((move[0] == 0) & (move[1] == 0)))
        if((move[0] == 0) & (move[1] == 0)):
            
            self.drop_bomb()
        else:
        # self.set_cell_color(self.path[self.move_count][0], self.path[self.move_count][1], Back.RED)
            self.move(move[0], move[1])
        


    def set_features(self, wrld):
        self.features = self.get_features(wrld, (self.x, self.y))

    def get_features(self, wrld, position):
        monst_path = self.get_nearest_monst_path(wrld,position)
        exit_path = self.get_exit_path(wrld,position)
        print('exit path:', exit_path) 
        #  f1 = a_star distance to exit
        f1 = 1/(1+len(exit_path))
        #  f2 = a_star distance to closest monster
        f2 = 1/(1+len(monst_path))
        # f3 = should we drop bomb  
        f3 = self.should_drop_bomb(wrld, position)
        # f4 = distance to bomb
        f4 = self.get_bomb_distance(position)
        # f5 = manhattan distance to closet wall on right
        f5 = 1/(1+self.get_wall_distance(wrld, position,1))
        # f6 = manhattan distance to closet wall on bottom
        f6 = 1/(1+ self.get_wall_distance(wrld, position,2))
        # f7 = manhattan distance to closet wall on left
        f7 = 1/(1+self.get_wall_distance(wrld, position,3))
        # f8 = manhattan distance to closet wall on top
        f8 = 1/(1+self.get_wall_distance(wrld, position,4))
        # f8 = cosine similarity from path to monster and path to exit
        if monst_path == [] or exit_path == []:
            f9=0
        else:
            f9 = - self.get_cos_sim(wrld,monst_path,exit_path)
        return [f1, f2, f3, f4, f5, f6, f7, f8, f9]
    
        
    def best_Q(self, wrld):
        best_q = -math.inf
        best_move = (self.x, self.y)
        neighbors = self.get_possible_moves(wrld, (self.x, self.y), True, False)
        # use 0,0 move to represent placing bomb
        neighbors.append((self.x, self.y))

        for move in neighbors:
            print("getting features")
            features = self.get_features(wrld, move)
            print(features)
            q = self.calc_Q(features)
            
            print('check', move, q)

            if(q > best_q):
                best_q = q
                best_move = move
        #print("best move", best_move, " best q", best_q)
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
        # print(features)
        q = 0
        for i in range(len(features)):
            q += (self.weights[i]*features[i])
        # q = (self.weights[0] * features[0]) + (self.weights[1] * features[1]) + (self.weights[2] * features[2]) + (self.weights[3] * features[3])+ (self.weights[4] * features[4])
        # print(q)
        return q
    
    def should_drop_bomb(self, wrld, position):
        dx, dy = self.extract_move(position)
        goal = self.find_next_best(wrld,(self.x,self.y))
        if goal==(self.x,self.y) and not(self.bomb_location):
            return 0.5
        else: 
            return 0
        # if(self.get_bomb_distance((self.x, self.y)) != 0):
        #     return 0
        # else:
        #     #print(float(self.get_exit_distance(wrld, (self.x, self.y)))**3)
        #     return  (2* float(self.get_exit_distance(wrld, position))) / (float(self.get_monster_distance(wrld)) + 1)
      
    def get_exit_path(self,wrld,position): 
        goal = self.find_next_best(wrld,(self.x,self.y))
        if goal==(self.x,self.y):
            return []
        else:
            print("goal", goal)
            came_from, cost_incurred = self.A_star(wrld,position,goal,False)
            #print('came from', came_from)
            path = self.get_path(position,came_from, goal)

            return path

    def find_next_best(self, wrld,position):
        if position[1]>15:
            return wrld.exitcell
        if position[1]>11:
            if self.is_behind_wall(wrld,wrld.exitcell) :
                return (7,14)
            else: 
                return wrld.exitcell
        if position[1]>7:
            if self.is_behind_wall(wrld,(7,14)) :
                return (7,10)
            else: 
                return (7,14)
        if position[1]>3:
            if self.is_behind_wall(wrld,(7,10)) :
                return (7,6)
            else: 
                return (7,10)
        else:
            if self.is_behind_wall(wrld,(7,6)) :
                return (7,2)
            else: 
                return (7,6)
            
    def get_nearest_monst_path(self,wrld,position):
        monsters = self.get_monster_position(wrld)
        print("monsters = ", monsters)
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
            print(path)
            return path
        else:
            return []
    
    def get_wall_distance(self,wrld, position, move_direction):
        print(position)
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
        print("v_m = ", v_m)
        print("v_e = ", v_e)
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
        self.weights = new_weights
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
        w_5 = weights["dist_r_wall"][self.WEIGHT_INDEX]
        w_6 = weights["dist_b_wall"][self.WEIGHT_INDEX]
        w_7 = weights["dist_l_wall"][self.WEIGHT_INDEX]
        w_8 = weights["dist_t_wall"][self.WEIGHT_INDEX]
        w_9 = weights["cos_sim"][self.WEIGHT_INDEX]
        self.weights = [w_1,w_2,w_3,w_4,w_5,w_6,w_7,w_8,w_9]

    def update_weights_in_csv(self, w1):
        with open("weights.csv", 'a') as csvfile:
            new_weights = w1
            updater = csv.writer(csvfile)
            updater.writerow(new_weights)
            csvfile.close()

    def drop_bomb(self):
        if(not self.bomb_location) and self.bomb_timer==12:
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
            self.bomb_timer = 12
            
        # print('bomb_timer', self.bomb_timer,)
            
    # def run_expectimax(self,wrld,mnstr_loc):
    #     potential_moves =self.build_tree(wrld,mnstr_loc)
    #     highest_value = -math.inf
    #     move_to_take = (0,0)
    #     for aMove in potential_moves:
    #         if aMove[1]>highest_value:
    #             move_to_take = aMove[0]
    #             highest_value = aMove[1]
    #     dx, dy = self.extract_move(move_to_take)
    #     return (dx, dy)
    
    # def run_minimax(self, wrld, mnstr_loc):

    #     char_potential_moves = self.get_possible_moves(wrld, (self.x, self.y), True)
    #     mnstr_potential_moves = self.get_possible_moves(wrld, mnstr_loc, False)

    #     move_to_take = (0,0)
    #     max_value = -math.inf

    #     for move in char_potential_moves:
    #         min_value = math.inf

    #         for move2 in mnstr_potential_moves:
    #             move_value = self.get_Hn(move, move2)

    #             if move_value < min_value:
    #                 min_value = move_value
            
    #         if min_value > max_value:
    #             max_value = min_value
    #             move_to_take = move

    #     dx, dy = (move_to_take[0] - self.x, move_to_take[1] - self.y)
    #     return move_to_take

    # def build_tree(self,wrld,mnstr_loc):
    #     chtr_loc = (self.x,self.y)
    #     #chance_node list of tuples (action, expected value)
    #     chance_node: list [tuple(tuple, float)] = []

    #     pot_chtr_moves = self.get_possible_moves(wrld,chtr_loc, True)
    #     orig_dist_exit =  self.get_Gn(wrld.exitcell,chtr_loc)
    #     for Cmove in pot_chtr_moves:
    #         utilities = self.build_T_nodes(wrld,mnstr_loc, Cmove,orig_dist_exit)
    #         probability = (1/len(utilities))
    #         expectedValue= 0
    #         for aReward in utilities:

    #             expectedValue = expectedValue + (aReward*probability)

    #         chance_node.append((Cmove,expectedValue))
    #     return chance_node

    # def build_T_nodes(self,wrld,mnstr_loc, Cmove, orig_dist_exit):
        
    #     utilities = []
    
    #     new_dist_exit = self.get_Gn(wrld.exitcell,Cmove)
    #     move_penalty = (orig_dist_exit-new_dist_exit)*.5
    #     pot_mnstr_moves = self.get_possible_moves(wrld,mnstr_loc,False)

    #     for Mmove in pot_mnstr_moves:
    #         #M_loc_moved = (mnstr_loc[0]+Mmove[0],mnstr_loc[1]+Mmove[1])
    #         reward = self.get_reward(Cmove, Mmove)
            
    #         utilities.append((reward+move_penalty))
    #     return utilities

    def A_star(self,wrld, start, goal,toM):
      
        frontier = PriorityQueue()
        frontier.put(start,0)
        came_from = {}  
        cost_incurred = {}  
        came_from[start] = None
        cost_incurred[start]=0
        # print("IN A STAR")
        # print(wrld.exitcell)
        # print(wrld.empty_at(0,15))
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
                        
                        if isC:
                            if toM:
                                if((wrld.exit_at(loc[0]  + dx, loc[1] + dy) or not(wrld.wall_at(loc[0]  + dx, loc[1] + dy)))
                                and (not self.blast_radius(self.bomb_location, (loc[0] + dx, loc[1] + dy)))):
                                    if((loc[0]  + dx, loc[1] + dy)) == (0,15):
                                        self.set_cell_color((loc[0] + dx), (loc[1] + dy),Fore.RED + Back.GREEN)
                                    neighbors.append((loc[0] + dx, loc[1] + dy))
                                else:
                                    continue
                            else:
                                if((wrld.exit_at(loc[0]  + dx, loc[1] + dy) or wrld.empty_at(loc[0]  + dx, loc[1] + dy))
                                and (not self.blast_radius(self.bomb_location, (loc[0] + dx, loc[1] + dy)))):
                                    if((loc[0]  + dx, loc[1] + dy)) == (0,15):
                                        self.set_cell_color((loc[0] + dx), (loc[1] + dy),Fore.RED + Back.GREEN)
                                    neighbors.append((loc[0] + dx, loc[1] + dy))
                                else:
                                    continue
                        else:
                            if((not wrld.wall_at(loc[0]  + dx, loc[1] + dy)) and
                                ((loc[0]  + dx, loc[1] + dy)!= (loc[0] ,loc[1]))):
                                        neighbors.append((loc[0] + dx, loc[1] + dy))

        return neighbors
    # def get_reward(self, C_loc_moved, M_loc_moved):
    #     manhat_dist = self.get_Gn(C_loc_moved,M_loc_moved)

    #     match manhat_dist:
    #         case 0:
    #             return int(-100)
    #         case 1:
    #             return int(-20)
    #         case 2:
    #             return int(-2)
    #         case 3:
    #             return int(4)
    #         case 4:
    #             return int(8)
    #         case 5:
    #             return int(24)

    #     return int(0)
                

    def extract_move(self, next):
            return next[0]-self.x, next[1]-self.y

     # the G(n) is the manhattan distance or right angle distance distance
    def get_Gn(self, current, next):
        return abs(current[0]-next[0]) + abs(current[1]-next[1])

    # the h(n) is the euclidean_distance or straight line distance
    def get_Hn(self, next, goal ):
        return math.floor(math.sqrt(pow((goal[0]-next[0]), 2) + pow((goal[1]-next[1]), 2)))

    """
    Checks if a monster is in a given proximity to the character. If the monster
    is in proximity, return location of the monster. Else, return False
    """
    def is_monster_in_proximity(self,wrld):
        for dx in range(-self.depth, self.depth + 1, 1):
            # Avoid out-of-bounds access
            if ((self.x + dx >= 0) and (self.x + dx < wrld.width())):
                for dy in range(-self.depth, self.depth + 1, 1):
                    # Avoid out-of-bounds access
                    if ((self.y + dy >= 0) and (self.y + dy < wrld.height())):
                        
                        monster_info = wrld.monsters_at(self.x + dx, self.y + dy)
                        if monster_info:
                            if self.get_Gn((self.x,self.y),(self.x + dx, self.y + dy)) <= 2:
                                # print("changed strategy for safety")
                                return ((True,"aggressive"), (self.x + dx, self.y + dy))
                            return ((True,monster_info[0].name), (self.x + dx, self.y + dy))
        return ((False, "empty"),(0,0))       


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
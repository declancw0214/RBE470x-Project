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
class TestCharacter(CharacterEntity):
    move_count = 0
    path_plan = True 
    called_special_move = False
    path = []
    """ 
    depth can be set at 3 to win 100% of the time with variant 2 
    with a depth of 2 it fails once with the seed set at 234
    """
    depth = 3
    bomb_location = None
    bomb_timer = 5 
    GAMMA = 0.8
    COST_OF_LIVING = -1
    ALPHA = 0.01
    WEIGHT_INDEX = 0
    need_weight_index = True
    def do(self, wrld):
        self.get_index()
        if(wrld.scores['me'] == -4999):
            self.get_weights()
            print("weight for feature 1 = ", self.weights[0])
            print("weight for feature 2 = ", self.weights[1])
            print("weight for feature 3 = ", self.weights[2])
            print("weight for feature 4 = ", self.weights[3])
            print("weight for feature 5 = ", self.weights[4])
    
        came_from, cost_incurred = self.A_star(wrld)
        self.path = self.get_path(came_from, wrld)
        print(self.path)
        self.set_features(wrld)
        # print('features', self.features)

        current_q = self.calc_Q(self.features)
        move, q = self.update_weights(wrld, current_q)

        # print('move', move, 'q value', q)
        # print('post weights', self.weights)

        if self.get_Gn(wrld.exitcell,(self.x,self.y)) <= 2:
            self.update_weights_in_csv(self.weights)
        print("move = ", move)
        print(((move[0] == 0) & (move[1] == 0)))
        if((move[0] == 0) & (move[1] == 0)):
            
            self.drop_bomb()
            self.update_weights_in_csv(self.weights)
        else:
        # self.set_cell_color(self.path[self.move_count][0], self.path[self.move_count][1], Back.RED)
            self.move(move[0], move[1])
        self.is_monster_in_proximity(wrld)
        # self.state_selector(monster_prox)
        # print(self.state)
        # match self.state:
            
        #     case "move":
              
        #         if self.called_special_move:
        #             print("stop moving")
        #             self.move(0, 0)
        #             self.path_plan = True
        #             self.called_special_move = False
        #         else: 
        #             print("move")
        #             print('move count', self.move_count)
        #             dx, dy = self.extract_move(self.path[self.move_count])
        #             self.set_cell_color(self.path[self.move_count][0], self.path[self.move_count][1], Back.RED)
        #             self.move(dx, dy)
        #             self.move_count+=1

        #     case "expectimax":
                    
        #         best_move = self.run_expectimax(wrld,monster_prox[1])
        #         self.path.clear()
                
        #         self.move(best_move[0],best_move[1])

        #         self.move_count = 0
        #         self.path_plan = True

        #     case "a_star":
        #         self.called_special_move = False
        #         came_from, cost_incurred = self.A_star(wrld)
        #         self.path = self.get_path(came_from, wrld)

        #         # bomb explosion was causing no path to goal
        #         # if no path rerun a_star next turn
        #         if(len(self.path) == 0):
        #             self.path_plan = True
        #         else:
        #             dx, dy = self.extract_move(self.path[self.move_count])
        #             self.set_cell_color(self.path[self.move_count][0], self.path[self.move_count][1], Back.RED)
        #             self.move(dx, dy)
        #             self.move_count+=1
        #             self.path_plan = False 
                

        #     case "minimax":
        #         best_move = self.run_minimax(wrld,monster_prox[1])
        #         self.path.clear()
        #         self.drop_bomb()
                
        #         if(not self.blast_radius(self.bomb_location, best_move)):
        #             print("blast radius failure")
        #             self.move(best_move[0],best_move[1])

        #         self.move_count = 0
        #         self.path_plan = True

        self.check_bomb()

    def set_features(self, wrld):
        self.features = self.get_features(wrld, (self.x, self.y))

    def get_features(self, wrld, position):
        #  f1 = distance from a star path
        f1 = self.get_from_A_star(position)

        
        
        # # f1 = distance to monster
        # f1 = self.get_monster_distance(wrld)
        # f2 = distance to exit
        f2 = self.get_exit_distance(wrld, position)
        # f3 = distance to bomb
        f3 = self.get_bomb_distance(position)
        # f4 = should we place bomb
        f4 = self.should_drop_bomb(wrld, position)
        # f5 = distance from monster
        f5 = self.get_monster_distance(wrld)
        return [f1, f2, f3, f4, f5]
    
        
    def best_Q(self, wrld):
        best_q = -math.inf
        best_move = (self.x, self.y)
        neighbors = self.get_possible_moves(wrld, (self.x, self.y), True)
        # use 0,0 move to represent placing bomb
        neighbors.append((self.x, self.y))

        for move in neighbors:
            features = self.get_features(wrld, move)
            reward = self.get_q_reward(features)
            q = self.calc_Q(features)
            
            print('check', move, q)

            if(q > best_q):
                best_q = q
                best_move = move
        print("best move = ", best_move, " best q", best_q)
        return best_q, best_move, reward

    def get_q_reward(self, features):
        rewards = 0
        for i in range(len(features)):
            rewards += features[i]
        return rewards
    def calc_Q(self, features):
        print(features)
        q = (self.weights[0] * features[0]) + (self.weights[1] * features[1]) + (self.weights[2] * features[2]) + (self.weights[3] * features[3])
        # print(q)
        return q
    
    def should_drop_bomb(self, wrld, position):
        dx, dy = self.extract_move(position)
        if self.path == [] and dx == 0 and dy ==0:
            return 1
        else: 
            return 0
        # if(self.get_bomb_distance((self.x, self.y)) != 0):
        #     return 0
        # else:
        #     #print(float(self.get_exit_distance(wrld, (self.x, self.y)))**3)
        #     return  (2* float(self.get_exit_distance(wrld, position))) / (float(self.get_monster_distance(wrld)) + 1)
        
    def get_from_A_star(self, move):
        print(move)
        if self.path == []:
            return 0
        else:
            a_star_move = self.path[0]
            distance = self.get_Gn(a_star_move,move)
            return (50/(math.e ** distance))
    
    def get_exit_distance(self, wrld, position):
        distance = self.get_Gn(position,wrld.exitcell)
        return (25/(math.e ** distance))
    
    def get_bomb_distance(self, position):
            if(self.bomb_location):
                return self.get_Gn(position,self.bomb_location)
                # return 1.0 / (self.get_Hn(position, self.bomb_location) + 1)
            else:
                return 0
    
    def get_monster_distance(self, wrld):
        monster_position = self.get_monster_position(wrld)
        if(monster_position != 0):
            return self.get_Gn((self.x, self.y), monster_position)
        else:
            return 0

    def get_monster_position(self, wrld):
        for x in range(wrld.width()):
            for y in range(wrld.height()):
                if(wrld.monsters_at(x, y)):
                    return x, y
        return 0


    def update_weights(self, wrld, q_current):
        max_a, move, reward = self.best_Q(wrld)

        delta = (reward + (self.GAMMA*max_a)) - q_current
        new_weights = []
        for i in range(len(self.weights)):
            w_i = self.weights[i]+(self.ALPHA*delta*self.features[i])
            new_weights.append(w_i)
        self.weights = new_weights
        direction = (())
        return ((move[0] - self.x), (move[1] - self.y)), max_a

    def get_index(self):
        if self.need_weight_index:
            indexes = pd.read_csv('index.csv')
            self.WEIGHT_INDEX = indexes["index"][0]
        

    def get_weights(self):
        weights = pd.read_csv('weights.csv')
  
        weight_1 = weights["weight1"][self.WEIGHT_INDEX]
        weight_2 = weights["weight2"][self.WEIGHT_INDEX]
        weight_3 = weights["weight3"][self.WEIGHT_INDEX]
        weight_4 = weights["weight4"][self.WEIGHT_INDEX]
        weight_5 = weights["weight5"][self.WEIGHT_INDEX]
        self.weights = [weight_1, weight_2, weight_3, weight_4, weight_5]

    def update_weights_in_csv(self, w1):
        with open("weights.csv", 'a') as csvfile:
            new_weights = w1
            updater = csv.writer(csvfile)
            updater.writerow(new_weights)
            csvfile.close()

    def state_selector(self, monster_prox):
        is_near_monster = monster_prox[0][0]
        monster_type = monster_prox[0][1]

        if self.bomb_timer !=5:
                self.state = "minimax"
                return 
        if self.called_special_move:
                self.state = "move"
                return
        if is_near_monster:
            if monster_type == "stupid":
                self.state = "expectimax"
                return
            else:
                self.state = "minimax"
                return 
        elif self.path_plan:
            self.state = "a_star"
            return
        else: 
            self.state = "move"
            return


    def drop_bomb(self):
        if(not self.bomb_location):
            self.place_bomb()
            self.bomb_location = (self.x, self.y)

    def blast_radius(self, bomb_pos, move):
        if(not bomb_pos):
            return False
        for dx in range(-4, 5, 1):
            radius = (bomb_pos[0] + dx, bomb_pos[1])
            if ((move[0] == radius[0]) & (move[1] == radius[1])):
                return True
        for dy in range(-4, 5, 1):  
            radius = (bomb_pos[0], bomb_pos[1] + dy)   
            if ((move[0] == radius[0]) & (move[1] == radius[1])):
                return True
        return False
    
    def check_bomb(self):
        if(self.bomb_location):
            self.bomb_timer -= 1
        if self.bomb_timer == 0:
            self.bomb_location = None
            self.bomb_timer = 5
        print('bomb_timer', self.bomb_timer)

    def run_expectimax(self,wrld,mnstr_loc):
        potential_moves =self.build_tree(wrld,mnstr_loc)
        highest_value = -math.inf
        move_to_take = (0,0)
        for aMove in potential_moves:
            if aMove[1]>highest_value:
                move_to_take = aMove[0]
                highest_value = aMove[1]
        dx, dy = self.extract_move(move_to_take)
        return (dx, dy)
    
    def run_minimax(self, wrld, mnstr_loc):

        char_potential_moves = self.get_possible_moves(wrld, (self.x, self.y), True)
        mnstr_potential_moves = self.get_possible_moves(wrld, mnstr_loc, False)

        move_to_take = (0,0)
        max_value = -math.inf

        for move in char_potential_moves:
            min_value = math.inf

            for move2 in mnstr_potential_moves:
                move_value = self.get_Hn(move, move2)

                if move_value < min_value:
                    min_value = move_value
            
            if min_value > max_value:
                max_value = min_value
                move_to_take = move

        dx, dy = (move_to_take[0] - self.x, move_to_take[1] - self.y)
        return dx, dy

    def build_tree(self,wrld,mnstr_loc):
        chtr_loc = (self.x,self.y)
        #chance_node list of tuples (action, expected value)
        chance_node: list [tuple(tuple, float)] = []

        pot_chtr_moves = self.get_possible_moves(wrld,chtr_loc, True)
        orig_dist_exit =  self.get_Gn(wrld.exitcell,chtr_loc)
        for Cmove in pot_chtr_moves:
            utilities = self.build_T_nodes(wrld,mnstr_loc, Cmove,orig_dist_exit)
            probability = (1/len(utilities))
            expectedValue= 0
            for aReward in utilities:

                expectedValue = expectedValue + (aReward*probability)

            chance_node.append((Cmove,expectedValue))
        return chance_node

    def build_T_nodes(self,wrld,mnstr_loc, Cmove, orig_dist_exit):
        
        utilities = []
    
        new_dist_exit = self.get_Gn(wrld.exitcell,Cmove)
        move_penalty = (orig_dist_exit-new_dist_exit)*.5
        pot_mnstr_moves = self.get_possible_moves(wrld,mnstr_loc,False)

        for Mmove in pot_mnstr_moves:
            #M_loc_moved = (mnstr_loc[0]+Mmove[0],mnstr_loc[1]+Mmove[1])
            reward = self.get_reward(Cmove, Mmove)
            
            utilities.append((reward+move_penalty))
        return utilities

    def A_star(self,wrld):
        start = (self.x,self.y)
        goal = wrld.exitcell
        frontier = PriorityQueue()
        frontier.put(start,0)
        came_from = {}  
        cost_incurred = {}  
        came_from[start] = None
        cost_incurred[start]=0
        print("IN A STAR")
        print(wrld.exitcell)
        print(wrld.empty_at(0,15))
        while not frontier.empty():
            current  = frontier.get()
            
            if current == goal:
                break
            
            for next in self.get_possible_moves(wrld,current, True):
                new_cost = cost_incurred[current] + self.get_Gn(current,next)
                
                if next not in cost_incurred or new_cost < cost_incurred[next]:
                    cost_incurred[next]= new_cost
                    priority = new_cost + self.get_Hn(goal, next)
                    frontier.put(next, priority)
                    came_from[next] = current
                    
        return came_from, cost_incurred
    
    def get_possible_moves(self,wrld, loc, isC):
        neighbors = []   

        for dx in [-1, 0, 1]:
            # Avoid out-of-bounds access
            if ((loc[0] + dx >= 0) and (loc[0]  + dx < wrld.width())):
                for dy in [-1, 0, 1]:
                    # Avoid out-of-bounds access
                    if ((loc[1] + dy >= 0) and (loc[1] + dy < wrld.height())):
                        
                        if isC:
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
    def get_reward(self, C_loc_moved, M_loc_moved):
        manhat_dist = self.get_Gn(C_loc_moved,M_loc_moved)

        match manhat_dist:
            case 0:
                return int(-100)
            case 1:
                return int(-20)
            case 2:
                return int(-2)
            case 3:
                return int(4)
            case 4:
                return int(8)
            case 5:
                return int(24)

        return int(0)
                

    def extract_move(self, next):
            return next[0]-self.x, next[1]-self.y

     # the G(n) is the manhattan distance or right angle distance distance
    def get_Gn(self, current, next):
        return abs(current[0]-next[0]) + abs(current[1]-next[1])

    # the h(n) is the euclidean_distance or straight line distance
    def get_Hn(self, goal, next):
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
                                print("changed strategy for safety")
                                return ((True,"aggressive"), (self.x + dx, self.y + dy))
                            return ((True,monster_info[0].name), (self.x + dx, self.y + dy))
        return ((False, "empty"),(0,0))       


    def get_path(self, came_from, wrld):
        start = (self.x, self.y)
        goal = wrld.exitcell
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
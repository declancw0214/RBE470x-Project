# This is necessary to find the main code
from queue import PriorityQueue
import sys
sys.path.insert(0, '../bomberman')
# Import necessary stuff
from entity import CharacterEntity
from sensed_world import SensedWorld
from colorama import Fore, Back
import math

class TestCharacter(CharacterEntity):
    move_count = 0
    path_plan = True 
    path = []
    depth = 3
    bomb_position = None
    timer = 0

    def do(self, wrld):

        monster_prox = self.is_monster_in_proximity(wrld)

        if self.move_count == len(self.path):
            
            print('finished path')
            self.path_plan = True

        if  monster_prox[0] == True:

            print('monster in proximity')
            print(monster_prox)
            if(not self.bomb_position):
                self.place_bomb()
                self.bomb_position = self.x, self.y
            print('bomb: ', self.bomb_position)
            best_move = self.run_expectimax(wrld,monster_prox[1])
            self.path.clear()
            self.path.append(best_move)
            print('path: ', self.path)
            self.move_count = 0   

        elif self.path_plan:

            print('planning path')
            came_from, cost_incurred = self.A_star(wrld)
            self.path = self.get_path(came_from, wrld)
            self.path_plan = False
            self.move_count = 0

        if self.path != []:

            print('path not complete')
            dx, dy = self.extract_move(self.path[self.move_count])
            print(dx, dy)
            print('bomb: ', self.bomb_position)
            if(self.blast_radius(self.bomb_position, (self.x + dx, self.y + dy))):
                print('stopped')
                self.move(0, 0)
                
            else:
                
                print('move: ', dx, dy) 
                self.set_cell_color(self.path[self.move_count][0], self.path[self.move_count][1], Back.RED)
                self.move(dx, dy)
                self.move_count+=1
        
        print(self.timer)
        if(self.bomb_position):    
            self.timer += 1
        if self.timer >= 5:
            self.bomb_position = None
            self.timer = 0 

    def run_expectimax(self,wrld,mnstr_loc):

        potential_moves = self.build_tree(wrld,mnstr_loc, False)
        highest_value = -math.inf
        move_to_take = (0,0)
        for aMove in potential_moves:
            if aMove[1]>highest_value:
                highest_value = aMove[1]
                move_to_take = aMove[0]
        return move_to_take

    def build_tree(self,wrld,mnstr_loc, isMiniMax):
        chtr_loc = (self.x,self.y)
        #chance_node list of tuples (action, expected value)
        chance_node: list [tuple(tuple, float)] = []

        pot_chtr_moves = self.get_possible_moves(wrld,chtr_loc,False,True)

        for Cmove in pot_chtr_moves:
            if(not self.blast_radius(self.bomb_position, Cmove)):
                utilities = self.build_T_nodes(wrld,mnstr_loc, Cmove)

                #print(utilities)
                probability = ((1)/len(utilities))
                expectedValue= 0
                if(isMiniMax):
                    chance_node.append((Cmove, min(utilities)))
                else:
                    for aReward in utilities:
                        expectedValue += (aReward*probability)
                    chance_node.append((Cmove,expectedValue))
        return chance_node

    def build_T_nodes(self,wrld,mnstr_loc, Cmove):
        chtr_loc = (self.x,self.y)
        #C_loc_moved = (chtr_loc[0] + Cmove[0], chtr_loc[1]+Cmove[1])
        C_loc_moved = Cmove

        utilities = []
        start_distance =self.get_Hn(chtr_loc, mnstr_loc)

        pot_mnstr_moves = self.get_possible_moves(wrld,mnstr_loc,True,False)
        for Mmove in pot_mnstr_moves:
            M_loc_moved = (mnstr_loc[0]+Mmove[0],mnstr_loc[1]+Mmove[1])
            new_dist = self.get_Hn(C_loc_moved, M_loc_moved)
            if new_dist <= 2:
                utilities.append(-10)
            else:
                utilities.append(2*(new_dist-start_distance))
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

        while not frontier.empty():
            current  = frontier.get()
            
            if current == goal:
                break
            
            for next in self.get_possible_moves(wrld,current,False, True):
                new_cost = cost_incurred[current] + self.get_Gn(current,next)
                
                if next not in cost_incurred or new_cost < cost_incurred[next]:
                    cost_incurred[next]= new_cost
                    priority = new_cost + self.get_Hn(goal, next)
                    frontier.put(next, priority)
                    came_from[next] = current
                    
        return came_from, cost_incurred
    
    def get_possible_moves(self,wrld, loc, want_moves, isC):
        neighbors = []   

        for dx in [-1, 0, 1]:
            # Avoid out-of-bounds access
            if ((loc[0] + dx >= 0) and (loc[0]  + dx < wrld.width())):
                for dy in [-1, 0, 1]:
                    # Avoid out-of-bounds access
                    if ((loc[1] + dy >= 0) and (loc[1] + dy < wrld.height())):
                        if isC:
                            if((wrld.exit_at(loc[0]  + dx, loc[1] + dy) or
                                wrld.empty_at(loc[0]  + dx, loc[1] + dy)) and
                                ((loc[0]  + dx, loc[1] + dy)!= (loc[0] ,loc[1]))):
                                    if want_moves:
                                        neighbors.append((dx, dy))
                                    else:
                                        neighbors.append((loc[0] + dx, loc[1] + dy))
                            else:
                                continue
                        else:
                            if((not wrld.wall_at(loc[0]  + dx, loc[1] + dy)) and
                                ((loc[0]  + dx, loc[1] + dy)!= (loc[0] ,loc[1]))):
                                    if want_moves:
                                        neighbors.append((dx, dy))
                                    else:
                                        neighbors.append((loc[0] + dx, loc[1] + dy))

        return neighbors
    
    
    def extract_move(self, next):
        print('next: ', next)
        return next[0]-self.x, next[1]-self.y

     # the G(n) is the manhattan distance or right angle distance distance
    def get_Gn(self, current, next):
        return abs(current[0]-next[0]) + abs(current[1]-next[1])

    # the h(n) is the euclidean_distance or straight line distance
    def get_Hn(self, goal, next):
        return abs(pow((goal[0]-next[0]), 2) + pow((goal[1]-next[1]), 2))

    """
    Checks if a monster is in a given proximity to the character. If the monster
    is in proximity, return location of the monster. Else, return False
    """
    def is_monster_in_proximity(self,wrld):
        for dx in range(-self.depth,self.depth,1):
            # Avoid out-of-bounds access
            if ((self.x + dx >= 0) and (self.x + dx < wrld.width())):
                for dy in range(-self.depth,self.depth,1):
                    # Avoid out-of-bounds access
                    if ((self.y + dy >= 0) and (self.y + dy < wrld.height())):
                        if wrld.monsters_at(self.x + dx, self.y + dy):
                            return (True, (self.x + dx, self.y + dy))
        return (False,(0,0))       


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
    
    def blast_radius(self, bomb_pos, move):
        #print(bomb_pos)
        #print(move)
        if(not bomb_pos):
            return False
        for dx in range(-4, 4, 1):
            radius = (bomb_pos[0] + dx, bomb_pos[1])
            if ((move[0] == radius[0]) & (move[1] == radius[1])):
                return True
        for dy in range(-4, 4, 1):  
            radius = (bomb_pos[0], bomb_pos[1] + dy)   
            if ((move[0] == radius[0]) & (move[1] == radius[1])):
                return True
        return False

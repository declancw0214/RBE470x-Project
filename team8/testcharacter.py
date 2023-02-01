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
    path = []
    depth = 1
    def do(self, wrld):
        start = (self.x,self.y)
        monster_prox = self.is_monster_in_proximity(wrld)

        if  monster_prox[0] != False:
            self.path_plan = True
            print (monster_prox)
            best_move = self.run_expectimax(wrld,monster_prox[1])
            self.path.append((0,0))
            self.path.append(best_move)
            self.path.append((0,0))
            self.move_count = 0

        if self.path_plan:
            came_from, cost_incurred = self.A_star(wrld)
            self.path = self.get_path(came_from, wrld)
            self.path_plan = False

        dx, dy = self.extract_move(self.path[self.move_count])
        self.set_cell_color(self.path[self.move_count][0], self.path[self.move_count][1], Back.RED)
        self.move(dx, dy)
        self.move_count+=1

    def run_expectimax(self,wrld,mnstr_loc):
        potential_moves =self.build_tree(wrld,mnstr_loc)
        highest_value = -math.inf
        move_to_take = (0,0)
        for aMove in potential_moves:
            if aMove[1]>highest_value:
                move_to_take = aMove[0]
        return move_to_take


    def build_tree(self,wrld,mnstr_loc):
        chtr_loc = (self.x,self.y)
        #chance_node list of tuples (action, expected value)
        chance_node: list [tuple(tuple, float)] = []

        pot_chtr_moves = self.get_possible_moves(wrld,chtr_loc,True,True)

        for Cmove in pot_chtr_moves:
            utilities = self.build_T_nodes(wrld,mnstr_loc, Cmove)
            probability = (1/len(utilities))
            expectedValue= 0
            for aReward in utilities:
                expectedValue += (aReward*probability)
            chance_node.append((Cmove,expectedValue))
        return chance_node

    def build_T_nodes(self,wrld,mnstr_loc, Cmove):
        chtr_loc = (self.x,self.y)
        C_loc_moved = (chtr_loc[0] + Cmove[0], chtr_loc[1]+Cmove[1])

        utilities = []
        start_distance =self.get_Gn(chtr_loc, mnstr_loc)

        pot_mnstr_moves = self.get_possible_moves(wrld,mnstr_loc,True, False)
        for Mmove in pot_mnstr_moves:
            M_loc_moved = (mnstr_loc[0]+Mmove[0],mnstr_loc[1]+Mmove[1])
            new_dist = self.get_Hn(C_loc_moved, M_loc_moved)
            if new_dist == 0:
                utilities.append(-10)
            else:
                utilities.append((new_dist-start_distance))
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
    
    def get_possible_moves(self,wrld, loc, want_moves,isC):
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
                            if((not wrld.wall_at(loc[0]  + dx, loc[1] + dy)) and
                                ((loc[0]  + dx, loc[1] + dy)!= (loc[0] ,loc[1]))):
                                    if want_moves:
                                        neighbors.append((dx, dy))
                                    else:
                                        neighbors.append((loc[0] + dx, loc[1] + dy))

        return neighbors
    
    
    def extract_move(self, next):
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
                            return (True, (self.y + dx, self.y + dy))
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
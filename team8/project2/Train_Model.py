# This is necessary to find the main code
import sys
sys.path.insert(0, '../../bomberman')
sys.path.insert(1, '..')

from collections import deque
import time
# Import necessary stuff
from game import Game

# TODO This is your code!
sys.path.insert(1, '../team8')
from testcharacter import TestCharacter
from monsters.stupid_monster import StupidMonster
from monsters.selfpreserving_monster import SelfPreservingMonster
from project2 import helper
MAX_MEMORY = 100000
BATCH_SIZE = 1000
LR = 0.001



class trainer:

    def __init__(self, map, character,monster1,m1_dect,monster2,m2_dect):
        self.n_games = 0
        self.epsilon = 0
        self.gamma = 0.9
        self.memory = deque(maxlen=MAX_MEMORY)
        self.map = map
        self.C_info = character
        self.monster1 = monster1
        self.m1_dect = m1_dect
        self.monster2 = monster2
        self.m2_dect = m2_dect
        self.setup()

    def setup(self):
        self.top_score = 5000
        self.score = 0
                # Create the game
        self.game= Game.fromfile(self.map)

        # TODO Add your character
        self.game.add_character(TestCharacter(self.C_info[0][0], # name
                                    self.C_info[0][1],  # avatar
                                   self.C_info[1][0], self.C_info[1][1]  # position
        ))
        if self.monster1 != None:
            self.add_monster(self.monster1,self.m1_dect)
        if self.monster2 != None:
            self.add_monster(self.monster2,self.m2_dect)
        self.train()
    def add_monster(self,monster, dect_r):
        if monster[0][0] == "stupid":
            self.game.add_monster(StupidMonster(monster[0][0], # name
                                    monster[0][1],  # avatar
                                   monster[1][0], monster[1][1]  # position
        ))
        else:
            self.game.add_monster(SelfPreservingMonster(monster[0][0], # name
                                    monster[0][1],  # avatar
                                   monster[1][0], monster[1][1],  # position
                                    dect_r                 # detection range
        ))
    def train(self):
        plot_scores = [0]
        plot_mean_scores =[0]
        total_score = 0
        record = 0

        while True:
            self.game.go(0)
            print('Game', self.n_games )
           
            self.get_Score()

            if self.game.done():
                print('game Status', self.game.done())
                self.n_games+=1
                if self.score > record:
                    record = self.score
                    print('New High Score', record)
                plot_scores.append(self.score)
                total_score+=self.score
                mean_score = total_score/self.n_games
                plot_mean_scores.append(mean_score)
                helper.plot(self.n_games,plot_scores,plot_mean_scores)
                self.setup()
                
                
            if self.n_games==2:
                break
        time.sleep(2)
    def get_Score(self):
        print("Getting Score")
        self.score = self.game.world.scores['me']
        

if __name__ == '__main__':
    trainer.train()
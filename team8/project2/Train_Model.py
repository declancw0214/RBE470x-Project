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
import csv
import pandas as pd



class trainer:
    need_weight_index = True
    
    def __init__(self, map, character,monster1,m1_dect,monster2,m2_dect):
        self.n_games = 0
        self.total_wins = 0
        self.total_losses = 0
        self.map = map
        self.C_info = character
        self.monster1 = monster1
        self.m1_dect = m1_dect
        self.monster2 = monster2
        self.m2_dect = m2_dect
        self.setup()
        self.get_index()

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
        
        self.get_index()

        while self.n_games < 10:
            self.game.go(0)
            print('Game', self.n_games )
           
            self.get_Score()
            if self.game.done():
                
                print('game Status', self.game.done())
                self.n_games+=1
                if self.score < 4500:
                    self.total_losses +=1
                else:
                    self.total_wins += 1
                if self.score > record:
                    record = self.score
                    print('New High Score', record)
                
                plot_scores.append(self.score)
                total_score+=self.score
                mean_score = total_score/self.n_games
                plot_mean_scores.append(mean_score)
                helper.plot(self.n_games,plot_scores,plot_mean_scores, self.total_losses,self.total_wins)
                # self.update_index()
                self.setup()

        print("total Games: ", self.n_games, "Wins: ", self.total_wins," Loses: ", self.total_losses)

    def get_Score(self):
        print("Getting Score")
        self.score = self.game.world.scores['me']

    def get_index(self):
        if self.need_weight_index:
            indexes = pd.read_csv('index.csv')
            self.WEIGHT_INDEX = indexes["index"][0]
            self.need_weight_index=False

    def update_index(self):
        self.WEIGHT_INDEX +=1
        with open("index.csv", 'w') as csvfile:
            updater = csv.writer(csvfile)
            updater.writerow(["index",""])
            updater.writerow([self.WEIGHT_INDEX, ])
            csvfile.close()
        self.need_weight_index=True

if __name__ == '__main__':
    trainer.train()
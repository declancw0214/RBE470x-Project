# This is necessary to find the main code
import sys
sys.path.insert(0, '../../bomberman')
sys.path.insert(1, '..')

# Import necessary stuff
import random
from game import Game
from monsters.selfpreserving_monster import SelfPreservingMonster
from project2 import Train_Model 
# TODO This is your code!
sys.path.insert(1, '../team8')
from testcharacter import TestCharacter

# Create the game
random.seed(123) # TODO Change this if you want different random choices
g = Game.fromfile('map.txt')
# g.add_monster(SelfPreservingMonster("selfpreserving", # name
#                                     "S",              # avatar
#                                     3, 9,             # position
#                                     1                 # detection range
# ))

# # TODO Add your character
# g.add_character(TestCharacter("me", # name
#                               "C",  # avatar
#                               0, 0  # position
# ))
"""If not monster fill last two places with none"""
Train_Model.trainer(('map.txt'),(("me","C"),(1,10)),(("selfpreserving","S"),(3,9)), 1,None,None)
# Run!
g.go()

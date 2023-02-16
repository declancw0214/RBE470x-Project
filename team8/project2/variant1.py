# This is necessary to find the main code
import sys
sys.path.insert(0, '../../bomberman')
sys.path.insert(1, '..')


# Import necessary stuff
from game import Game
from project2 import Train_Model 
# TODO This is your code!
sys.path.insert(1, '../team8')
# from testcharacter import TestCharacter


# Create the game
# g= Game.fromfile('map.txt')

# # TODO Add your character
# g.add_character(TestCharacter("me", # name
#                             "C",  # avatar
#                             0, 0  # position
# ))
"""If not monster fill last two places with None"""
Train_Model.trainer(('map.txt'),(("me","C"),(1,16)),None,None,None,None)
# Run!
    
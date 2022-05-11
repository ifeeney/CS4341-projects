# This is necessary to find the main code
import sys
sys.path.insert(0, '../../bomberman')
sys.path.insert(1, '..')

# Import necessary stuff
import random
from game import Game
from monsters.selfpreserving_monster import SelfPreservingMonster

# TODO This is your code!
sys.path.insert(1, '../group10')
from testcharacter import TestCharacter

# Create the game
random.seed(1237) # TODO Change this if you want different random choices
# random.seed(324)
g = Game.fromfile('map.txt')
g.add_monster(SelfPreservingMonster("aggressive", # name
                                    "A",          # avatar
                                    3, 13,        # position
                                    2             # detection range
))

# g.add_monster(SelfPreservingMonster("aggressive", # name
#                                     "A",          # avatar
#                                     3, 3,        # position
#                                     2             # detection range
# ))

# TODO Add your character
g.add_character(TestCharacter("me", # name
                              "C",  # avatar
                              0, 0  # position
))

# g.add_character(TestCharacter("me", # name
#                               "C",  # avatar
#                               7, 1  # position
# ))

# Run!
g.go(1)

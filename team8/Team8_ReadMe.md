Helper classes

    helper.py
        - plots the average score and mean score over game interations
        - code was inspired from the following github
            https://github.com/patrickloeber/snake-ai-pytorch/blob/main/helper.py

    Train_Model.py
        - iteratively runs the game to train the test character
        - code was inspired from the following github with a lot of variation to suit our projects needs
            https://github.com/patrickloeber/snake-ai-pytorch/blob/main/helper.py
        - the class is called by the variants as 
            Train_Model.trainer(map name,character setup info, monster 1 setup info, monster 1 dection range,monster 2 setup info, monster 2 dection range)
                character and monster info is structured as a tuple of tuple:
                        ((name,avatar)(x position, y position))
                        if a monster isn't used monster info and dection range is set to None
    Test Character Festures
        -a star distance to exit
        -a star distance to closest monster
        -should a bomb be dropped
        - distance to explosion
        - distance to right wall
        - distance to bottom wall
        - distance to left wall
        - distance to top wall
        - cosine similarity of the path to monster and path to exit

        - wall directions are labeled clockwise
            |-------|
            |   4   |
            |   |   |
            | 3-C-1 |
            |   |   |
            |   2   |
            |-------|

Packages we added
    -pandas
    -matlibplot
    -ipython
    -time
    -csv
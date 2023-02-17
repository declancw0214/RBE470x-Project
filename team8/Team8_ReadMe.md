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
Packages we added
    -pandas
    -matlibplot
    -ipython
    -time
    -csv
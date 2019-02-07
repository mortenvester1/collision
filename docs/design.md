# Design Thoughts
This documents outlines the thoughts behind the implementation


### engine.py
Here is where the code that runs the sim


### objects.py
In objects.py you'll find the implementation for the objects that are colliding in this simulation. The block will have attributes and methods related to the block.

The purpose of the block is to move and collide. Thus we need method to move it around and change it's speed. Not much more should be needed. Of course a mass is also required.
For visual purposes we also have a shape.

attributes:
    mass        - required to
    velocity    -
    position    -
    shape       - for visual purposes
methods:
    set_x       - to update position
    set_v       - to update velocity


### simulate.py
simulate.py is meant for the execution of the a simulation. It is there to parse command line arguments and initiate everything.

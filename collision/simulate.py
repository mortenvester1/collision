import os
import sys
import argparse

from engine import *
from objects import *


def main():
    left =  Block(mass=100**0, shape=(1,1), start_v=0., start_x=2)
    right =  Block(mass=100**2., shape=(1,1), start_v=-1, start_x=4)
    wall = Wall(position=0, width=1)

    engine = PhysicsEngine(timestep=1e-2,
                           wall=wall,
                           left_square=left,
                           right_square=right,
                           visual=False)

    engine.run(debug=False)
    return


if __name__ == '__main__':
    main()

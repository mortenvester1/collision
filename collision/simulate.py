import os
import sys
import argparse

from engine import *
from objects import *


def main(dt, ratio, v, viz):


    left =  Block(mass=100**0, shape=(1,1), start_v=0., start_x=3)
    right =  Block(mass=100**ratio, shape=(1,1), start_v=v, start_x=6)
    wall = Wall(position=0, width=1)

    engine = PhysicsEngine(timestep=dt,
                           wall=wall,
                           left_square=left,
                           right_square=right)

    engine.run(debug=False, viz=viz)
    return


if __name__ == '__main__':
    parser = argparse.ArgumentParser()

    # action=store_true means default is False
    parser.add_argument('-V', '--viz',
        action='store_true',
        help='flag whether to visualize')
    parser.add_argument('-dt', '--timestep',
        type=float,
        default=1e-2,
        help='size of time step')
    parser.add_argument('-r', '--ratio',
        type=int,
        default=3,
        help='mass ratio')
    parser.add_argument('-v', '--velocity',
        type=float,
        default=-1,
        help='init velocity')

    args = parser.parse_args()
    dt = args.timestep
    ratio = args.ratio
    viz = args.viz
    v = args.velocity
    main(dt=dt, ratio=ratio, v=v, viz=viz)

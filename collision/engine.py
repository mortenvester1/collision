import pdb
import time
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.animation as ani


class PhysicsEngine(object):
    """
    Our physics engine used to simulate the collision problem in a overly
    simple 2d world. The world has an immovable wall on the left side
    and extends into the void.
    """
    def __init__(self, timestep=.2, wall=None, left_square=None, right_square=None):
        """
        Initialize the world

        visualize
        """
        #assert viz is False, "Visualization not yet implemented"
        self.dt = timestep
        self.elapsed = 0
        self.l = left_square
        self.r = right_square
        self.w = wall
        self.c = 0

        self.eps = 1e-6

    def run(self, viz=False, verbose=True, debug=True):
        if viz:
            self._run_ani()
            #raise Exception('viz not ready')
        else:
            self._run(verbose=verbose, debug=debug)

        return


    def _run(self, verbose=True, debug=True):
        """
        run the full simulation until no more collisions occur
        """
        # Counter to end simulation when there will be no more
        end_simulation = False
        elapsed = 0

        if verbose: print("{0:>8s}\t{1:>11s}\t{2:>7s}\t\t{3:>7s}\t{4:>10s}\t{5:>10s}\t{6:>10s}"\
                           .format("time", "#Collisions", "left x", "right x", "left v", "right v", "total K"))

        prev_count = -1
        prev_K = self.l.K + self.r.K
        while True and not end_simulation:
            #Take a timestep
            self.elapsed += self._step(self.w, self.l, self.r)

            # Compute and print metrics/stats/...
            new_K = self.l.K + self.r.K
            dK = abs(prev_K - new_K)
            #if verbose and prev_count < self.c:
            if verbose:
                print("{0:>8.4f}\t{1:>11d}\t{2:>7.4f}\t\t{3:>7.4f}\t{4: >10.4f}\t{5: >10.4f}\t{6:>10.4f}"\
                   .format(self.elapsed, self.c, self.l.x, self.r.x, self.l.v, self.r.v, dK), flush=True)
                prev_count = self.c

            # Force simulation to end when no more collisions
            if self.check_more_collisions(self.l, self.r) or debug:
                print('NO MORE COLLISIONS... ending simulation')
                print("l end momentum : {0: >10.4f}".format(self.l.p))
                print("l end kinetic e: {0: >10.4f}".format(self.l.K))
                print("r end momentum : {0: >10.4f}".format(self.r.p))
                print("r end kinetic e: {0: >10.4f}".format(self.r.K))
                end_simulation = True

            # Verify no loss of kinetic energy
            if dK > 1e-4:# and False:
                raise Exception('Conservation of kinetic energy violated.')

            prev_K = new_K

        return


    def _run_ani(self):
        xlim_r = self.r.x + self.r.w/2 + 1
        xlim_l = self.w.x - self.w.w/2 - 1

        fig = plt.figure()
        ax = fig.add_subplot(111, aspect='equal', autoscale_on=False,
                             xlim=(xlim_l, xlim_r), ylim=(0, 4))
        ax.grid()

        wall, = ax.fill([], [], color='k')
        lblock, = ax.fill([], [], color='b')
        rblock, = ax.fill([], [], color='r')
        time_text = ax.text(0.02, 0.95, '', transform=ax.transAxes)
        count_text = ax.text(0.02, 0.90, '', transform=ax.transAxes)

        def draw_square(x, w, h=1):
            arr = np.array([[x - w/2, 0], [x + w/2, 0],
                            [x + w/2, h], [x - w/2, h]])
            return arr


        def animate(frame):
            """ update animate animation step """
            wall.set_xy(draw_square(self.w.x, self.w.w, h=2))
            lblock.set_xy(draw_square(self.l.x, self.l.w, self.l.w))
            rblock.set_xy(draw_square(self.r.x, self.r.w, self.r.w))

            time_text.set_text('time: {0:5.4f}'.format(self.elapsed))
            count_text.set_text('count: {0:<d}'.format(self.c))

            return wall, lblock, rblock, time_text, count_text


        def frames_generator():
            """
            generate
            """
            while True:
                dt = self._ani_step(wall=self.w, lblock=self.l, rblock=self.r)
                yield dt
                if dt is None:
                    time.sleep(1)
                    break
                self.elapsed += dt

        try:
            meh = ani.FuncAnimation(
                fig,
                animate,
                frames=frames_generator,
                interval=0,
                blit=True,
                init_func=None,
                repeat=False)
            plt.show(block=True)
            #meh.event_source.stop()
            #del meh
            #plt.close()
        except:
            pass

        return


    def _ani_step(self, wall, lblock, rblock, dt=None):

        if self.check_more_collisions(lblock, rblock):
            return None

        return self._step(wall, lblock, rblock)


    def _step(self, wall, lblock, rblock, dt=None):
        """
        take a single timestep, if a collisino occurs in that timestep, we will
        set the positions to where the colliion occurs, update speed
        """
        if dt is None:
            dt = self.dt

        # 0 velocity change if no collision
        l_dv, r_dv = 0, 0

        # Find Displacements if no collision
        l_dx, r_dx = lblock.v * dt, rblock.v * dt

        # Find new Locations / velocity, if nothing happens
        l_new_x, l_new_v = lblock.x + l_dx, lblock.v
        r_new_x, r_new_v = rblock.x + r_dx, rblock.v

        # Check if left square hits wall. get new position and velocity
        wall_collision, wall_time = self.check_wall_collision(wall, lblock, l_dx)

        # Check if squares collide
        block_collision, block_time = self.check_block_collision(lblock, rblock, l_dx, r_dx)

        # Wall collision happens first
        if wall_time < block_time:
            #print('wall collision')
            dt = wall_time
            l_new_x, l_new_v, r_new_x, r_new_v = self.wall_collision(wall, lblock, rblock, dt)
        # Block collision happens first
        elif block_time < wall_time:
            #print('block collision')
            dt = block_time
            l_new_x, l_new_v, r_new_x, r_new_v = self.block_collision(lblock, rblock, dt)

        # no collision we just update
        self.update(l_new_x, l_new_v, r_new_x, r_new_v)
        return dt


    def check_more_collisions(self, lblock, rblock):
        """
        Check if there are no more collisions
        """
        # stop, if both are moving to the right and the big is moving faster
        return (0 <= lblock.v <= rblock.v)


    def check_wall_collision(self, wall, block, dx):
        """
        Check if there will be a collision with the wall
        If True compute time until collision
        """
        collision = False
        time_until = float('inf')

        if block.get_anchor(left=True) + dx <= wall.get_anchor(left=False):
            collision = True
            time_until = abs((block.get_anchor(left=True) - wall.get_anchor(left=False)) / block.v)

        return collision, time_until


    def check_block_collision(self, lblock, rblock, l_dx, r_dx):
        """
        Check if there will be a collision with the wall
        If True compute time until collision
        """
        collision = False
        time_until = float('inf')

        if lblock.get_anchor(left=False) + l_dx >= rblock.get_anchor(left=True) + r_dx:
            collision = True
            time_until = (rblock.get_anchor(left=True) - lblock.get_anchor(left=False)) / (abs(lblock.v) + abs(rblock.v))

        return collision, time_until


    def wall_collision(self, wall, lblock, rblock, dt):
        """
        If left square will collide with wall, find new speed/location

        Object changes direction but keeps speed, i.e. velocity changes sign.
        We place the object

        """
        l_new_x = wall.get_anchor(left=False) + lblock.get_anchor(left=True)
        l_new_v = - lblock.v

        r_new_x = rblock.x + rblock.v * dt
        r_new_v = rblock.v

        # Update Count
        self.c += 1

        return l_new_x, l_new_v, r_new_x, r_new_v


    def block_collision(self, lblock, rblock, dt):
        """ Check if two square will collide """
        # Compute momentum before collision
        p_before = lblock.p + rblock.p

        # Find new position
        l_new_x = lblock.x + lblock.v * dt
        r_new_x = rblock.x + rblock.v * dt

        # Find new velocity
        total_mass = lblock.m + rblock.m
        l_new_v = lblock.v * (lblock.m - rblock.m) / total_mass + rblock.v * (2.*rblock.m / total_mass)
        r_new_v = rblock.v * (rblock.m - lblock.m) / total_mass + lblock.v * (2.*lblock.m / total_mass)

        # Compute momentum after collision
        p_after = l_new_v * lblock.m + r_new_v * rblock.m

        # Verify no loss of momentum due to collision
        if abs(p_before - p_after) > 1e-4:
            # Not total momentum changes over time due
            # to the wall changing the momentum of the left Object
            raise Exception("Momentum not conserved in block collision.")

        # Update Count
        self.c += 1

        return l_new_x, l_new_v, r_new_x, r_new_v


    def update(self, l_x, l_v, r_x, r_v):
        self.l._set_x(l_x)
        self.l._set_v(l_v)
        self.r._set_x(r_x)
        self.r._set_v(r_v)
        return

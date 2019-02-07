


class Block(object):
    def __init__(self, mass=1, shape=(1,1), start_v=0, start_x=10):
        """
        args
            mass        <int>               | mass of the square
            shape       <tuple[int]>        | shape of the square (width, height)
            start_v     <int or float>      | start velocity
            start_x     <int or float>      | start position

        notes
            - Only the width of the square matters for name
            - Velocity abides by the convention that negative velocity moves the
                square to the left and postive moves it to the right.
        """
        self.m = mass
        self.w = shape[0]
        self.h = shape[1]
        self.v = start_v
        self.x = start_x
        self.p = None
        self.K = None

        self._set_p()
        self._set_K()
        pass

    def _set_v(self, v):
        self.v = v

        self._set_p()
        self._set_K()


    def _set_x(self, x):
        self.x = x

    def _set_p(self):
        self.p = self.v * self.m

    def _set_K(self):
        self.K = .5 * self.m * (self.v**2)


    def get_anchor(self, x=None, left=True):
        if x is None:
            x = self.x
        return x - self.w / 2. if left else x + self.w / 2.

class Wall(object):
    def __init__(self, position=0, width=1):
        self.w = width
        self.x = 0

    def get_anchor(self, left=True):
        return self.x - self.w / 2. if left else self.x + self.w / 2.

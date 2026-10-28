"""A station-time sample used by the lattice end-condition sampler."""


class STPoint:
    def __init__(self, s: float, t: float):
        self.s = float(s)
        self.t = float(t)

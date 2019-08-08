import numpy as np


class Observation:
    # Observations of the players
    possible_speeds = [-1, 1]
    possible_lanes = [-1, 0, 1]
    possible_dgoal = [0, 1, 2, 3, 4]

    def __init__(self):
        #
        self._fl = -1
        # Front
        self._f = -1
        # Front right
        self._fr = -1
        # Center left
        self._cl = -1
        # Center right
        self._cr = -1
        # Rear left
        self._rl = -1
        # Rear right
        self._rr = -1
        # Back left
        self._bl = -1
        # Back right
        self._br = -1
        # Distance to goal
        self._dgoal = 0
        # Lane
        self._lane = 0
        # Observation
        self.observation = np.array([[self._fl, self._f, self._fr, self._cl, self._cr, self._rl,
                                      self._rr, self._bl, self._br, self._dgoal, self._lane]])

    @property
    def fl(self):
        """ Front left observation value """
        return self._fl

    @fl.setter
    def fl(self, value):
        self._fl = self.set_value(value, self.possible_speeds)

    @property
    def f(self):
        """ Front observation value """
        return self._f

    @f.setter
    def f(self, value):
        self._f = self.set_value(value, self.possible_speeds)

    @property
    def fr(self):
        """ Front right observation value """
        return self._fr

    @fr.setter
    def fr(self, value):
        self._fr = self.set_value(value, self.possible_speeds)

    @property
    def cl(self):
        """ Center left observation value """
        return self._cl

    @cl.setter
    def cl(self, value):
        self._cl = self.set_value(value, self.possible_speeds)

    @property
    def cr(self):
        """ Center right observation value """
        return self._cr

    @cr.setter
    def cr(self, value):
        self._cr = self.set_value(value, self.possible_speeds)

    @property
    def bl(self):
        """ Back left observation value """
        return self._bl

    @bl.setter
    def bl(self, value):
        self._bl = self.set_value(value, self.possible_speeds)

    @property
    def br(self):
        """ Back right observation value """
        return self._br

    @br.setter
    def br(self, value):
        self._br = self.set_value(value, self.possible_speeds)

    @property
    def rl(self):
        """ Rear left observation value """
        return self._rl

    @rl.setter
    def rl(self, value):
        self._rl = self.set_value(value, self.possible_speeds)

    @property
    def rr(self):
        """ Front left observation value """
        return self._rr

    @rr.setter
    def rr(self, value):
        self._rr = self.set_value(value, self.possible_speeds)

    @property
    def dgoal(self):
        """ Distance to goal observation value """
        return self._dgoal

    @dgoal.setter
    def dgoal(self, value):
        self._dgoal = self.set_value(value, self.possible_dgoal)

    @property
    def lane(self):
        """ Lane observation value """
        return self._lane

    @lane.setter
    def lane(self, value):
        self._lane = self.set_value(value, self.possible_lanes)

    @staticmethod
    def set_value(value, possible_values):
        if value in possible_values:
            variable = value
            # self.reload_observation_value()
        else:
            raise ValueError("lane should be in one of the possible lanes")

        return variable

    def reload_observation_value(self):
        # Observation
        self.observation = np.array([[self._fl, self._f, self._fr, self._cl, self._cr, self._rl,
                                      self._rr, self._bl, self._br, self._dgoal, self._lane]])


class ObservationSpace(Observation):
    # Observations of the players
    def __init__(self):
        Observation.__init__(self)
        self.observation_space = self.get_observation_space()

    @classmethod
    def get_observation_space(cls):
        # possible_states = [np.array([[fl, f, fr, rl, rr, bl, br, vel]])
        observation_space = [np.array([[fl, f, fr, cl, cr, rl, rr, bl, br, dgoal, lane]])
                             for fl in cls.possible_speeds
                             for f in cls.possible_speeds
                             for fr in cls.possible_speeds
                             for cl in cls.possible_speeds
                             for cr in cls.possible_speeds
                             for rl in cls.possible_speeds
                             for rr in cls.possible_speeds
                             for bl in cls.possible_speeds
                             for br in cls.possible_speeds
                             for dgoal in cls.possible_dgoal
                             for lane in cls.possible_lanes]
        #
        print 'observation space : \n ' + str(observation_space)
        print 'observation space size: \n ' + str(len(observation_space))
        return observation_space

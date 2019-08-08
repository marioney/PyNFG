import numpy as np


class Action:
    # actions of the players

    left = np.array([1, 0])
    right = np.array([-1, 0])
    accel = np.array([0, 1.])
    remain = np.array([0, 0])
    brake = np.array([0, -1])
    hard_brake = np.array([0, -2])
    hard_accel = np.array([0, 2])
    possible_actions = [left, right, accel, remain, brake, hard_brake]
    nr_actions = len(possible_actions)

    def __init__(self, zeros=False):
        self._prob_right = 0.0
        self._prob_left = 0.0
        self._prob_accel = 0.0
        self._prob_remain = 1.0
        self._prob_brake = 0.0
        self._prob_hard_brake = 0.0
        # self._prob_hard_accel = 0.1
        # Probability distribution
        if zeros:
            self._prob_remain = 0.0
        self.prob_actions = np.array([self._prob_left,
                                      self._prob_right,
                                      self._prob_accel,
                                      self._prob_remain,
                                      self._prob_brake,
                                      self._prob_hard_brake])

    @classmethod
    def action_zeros(cls):
        return Action(True)

    def default_prob(self):
        self._prob_right = 0.05
        self._prob_left = 0.05
        self._prob_accel = 0.1
        self._prob_remain = 0.5
        self._prob_brake = 0.2
        self._prob_hard_brake = 0.1
        # self._prob_hard_accel = 0.1
        self.reload_prob_actions_value()
        return self.prob_actions

    @property
    def prob_left(self):
        """ Probability of action Left """
        return self._prob_right

    @prob_left.setter
    def prob_left(self, value):
        self.prob_left = self.set_value(value)

    @property
    def prob_right(self):
        """ Probability of action Right """
        return self._prob_right

    @prob_right.setter
    def prob_right(self, value):
        self._prob_right = self.set_value(value)

    @property
    def prob_accel(self):
        """ Probability of action Accel """
        return self._prob_accel

    @prob_accel.setter
    def prob_accel(self, value):
        self._prob_accel = self.set_value(value)

    @property
    def prob_remain(self):
        """ Probability of action Remain """
        return self._prob_remain

    @prob_remain.setter
    def prob_remain(self, value):
        self._prob_remain = self.set_value(value)

    @property
    def prob_brake(self):
        """ Probability of action Brake """
        return self._prob_brake

    @prob_brake.setter
    def prob_brake(self, value):
        self._prob_brake = self.set_value(value)

    @property
    def prob_hard_brake(self):
        """ Probability of action Hard brake """
        return self._prob_hard_brake

    @prob_hard_brake.setter
    def prob_hard_brake(self, value):
        self._prob_hard_brake = self.set_value(value)

    @staticmethod
    def set_value(value):
        if 0.0 > value > 1.0:
            variable = value
        else:
            raise ValueError("Probability value should be in the [0 - 1] range")

        return variable

    def reload_prob_actions_value(self):
        # Check that the sum of the probabilities is 1
        sum_probabilities = 0.0
        sum_probabilities += self._prob_left
        sum_probabilities += self._prob_right
        sum_probabilities += self._prob_accel
        sum_probabilities += self._prob_remain
        sum_probabilities += self._prob_brake
        sum_probabilities += self._prob_hard_brake

        if abs(sum_probabilities - 1.0) < 0.00000000000001:
            self.prob_actions = np.array([self._prob_left,
                                          self._prob_right,
                                          self._prob_accel,
                                          self._prob_remain,
                                          self._prob_brake,
                                          self._prob_hard_brake])
        else:
            raise ValueError("Sum of probabilities should be 1")
import numpy as np
from observation import ObservationSpace
from action import Action


class InitialPolicy:

    def __init__(self):
        self.observations = ObservationSpace()
        self.action_space = Action()
        self.level_0_policy = self.get_initial_policy()

    def get_initial_policy(self):
        # possible_states = [np.array([[a, b, c], [d, e, f]])
        #                    for a in possible_speeds
        #                    for b in [0, 1, 2]
        #                    for c in possible_speeds
        #                    for d in possible_speeds
        #                    for e in possible_speeds
        #                    for f in possible_speeds]

        policy_shape = (len(self.observations.observation), self.action_space.nr_actions)

        level_0_policy = np.zeros(policy_shape)
        print 'policy size: ' + str(len(level_0_policy))
        # print 'Previous: '
        # print level_0_policy

        # prob_dist_actions = np.array([prob_left, prob_accel, prob_remain, prob_brake])
        index = 0
        for fl in self.observations.possible_speeds:
            for f in self.observations.possible_speeds:
                for fr in self.observations.possible_speeds:
                    for cl in self.observations.possible_speeds:
                        for cr in self.observations.possible_speeds:
                            for rl in self.observations.possible_speeds:
                                for rr in self.observations.possible_speeds:
                                    for bl in self.observations.possible_speeds:
                                        for br in self.observations.possible_speeds:
                                            for dgoal in self.observations.possible_dgoal:
                                                for lane in self.observations.possible_lanes:
                                                    new_action = Action.action_zeros()
                                                    if lane == 0:
                                                        if f == 1:
                                                            new_action.prob_brake = 1.0
                                                        elif f == -1:
                                                            new_action.prob_accel = 1.0
                                                        else:
                                                            new_action.prob_remain = 1.0
                                                    elif lane == 1:
                                                        if rl == -1 and cl == -1:
                                                            new_action.prob_left = 1.0
                                                        else:
                                                            if f == 1:
                                                                new_action.prob_brake = 1.0
                                                            else:
                                                                new_action.prob_remain = 1.0
                                                    elif lane == -1:
                                                        if rr == -1 and cr == -1:
                                                            new_action.prob_right = 1.0
                                                        else:
                                                            if f == 1:
                                                                new_action.prob_brake = 1.0
                                                            else:
                                                                new_action.prob_remain = 1.0

                                                    new_action.reload_prob_actions_value()
                                                    # prob_dist_actions = np.array([prob_left,
                                                    #                               prob_right,
                                                    #                               prob_accel,
                                                    #                               prob_remain,
                                                    #                               prob_brake])
                                                    # print index
                                                    # print prob_dist_actions
                                                    level_0_policy[index, :] = new_action.prob_actions
                                                    # prob_dist_actions
                                                    index += 1

        print 'After: '
        print level_0_policy
        return level_0_policy

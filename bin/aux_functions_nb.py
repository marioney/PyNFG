#!/usr/bin/env python


import numpy as np

try:
    import cPickle as pickle
except ImportError:
    import pickle

from math import ceil

# import math


def calculate_distance(current_pose, other_pose):

    x1 = current_pose[0]
    x2 = current_pose[1]
    y1 = other_pose[0]
    y2 = other_pose[1]
    dist = 100
    if x1 == y1:
        if y2 >= x2:
            dist = y2 - x2
        else:
            dist = x2 - y2

    return dist


def distance_to_back(current_pose, other_pose):

    x1 = current_pose[0]
    x2 = current_pose[1]
    y1 = other_pose[0]
    y2 = other_pose[1]
    dist = 100
    if x1 == y1:
        if x2 >= y2:
            dist = x2 - y2
    return dist


def distance_to_front(current_pose, other_pose):

    x1 = current_pose[0]
    x2 = current_pose[1]
    y1 = other_pose[0]
    y2 = other_pose[1]
    dist = 100
    if x1 == y1:
        if y2 >= x2:
            dist = y2 - x2
    return dist


def check_col(current_pose, other_pose):
    d_front = distance_to_front(current_pose, other_pose)
    d_back = distance_to_back(current_pose, other_pose)
    if d_front < 1.0:
        return True
    elif d_back < 1.0:
        return True
    return False


def get_distance(current_pose, pose_array):
    """
    :param current_pose: Pose being evaluated
    :param pose_array:   List of occupied poses.
    :return: True if a collision is found, False otherwise
    """
    # print 'pose array shape' + str(pose_array.shape)
    distance = 100
    if pose_array is None:
        return True
    if pose_array.ndim is 1:
        distance = calculate_distance(current_pose, pose_array)
        # print ' Collission in pose' + str(current_pose)
        # return True
    else:
        [rows, cols] = pose_array.shape
        if cols is not 2:
            print 'pose array should have 2 cols - it has ' + str(cols)
            return True
        for index_compare in range(0, rows):
            temp_distance = calculate_distance(current_pose, pose_array[index_compare])
            if temp_distance < distance:
                distance = temp_distance
                # print 'new_pose ' + str(current_pose) + 'in collision with car ' + str(index_compare)
    return distance


def check_collision(current_pose, pose_array):
    """
    :param current_pose: Pose being evaluated
    :param pose_array:   List of occupied poses.
    :return: True if a collision is found, False otherwise
    """
    # print 'pose array shape' + str(pose_array.shape)
    if pose_array is None:
        return True
    if pose_array.ndim is 1:
        if check_col(current_pose, pose_array) is True:
            # print ' Collission in pose' + str(current_pose)
            return True
    else:
        [rows, cols] = pose_array.shape
        if cols is not 2:
            print 'pose array should have 2 cols - it has ' + str(cols)
            return True

        for index_compare in range(0, rows):
            if check_col(current_pose, pose_array[index_compare]) is True:
                return True
    return False


def gap_to_front(current_pose, pose_array):
    """
    :param current_pose: Pose being evaluated
    :param pose_array:   List of occupied poses.
    :return: True if a collision is found, False otherwise
    """
    # print 'pose array shape' + str(pose_array.shape)
    distance = 100
    if pose_array is None:
        return True
    if pose_array.ndim is 1:
        distance = distance_to_front(current_pose, pose_array)
        # print ' Collission in pose' + str(current_pose)
        # return True
    else:
        [rows, cols] = pose_array.shape
        if cols is not 2:
            print 'pose array should have 2 cols - it has ' + str(cols)
            return True
        for index_compare in range(0, rows):
            temp_distance = distance_to_front(current_pose, pose_array[index_compare])
            if temp_distance < distance:
                distance = temp_distance
                # print 'new_pose ' + str(current_pose) + 'in collision with car ' + str(index_compare)
    return distance


def read_training_values(nr_vehicles):

    training_values = []
    for index_node in range(nr_vehicles):
        # adding the other nodes

        car_name = 'Dcar' + str(index_node)

        file_name = 'Training' + car_name + '.pickle'
        print 'Loading: ' + file_name
        try:
            pickle_in = open(file_name, "rb")
        except IOError:
            print "Error: File does not appear to exist"
            pickle_in = None

        t_vaule = None
        if pickle_in is not None:
            try:
                t_vaule = pickle.load(pickle_in)
            except pickle.PicklingError:
                print 'Error when reading data'

        if t_vaule is not None:
            # print t_vaule
            training_values.append(t_vaule)
        else:
            return None

    return training_values


def store_training_values(nr_vehicles, game):

    for index_node in range(nr_vehicles):
        car_name = 'Dcar' + str(index_node)
        print 'Training: ' + car_name + 'at level 2'
        file_name = 'Training' + car_name + '.pickle'
        pickle_out = open(file_name, "wb")
        pickle.dump(game.bn_part[car_name][0].CPT, pickle_out)
        # print ' 0 '
        # print game.bn_part[car_name][0].CPT
        pickle_out.close()


def store_game(game, filename='Game_stored.pickle'):

    print 'Storing Game'
    file_name = filename
    pickle_out = open(file_name, "wb")
    pickle.dump(game, pickle_out)
    pickle_out.close()


def read_game(filename='Game_stored.pickle'):

    print 'Reading Game'
    file_name = filename
    try:
        pickle_in = open(file_name, "rb")
    except IOError:
        print "Error: File does not appear to exist"
        pickle_in = None

    game_vaule = None
    if pickle_in is not None:
        try:
            game_vaule = pickle.load(pickle_in)
        except pickle.PicklingError:
            print 'Error when reading data'
            game_vaule = None
    return game_vaule
    pickle_in.close()


def get_observations(current_status, nr_vehicles, vehicle_nr, north, east, west, blocked_north):

    current_poses = current_status[0]
    current_velocities = current_status[1]
    my_vel = current_velocities[vehicle_nr][1]
    my_pos = current_poses[vehicle_nr]

    # print '\n NEW OBSERVATION \n'
    # print 'current_velocities : \n ' + str(current_velocities)
    # print 'My vel : \n ' + str(my_vel)

    fl = -100
    f = -100
    fr = -100
    rl = -100
    rr = -100
    bl = -100
    br = -100

    vel = my_vel

    if vel > 0:
        vel = 1
    # my_vel += 1
    for index_node in range(nr_vehicles):
        if index_node == vehicle_nr:
            continue
        veh_pos = current_poses[index_node]
        # print 'veh n : ' + str(index_node) + ' Vel : ' + str(current_velocities[index_node][1])
        # print 'veh pos : ' + str(veh_pos) + ' my pos : ' + str(my_pos)
        difference = veh_pos - my_pos
        # print 'Difference: ' + str(difference)

        # if my_vel == 0:
        #     my_vel = 0.5

        x = difference[0]
        y = difference[1]

        if x == -1:
            if 1 <= y <= (my_vel + 1):
                if current_velocities[index_node][1] > fr:
                    fr = ceil(current_velocities[index_node][1])
                    if fr > 1:
                        fr = 1
            if 0.5 <= y < 1:
                rr = ceil(current_velocities[index_node][1])
                if rr > 1:
                    rr = 1
            if -1.5 < y < 0.5:
                br = ceil(current_velocities[index_node][1])
                if br > 1:
                    br = 1

        if x == 0:
            # print 'my vel : ' + str(my_vel)
            if 1 <= y <= (my_vel + 1):
                # print 'X : ' + str(x) + ' Y : ' + str(y)
                # print 'veh n : ' + str(index_node) + ' Vel : ' + str(current_velocities[index_node][1])
                # print 'prev f: ' + str(f)
                if current_velocities[index_node][1] > f:
                    f = ceil(current_velocities[index_node][1])
                    if f > 1:
                        f = 1

            if -1 < y < 1:
                vel = 100
                # current_velocities[index_node][1]

        if x == 1:
            if 1 <= y <= (my_vel + 1):
                if current_velocities[index_node][1] > fl:
                    fl = ceil(current_velocities[index_node][1])
                    if fl > 1:
                        fl = 1
            if 0.5 <= y < 1:
                rl = ceil(current_velocities[index_node][1])
                if rl > 1:
                    rl = 1
            if -1.5 < y < 0.5:
                bl = ceil(current_velocities[index_node][1])
                if bl > 1:
                    bl = 1

    # print 'observation space : \n ' + str(observation_space)

    # print 'my pos : ' + str(my_pos)

    # if my_pos[1] == 0:
    #    observation_space[1, :] = 100
    # if my_pos[1] == north:
    #     fl = 100
    #     fr = 100
    #     f = 100

    if my_pos[0] == west:
        rr = 100
        # fr = 100
        # br = 100
        if my_pos[1] >= north - blocked_north - 1 - my_vel:
            f = 100
    if my_pos[0] == east:
        rl = 100
        # fl = 100
        # bl = 100
        # if my_pos[1] > north - blocked_north - 1 - my_vel:
        #     fr = 100
        # if my_pos[1] > north - blocked_north - 1:
        #     rr = 100
        # if my_pos[1] > north - blocked_north:
        #     br = 100

    observations = np.array([[fl, f, fr, rl, rr, bl, br, vel]])
    # print 'Vehicle nr : ' + str(vehicle_nr) + ' pos : ' + str(my_pos)
    # print 'my vel : ' + str(my_vel)
    # print 'Current poses: \n' + str(current_poses)
    # print 'Current vel: \n' + str(current_velocities)
    # print ' fl: ' + str(fl) + ' f: ' + str(f) + ' fr: ' + str(fr) +\
    #       ' rl: ' + str(rl) + ' rr: ' + str(rr) + ' vel: ' + str(vel) + '\n'
    return observations


def get_observation_space(possible_speeds=[-100, 0, 1, 100]):
    possible_states = [np.array([[fl, f, fr, rl, rr, bl, br, vel]])
                       for fl in [-100, 0, 1]
                       for f in possible_speeds
                       for fr in [-100, 0, 1]
                       for rl in possible_speeds
                       for rr in possible_speeds
                       for bl in [-100, 0, 1]
                       for br in [-100, 0, 1]
                       for vel in [0, 1, 100]]
    #
    print 'observation space : \n ' + str(possible_states)
    print 'observation space size: \n ' + str(len(possible_states))
    return possible_states


def get_initial_policy(possible_states, possible_speeds=[-100, 0, 1, 100], n_actions=5):

    # possible_states = [np.array([[a, b, c], [d, e, f]])
    #                    for a in possible_speeds
    #                    for b in [0, 1, 2]
    #                    for c in possible_speeds
    #                    for d in possible_speeds
    #                    for e in possible_speeds
    #                    for f in possible_speeds]

    observation_shape = (len(possible_states), n_actions)

    level_0_policy = np.zeros(observation_shape)
    print 'policy size: ' + str(len(level_0_policy))
    # print 'Previous: '
    # print level_0_policy

    # prob_dist_actions = np.array([prob_left, prob_accel, prob_remain, prob_brake])
    index = 0
    for fl in [-100, 0, 1]:
        for f in possible_speeds:
            for fr in [-100, 0, 1]:
                for rl in possible_speeds:
                    for rr in possible_speeds:
                        for bl in [-100, 0, 1]:
                            for br in [-100, 0, 1]:
                                for vel in [0, 1, 100]:
                                    prob_right = 0.0
                                    prob_left = 0.0
                                    prob_accel = 0.0
                                    prob_remain = 0.0
                                    prob_brake = 0.0
                                    if vel == 1:
                                        if f == 100:
                                            prob_brake = 1.0
                                            # if fl == -100:
                                            #     prob_left = 1.0
                                            # else:
                                        elif f == 0:
                                            prob_brake = 1.0
                                        else:
                                            prob_remain = 1.0
                                    elif vel == 0:
                                        if f == -100:
                                            prob_accel = 1.0
                                        elif f == 100:
                                            if rl == -100:
                                                prob_left = 1.0
                                            else:
                                                prob_remain = 1.0
                                        else:
                                            prob_remain = 1.0
                                    elif vel == 100:
                                        prob_brake = 1.0
                                    else:
                                        prob_remain = 1.0

                                    prob_dist_actions = np.array([prob_left,
                                                                  prob_right,
                                                                  prob_accel,
                                                                  prob_remain,
                                                                  prob_brake])
                                    # print index
                                    # print prob_dist_actions
                                    level_0_policy[index, :] = prob_dist_actions
                                    index += 1

    print 'After: '
    print level_0_policy
    return level_0_policy
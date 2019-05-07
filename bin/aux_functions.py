#!/usr/bin/env python


import numpy as np

try:
    import cPickle as pickle
except ImportError:
    import pickle

# import math


def calculate_distance(current_pose, other_pose):

    x1 = current_pose[0]
    x2 = current_pose[1]
    y1 = other_pose[0]
    y2 = other_pose[1]
    dist = 100
    if y2 >= x2:
        if x1 == y1:
            dist = y2 - x2
            # dist = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
            return dist

    return dist


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
        if np.array_equal(current_pose, pose_array):
            # print ' Collission in pose' + str(current_pose)
            return True

    else:
        [rows, cols] = pose_array.shape
        if cols is not 2:
            print 'pose array should have 2 cols - it has ' + str(cols)
            return True

        for index_compare in range(0, rows):
            if np.array_equal(current_pose, pose_array[index_compare]):
                # print 'new_pose ' + str(current_pose) + 'in collision with car ' + str(index_compare)
                return True

    return False


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


def store_game(game):

    print 'Storing Game'
    file_name = 'Game_stored.pickle'
    pickle_out = open(file_name, "wb")
    pickle.dump(game, pickle_out)
    pickle_out.close()


def read_game(game):

    print 'Reading Game'
    file_name = 'Game_stored.pickle'
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
    my_pos = current_poses[vehicle_nr]
    my_vel = current_velocities[vehicle_nr][1]
    lane = my_pos[0]

    # print '\n NEW OBSERVATION \n'
    # print 'current_velocities : \n ' + str(current_velocities)
    # print 'My vel : \n ' + str(my_vel)

    fl = -100
    f = -100
    fr = -100
    rl = -100
    rr = -100
    vel = my_vel

    my_pos = current_poses[vehicle_nr]

    for index_node in range(nr_vehicles):
        if index_node == vehicle_nr:
            continue
        veh_pos = current_poses[index_node]
        # print 'veh n : ' + str(index_node) + ' Vel : ' + str(current_velocities[index_node][1])
        # print 'veh pos : ' + str(veh_pos) + ' my pos : ' + str(my_pos)
        difference = veh_pos - my_pos
        # print 'Difference: ' + str(difference)

        if my_vel == 0:
            my_vel = 1
        x = difference[0]
        y = difference[1]

        # print 'X : ' + str(x) + ' Y : ' + str(y)

        if x == -1:
            if 0 < y <= my_vel:
                if current_velocities[index_node][1] > fr:
                    fr = 0  # current_velocities[index_node][1]
            if y == 0:
                rr = 0  #  current_velocities[index_node][1]

        if x == 0:
            if 0 < y <= my_vel:
                if current_velocities[index_node][1] > f:
                    f = 0  #  current_velocities[index_node][1]
            if y == 0:
                vel = 100
                # current_velocities[index_node][1]

        if x == 1:
            if 0 < y <= my_vel:
                if current_velocities[index_node][1] > fl:
                    fl = 0  #  current_velocities[index_node][1]
            if y == 0:
                rl = 0  #  current_velocities[index_node][1]

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
        fr = 100
        if my_vel >= 2:
            if my_pos[1] >= north - blocked_north - 1:
                f = 100
        else:
            if my_pos[1] >= north - blocked_north:
                f = 100
    if my_pos[0] == east:
        rl = 100
        fl = 100
        if my_pos[1] > north - blocked_north:
            fr = 100
            rr = 100

    observations = np.array([[fl, f, fr, rl, rr, lane, vel]])
    # print 'my pos : ' + str(my_pos)
    # print 'my vel : ' + str(my_vel)
    # print 'Current poses: \n' + str(current_poses)
    # print 'Current vel: \n' + str(current_velocities)
    # print ' fl: ' + str(fl) + ' f: ' + str(f) + ' fr: ' + str(fr) +\
    #       ' rl: ' + str(rl) + ' rr: ' + str(rr) + ' lane: ' + str(lane) + '\n'
    return observations


# def get_observations(current_status, nr_vehicles, vehicle_nr):
#
#     observation_shape = (2, 3)
#
#     observation_space = np.ones(observation_shape)
#     # print 'observation space : \n ' + str(observation_space)
#     observation_space = observation_space * -100
#
#     # print 'observation space : \n ' + str(observation_space)
#
#     observation_space[0, 1] = 0
#
#     # print 'observation space : \n ' + str(observation_space)
#
#     current_poses = current_status[0]
#     current_velocities = current_status[1]
#     # print 'current_velocities : \n ' + str(current_velocities)
#
#     my_pos = current_poses[vehicle_nr]
#
#     for index_node in range(nr_vehicles):
#         if index_node == vehicle_nr:
#             continue
#         veh_pos = current_poses[index_node]
#         # print 'veh n : ' + str(index_node) + ' Vel : ' + str(current_velocities[index_node][1])
#         # print 'curr pos : ' + str(veh_pos) + ' my pos : ' + str(my_pos)
#         difference = veh_pos - my_pos
#
#         # print 'Difference: ' + str(difference)
#
#         x = 1 + difference[0]
#         y = difference[1]
#
#         # print 'X : ' + str(x) + ' Y : ' + str(y)
#
#         if x < 3 and y < 2:
#             if x >= 0 and y >= 0:
#                 observation_space[y, x] = current_velocities[index_node][1]
#
#     # print 'observation space : \n ' + str(observation_space)
#
#     # print 'my pos : ' + str(my_pos)
#
#     # if my_pos[1] == 0:
#     #    observation_space[1, :] = 100
#     if my_pos[1] == 20:
#         observation_space[1, :] = 100
#     if my_pos[0] == 0:
#         if my_pos[1] >= 15:
#             observation_space[1, 1:2] = 100
#         observation_space[:, 0] = 100
#     if my_pos[0] == 1:
#         observation_space[:, 2] = 100
#
#         # print 'observation space : \n ' + str(observation_space)
#         #
#         # if difference[1] == 0:
#         #     if difference[0] == 1:
#         #         observation_space[1, 0] = current_velocities[index_node]
#         #     elif difference[0] == -1:
#         #         observation_space[1, 2] = current_velocities[index_node]
#         #     elif difference[0] == 0:
#         #         observation_space[1, 1] = current_velocities[index_node]
#         # if difference[1] == -1:
#         #     if difference[0] == 1:
#         #         observation_space[2, 0] = current_velocities[index_node]
#         #     elif difference[0] == -1:
#         #         observation_space[2, 2] = current_velocities[index_node]
#         #     elif difference[0] == 0:

#         #         observation_space[2, 1] = current_velocities[index_node]
#         # if difference[1] == 1:
#         #     if difference[0] == 1:
#         #         observation_space[0, 0] = current_velocities[index_node]
#         #     elif difference[0] == -1:
#         #         observation_space[0, 2] = current_velocities[index_node]
#         #     elif difference[0] == 0:
#         #         observation_space[0, 1] = current_velocities[index_node]
#         #
#     # print 'observation space : \n ' + str(observation_space)
#
#     return observation_space


def get_observation_space(possible_speeds=[-100, 0, 100], possible_lanes=[0, 1]):
    possible_states = [np.array([[fl, f, fr, rl, rr, lane, vel]])
                       for fl in possible_speeds
                       for f in possible_speeds
                       for fr in possible_speeds
                       for rl in possible_speeds
                       for rr in possible_speeds
                       for lane in possible_lanes
                       for vel in [0, 1, 2, 100]]
    #
    print 'observation space : \n ' + str(possible_states)
    print 'observation space size: \n ' + str(len(possible_states))

    return possible_states


def get_initial_policy(possible_states, possible_speeds=[-100, 0, 100], n_actions=7):

    # possible_states = [np.array([[a, b, c], [d, e, f]])
    #                    for a in possible_speeds
    #                    for b in [0, 1, 2]
    #                    for c in possible_speeds
    #                    for d in possible_speeds
    #                    for e in possible_speeds
    #                    for f in possible_speeds]

    observation_shape = (len(possible_states), n_actions)

    level_0_policy = np.zeros(observation_shape)
    print 'polocy size: ' + str(len(level_0_policy))
    # print 'Previous: '
    # print level_0_policy

    # prob_dist_actions = np.array([prob_left, prob_accel, prob_remain, prob_brake])
    index = 0
    for fl in possible_speeds:
        for f in possible_speeds:
            for fr in possible_speeds:
                for rl in possible_speeds:
                    for rr in possible_speeds:
                        for lane in [0, 1]:
                            for vel in [0, 1, 2, 100]:
                                prob_right = 0.0
                                prob_left = 0.0
                                prob_accel = 0.0
                                prob_remain = 1.0
                                prob_brake = 0.0
                                prob_hard_brake = 0.0
                                prob_hard_accel = 0.0
                                if fl == -100:
                                    if rl == -100:
                                        prob_left = 1.0
                                        prob_remain = 0.0
                                        # print ' fl: ' + str(fl) + ' f: ' + str(f) + ' fr: ' + str(fr) +\
                                        #       ' rl: ' + str(rl) + ' rr: ' + str(rr) + ' lane: ' + str(lane) + '\n' +\
                                        #       'Left!'
                                        # if rl == 0:
                                        #     prob_left = 1.0
                                        #     prob_remain = 0.0
                                        # print ' fl: ' + str(fl) + ' f: ' + str(f) + ' fr: ' + str(fr) + \
                                        #       ' rl: ' + str(rl) + ' rr: ' + str(rr) + ' lane: ' + str(lane) + '\n' + \
                                        #       'Left!'
                                if f == 0 or f == 100:
                                    prob_brake = 1.0
                                    prob_remain = 0.0
                                    # if rl == -100:
                                    #     prob_left = 1.0
                                    #     prob_remain = 0.0
                                    # print ' fl: ' + str(fl) + ' f: ' + str(f) + ' fr: ' + str(fr) + \
                                    #       ' rl: ' + str(rl) + ' rr: ' + str(rr) + ' lane: ' + str(lane) + '\n' + \
                                    #       'Left!'
                                    # else:
                                    #     prob_brake = 1.0
                                    #     prob_remain = 0.0
                                    # print ' fl: ' + str(fl) + ' f: ' + str(f) + ' fr: ' + str(fr) + \
                                    #       ' rl: ' + str(rl) + ' rr: ' + str(rr) + ' lane: ' + str(lane) + '\n' + \
                                    #       'Brake'
                                if vel == 0 and f == -100:
                                    prob_accel = 1.0
                                    prob_remain = 0.0

                                prob_dist_actions = np.array([prob_left,
                                                              prob_right,
                                                              prob_accel,
                                                              prob_remain,
                                                              prob_brake,
                                                              prob_hard_brake,
                                                              prob_hard_accel])
                                # print index
                                # print prob_dist_actions
                                level_0_policy[index, :] = prob_dist_actions
                                index += 1

    # print 'After: '
    # print level_0_policy

    return level_0_policy



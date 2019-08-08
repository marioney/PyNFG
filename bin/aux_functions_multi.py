#!/usr/bin/env python


import numpy as np

try:
    import cPickle as pickle
except ImportError:
    import pickle

from math import ceil
###########################################
# PARAMETERS AND FUNCTIONS
###########################################
# boundaries of the grid


south = 0
west = 0
east = 1
goal_lane = 1
n_lanes = 2

# distance to Blocked (end of lane)
dis_blocked = 25

# time steps
max_time = 10

# Maximum velocity
max_vel = 2

# Vehicle length
veh_len = 5

# Min Gap
min_gap = 2.0

# Number of players
nr_vehicles = 4  # ego-vehicle + 3 other cars


def return_goal_lane():
    global goal_lane
    return goal_lane


def get_starting_status():
    # Starting positions
    # Ego-vehicle starts at west south
    global goal_lane
    # print 'new_pose'
    pose_ind = 0
    reset_count = 0
    accepted_gap = np.random.randint(veh_len + 3, size=1) + 1.0
    # print 'accepted_gap :  ' + str(accepted_gap)
    while pose_ind < nr_vehicles:
        new_pose = np.array([0, 0])
        # print 'new_pose' + str(new_pose)
        if pose_ind is 0:
            # starting_poses = new_pose
            new_pose[0] = np.random.randint(n_lanes, size=1)
            new_pose[1] = np.random.randint(south + veh_len, size=1)
            pose_ind += 1
            starting_poses = np.array([new_pose[0], 15 + new_pose[1]])
            if new_pose[0] == 1:
                goal_lane = 0
            elif new_pose[0] == 0:
                goal_lane = 1
        else:
            new_pose[0] = goal_lane
            new_pose[1] = np.random.randint(south + 3.5 * (veh_len + accepted_gap), size=1)
            if not check_collision(new_pose, starting_poses, accepted_gap):
                starting_poses = np.vstack((starting_poses, new_pose))  # axis=0))
                pose_ind += 1
            else:
                reset_count += 1
                if reset_count > 1000:
                    # print 'reset poses'
                    pose_ind = 0

        # print 'Pose index : ' + str(pose_ind)
    # print 'Starting_poses :  \n ' + str(starting_poses)
    # print 'Goal lane :  ' + str(goal_lane)

    starting_velocities = np.array([0, 1.0])
    pose_ind = 1

    while pose_ind < nr_vehicles:
        new_vel = np.array([0, 1.0])
        # print 'new_velocity' + str(new_vel)
        starting_velocities = np.vstack((starting_velocities, new_vel))  # axis=0))
        pose_ind += 1
        # print 'Pose index : ' + str(pose_ind)

    # print 'Starting velocities : \n ' + str(starting_velocities)

    current_status = [starting_poses, starting_velocities]
    # print 'CurrStatus'
    return current_status


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


def check_col(current_pose, other_pose, allowed_gap):
    d_front = distance_to_front(current_pose, other_pose)
    d_back = distance_to_back(current_pose, other_pose)
    if d_front < veh_len + allowed_gap:
        return True
    elif d_back < veh_len + allowed_gap:
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


def check_collision(current_pose, pose_array, allowed_gap=0.5):
    """
    :param current_pose: Pose being evaluated
    :param pose_array:   List of occupied poses.
    :param allowed_gap: Minimum allowed gap
    :return: True if a collision is found, False otherwise
    """
    # print 'pose array shape' + str(pose_array.shape)
    if pose_array is None:
        return True
    if pose_array.ndim is 1:
        if check_col(current_pose, pose_array, allowed_gap) is True:
            # print ' Collission in pose' + str(current_pose)
            return True
    else:
        [rows, cols] = pose_array.shape
        if cols is not 2:
            print 'pose array should have 2 cols - it has ' + str(cols)
            return True

        for index_compare in range(0, rows):
            if check_col(current_pose, pose_array[index_compare], allowed_gap) is True:
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


def read_training_values():

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


def store_training_values(game):

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


def get_observations(current_status, vehicle_nr):

    current_poses = current_status[0]
    current_velocities = current_status[1]
    my_vel = current_velocities[vehicle_nr][1]
    my_pos = current_poses[vehicle_nr]

    # print '\n NEW OBSERVATION \n'
    # print 'current_velocities : \n ' + str(current_velocities)
    # print 'My vel : \n ' + str(my_vel)

    fl = -1
    f = -1
    fr = -1
    cl = -1
    cr = -1
    rl = -1
    rr = -1
    bl = -1
    br = -1
    dgoal = 0

    current_lane = my_pos[1]
    diff_lane = goal_lane - current_lane

    # Compute lane observation
    if diff_lane is 0:
        lane = 0
    elif diff_lane > 0:
        lane = 1
    else:
        lane = -1

    # Compute dgoal observation

    current_pos_x = my_pos[0]
    diff_goal = dis_blocked - current_pos_x

    # Compute lane observation
    if diff_goal < 0:
        dgoal = 0
    elif 0 < diff_goal < 10:
        dgoal = 1
    elif 10 < diff_goal < 20:
        dgoal = 2
    elif 20 < diff_goal < 30:
        dgoal = 3
    elif 30 < diff_goal < 40:
        dgoal = 4
    elif 40 < diff_goal < 50:
        dgoal = 4

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

        # print("front ", self.observations.front)

        if x == -1:
            if 4.0 <= y <= 9.0:
                if current_velocities[index_node][1] > fr:
                    fr = 1
            if 0.0 <= y < 5.0:
                if current_velocities[index_node][1] > cr:
                    cr = 1
            if -4.0 <= y < 1.0:
                if current_velocities[index_node][1] > rr:
                    rr = 1
            if -8.0 <= y < -3.0:
                if current_velocities[index_node][1] > br:
                    br = 1

        if x == 0:
            if 4.0 <= y <= 9.0:
                if current_velocities[index_node][1] > f:
                    f = 1

        if x == 1:
            if 4.0 <= y <= 9.0:
                if current_velocities[index_node][1] > fl:
                    fl = 1
            if 0.0 <= y < 5.0:
                if current_velocities[index_node][1] > cl:
                    cl = 1
            if -4.0 <= y < 1.0:
                if current_velocities[index_node][1] > rl:
                    rl = 1
            if -8.0 <= y < -3.0:
                if current_velocities[index_node][1] > bl:
                    bl = 1

    # print 'observation space : \n ' + str(observation_space)

    # print 'my pos : ' + str(my_pos)

    # if my_pos[1] == 0:
    #    observation_space[1, :] = 100
    # if my_pos[1] == north:
    #     fl = 100
    #     fr = 100
    #     f = 100

    observations = np.array([[fl, f, fr, cl, cr, rl, rr, bl, br, dgoal, lane]])
    # print 'Vehicle nr : ' + str(vehicle_nr) + ' pos : ' + str(my_pos)
    # print 'my vel : ' + str(my_vel)
    # print 'Current poses: \n' + str(current_poses)
    # print 'Current vel: \n' + str(current_velocities)
    # print ' fl: ' + str(fl) + ' f: ' + str(f) + ' fr: ' + str(fr) +\
    #       ' rl: ' + str(rl) + ' rr: ' + str(rr) + ' vel: ' + str(vel) + '\n'
    return observations


def get_observation_space(possible_speeds=[-1, 1],
                          possible_lanes=[-1, 0, 1],
                          possible_dgoal=[0, 1, 2, 3, 4]):
    # possible_states = [np.array([[fl, f, fr, rl, rr, bl, br, vel]])
    possible_states = [np.array([[fl, f, fr, cl, cr, rl, rr, bl, br, dgoal, lane]])
                       for fl in possible_speeds
                       for f in possible_speeds
                       for fr in possible_speeds
                       for cl in possible_speeds
                       for cr in possible_speeds
                       for rl in possible_speeds
                       for rr in possible_speeds
                       for bl in possible_speeds
                       for br in possible_speeds
                       for dgoal in possible_dgoal
                       for lane in possible_lanes]
    #
    print 'observation space : \n ' + str(possible_states)
    print 'observation space size: \n ' + str(len(possible_states))
    return possible_states


def get_initial_policy(possible_states,
                       possible_speeds=[-1, 1],
                       possible_lanes=[-1, 0, 1],
                       possible_dgoal=[0, 1, 2, 3, 4],
                       n_actions=5):

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
    for fl in possible_speeds:
        for f in possible_speeds:
            for fr in possible_speeds:
                for cl in possible_speeds:
                    for cr in possible_speeds:
                        for rl in possible_speeds:
                            for rr in possible_speeds:
                                for bl in possible_speeds:
                                    for br in possible_speeds:
                                        for dgoal in possible_dgoal:
                                            for lane in possible_lanes:
                                                prob_right = 0.0
                                                prob_left = 0.0
                                                prob_accel = 0.0
                                                prob_remain = 0.0
                                                prob_brake = 0.0
                                                if lane == 0:
                                                    if f == 1:
                                                        prob_brake = 1.0
                                                    elif f == -1:
                                                        prob_accel = 1.0
                                                    else:
                                                        prob_remain = 1.0
                                                elif lane == 1:
                                                    if rl == -1 and cl == -1:
                                                        prob_left = 1.0
                                                    else:
                                                        if f == 1:
                                                            prob_brake = 1.0
                                                        else:
                                                            prob_remain = 1.0
                                                elif lane == -1:
                                                    if rr == -1 and cr == -1:
                                                        prob_right = 1.0
                                                    else:
                                                        if f == 1:
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

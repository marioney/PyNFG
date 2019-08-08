#!/usr/bin/env python

try:
    import cPickle as pickle
except ImportError:
    import pickle


###########################################
# PARAMETERS AND FUNCTIONS
###########################################
# boundaries of the grid

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


def check_col(current_pose, other_pose, allowed_gap):
    d_front = distance_to_front(current_pose, other_pose)
    d_back = distance_to_back(current_pose, other_pose)
    if d_front < allowed_gap:
        return True
    elif d_back < allowed_gap:
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


def store_training_values(game, nr_vehicles):

    for index_node in range(nr_vehicles):
        car_name = 'Dcar' + str(index_node)
        print 'Training: ' + car_name + 'at level 2'
        file_name = 'Training' + car_name + '.pickle'
        pickle_out = open(file_name, "wb")
        pickle.dump(game.bn_part[car_name][0].CPT, pickle_out)
        # print ' 0 '
        # print game.bn_part[car_name][0].CPT
        pickle_out.close()
#!/usr/bin/env python

# -*- coding: utf-8 -*-
"""
Implements a simple merge lane with iterSemiNFG:

In each time step, each player is at a location
on a grid. The seeker gets a point each time both players are at the same
location. The hider loses a point when this happens. In each time step, players
each make an observation (possibly noisy) about their opponent's location and
choose one of the five following moves: up, down, left, right, stay. These moves
result in an associated location change of one grid step. A player on the
"northern" boundary that chooses up remains as is. Similiar rules apply to
other moves that would result in locations off the grid.

Note: It is better to run this script line by line or customize your own run
script from the pieces contained herein rather than running the entire file. The
reason is that the PGT and RL algorithms will take a long time.

Created on Mon Jan 28 16:22:43 2013

Copyright (C) 2013 James Bono

GNU Affero General Public License

"""
from __future__ import division

from pynfg_ros import DeterNode, ChanceNode, DecisionNode
from pynfg_ros.levelksolutions.mcrl import *
from pynfg_ros.pgtsolutions.intelligence.policy import *

from aux_functions_nb import *


###########################################
# PARAMETERS AND FUNCTIONS
###########################################
# boundaries of the grid
west = 0
east = 1
north = 20
south = 0

# Blocked cells from the north
blocked_north = 15.5

# Blocked cells from the west
blocked_west = 1

# time steps
max_time = 10

# Maximum velocity
max_vel = 1

# Vehicle length
veh_len = 2

# Min Gap
min_gap = 1

# Number of players
nr_vehicles = 4  # ego-vehicle + 3 other cars

# actions of the players
left = np.array([1, 0])
right = np.array([-1, 0])
accel = np.array([0, 0.5])
remain = np.array([0, 0])
brake = np.array([0, -1])
# hard_brake = np.array([0, -2])
# hard_accel = np.array([0, 2])


# Probabilities of other vehicles actions (must sum 1)

prob_right = 0.05
prob_left = 0.1
prob_accel = 0.2
prob_remain = 0.4
prob_brake = 0.25
# prob_hard_brake = 0.0
# prob_hard_accel = 0.0


# Probability distribution
prob_dist_actions = np.array([prob_left,
                              prob_right,
                              prob_accel,
                              prob_remain,
                              prob_brake])

# space of actions that players can choose
action_space = [left, right, accel, remain, brake]

training_values = read_training_values(nr_vehicles)

observation_space = get_observation_space()

# Starting positions

# # new_pose = np.array([east, south])
# # starting_poses = np.vstack((starting_poses, new_pose))  # axis=0))
# pose_ind = 0
# while pose_ind < nr_vehicles-1:
#     new_pose = np.array([1, pose_ind * 1.5])
#     starting_poses = np.vstack((starting_poses, new_pose))  # axis=0))
#     # new_pose = np.array([0, pose_ind * 1.5])
#     # starting_poses = np.vstack((starting_poses, new_pose))  # axis=0))
#     pose_ind += 1
#     # new_pose[0] = np.random.randint(0, high=east + 1, size=1)
#     # new_pose[1] = np.random.randint(0, high=south + int((nr_vehicles + 1)), size=1)
#     print 'new_pose' + str(new_pose)
#
#     # if not check_collision(new_pose, starting_poses):
#     # pose_ind += 1
#     print 'Pose index : ' + str(pose_ind)
#
# print 'Starting_poses :  \n ' + str(starting_poses)
#
# # Ego-vehicle starts at [0, 0]
# starting_velocities = np.array([0, 0.5])
# pose_ind = 1
# while pose_ind < nr_vehicles:
#     new_vel = np.array([0, 0.5])
#     print 'new_velocity' + str(new_vel)
#     starting_velocities = np.vstack((starting_velocities, new_vel))  # axis=0))
#     pose_ind += 1
#     # print 'Pose index : ' + str(pose_ind)
#
# print 'Starting velocities : \n ' + str(starting_velocities)
#
# current_status = [starting_poses, starting_velocities]


def get_starting_status():
    # Starting positions
    # Ego-vehicle starts at west south
    # Ego-vehicle starts at west south
    # print 'get Starting status!'
    starting_poses = np.array([west, round(np.random.uniform(1.0, 2.9), 2)])
    pose_ind = 0
    while pose_ind < nr_vehicles - 1:
        new_pose = np.array([0.0, 0.0])
        new_pose[0] = east
        new_pose[1] = round(np.random.uniform(0.2, 4.5), 2)
        # np.random.randint(south + 3, size=1)
        # print 'new_pose 1' + str(new_pose)
        if not check_collision(new_pose, starting_poses):
            starting_poses = np.vstack((starting_poses, new_pose))
            pose_ind += 1
            # print 'Pose index : ' + str(pose_ind)
    # print 'Starting_poses :  \n ' + str(starting_poses)

    # Ego-vehicle starts at [0, 0]
    starting_velocities = np.array([0, 0.5])
    pose_ind = 1

    while pose_ind < nr_vehicles:
        new_vel = np.array([0, 0.5])
        # print 'new_velocity' + str(new_vel)
        starting_velocities = np.vstack((starting_velocities, new_vel))  # axis=0))
        pose_ind += 1
        # print 'Pose index : ' + str(pose_ind)

    # print 'Starting velocities : \n ' + str(starting_velocities)

    current_status = [starting_poses, starting_velocities]
    # print 'Current status :  \n ' + str(current_status)

    return current_status


#  print 'All observation space : \n ' + str(observation_space)

initial_policy = get_initial_policy(observation_space)


def adjust_loc(location):
    """"
     function that adjusts for moves off the grid
    """

    if location[0] < west:
        location[0] = west

    elif location[0] > east:
        location[0] = east

    if location[1] < south:
        location[1] = south
    # elif location[1] > north:
    #     location[1] = north

    if location[0] == west:
        if location[1] > north - blocked_north:
            location[1] = north - blocked_north

    return location


def get_node_observation(**keyword_args):
    """
    Combines start loc. and moves to give new loc.
    Used by DeterNode F
    :param :param keyword_args: multiple key-value arguments, namely previous location and
                        observational noise of all other vehicles.
    :return:
    """

    # print "\n Previous Observation!!"
    obs_noise = []
    # new_poses = np.array([0, 0])
    car_number = -1  # for index_node in range(nr_vehicles):
    previous_status = None
    for keyword, value in keyword_args.iteritems():
        # print "%s = %s" % (keyword, value)
        if keyword is 'previous_status':
            previous_status = value
        elif keyword is 'car_nr':
            car_number = value
        else:
            obs_noise.append(value)
    if car_number is -1:
        print 'No car number specified'
        return None
    if previous_status is None:
        print 'No previous status specified'
        return None

    observation = get_observations(previous_status, nr_vehicles, car_number, north, east, west, blocked_north)
    #
    # print '\nCurrent Observation for car ' + str(car_number) + ' at pos ' + str(previous_status[0][car_number])
    # print ' fl: ' + str(observation[0][0]) + ' f: ' + str(observation[0][1]) + ' fr: ' + str(observation[0][2]) + \
    #       ' rl: ' + str(observation[0][3]) + ' rr: ' + str(observation[0][4]) + ' vel: ' + str(observation[0][5])
    # print 'new_loc'
    # print 'Observation for car Nr ' + str(car_number) + '\n' + str(observation)
    return observation


def compute_new_status(**keyword_args):
    """
    Combines start loc. and moves to give new loc.
    Used by DeterNode F
    :param :param keyword_args: multiple key-value arguments, namely previous location and
                        observational noise of all other vehicles.
    :return:
    """

    # print "\n \n Compute new status!! \n"

    car_movements = []
    car_numbers = []
    # new_poses = np.array([0, 0])
    # new_poses = []
    previous_status = None
    for keyword, value in keyword_args.iteritems():
        # print "%s = %s" % (keyword, value)
        if keyword is 'previous_status':
            previous_status = value
            # print "%s = %s" % (keyword, value)
        else:
            # print "%s = %s" % (keyword, value)
            player_nr = int(keyword[5:])
            # print 'Car Number : ' + str(player_nr)
            car_movements.append(value)
            car_numbers.append(player_nr)

    if previous_status is None:
        previous_status = get_starting_status()
        # print "No prev status - Return initial pose"
        # print "Current status \n" + str(previous_status)
        return previous_status

    # print "\n Prev status \n" + str(previous_status[0])
    # print "Car Movements \n" + str(car_movements)
    # print "Car Numbers \n" + str(car_numbers)
    for movement_index in range(nr_vehicles):
        # print "Car nr %s" % movement_index
        # print "Prev pos %s - prev vel %s " % (previous_status[0][movement_index],
        #                                       previous_status[1][movement_index])
        car_move = remain
        if len(car_movements) is not 0:
            # Remain as default action
            car_move = remain
        else:
            for player_index in range(nr_vehicles):
                if car_numbers[player_index] == movement_index:
                    car_move = car_movements[player_index]

        # print "Car move " + str(car_move)
        temp_vel = car_move + previous_status[1][movement_index]
        # print "Temp vel prev " + str(temp_vel)
        if temp_vel[1] < 0:
            temp_vel[1] = 0
        if temp_vel[1] > max_vel:
            temp_vel[1] = max_vel
        # print "Temp vel" + str(temp_vel)
        temp_pose = adjust_loc(temp_vel + previous_status[0][movement_index])
        # print "New pose" + str(temp_pose)
        temp_vel[0] = 0
        if movement_index is 0:
            new_status = [temp_pose, temp_vel]
        else:
            new_poses = np.vstack((new_status[0], temp_pose))
            new_velocities = np.vstack((new_status[1], temp_vel))
            new_status = [new_poses, new_velocities]
        # print "New pos %s" % new_poses[movement_index]

    # print "\n New status \n" + str(new_status[0])
    return new_status


def estimate_new_obs(**keyword_args):
    """
    Combines observational noise and previous locations of cars to give a valid location on-grid
    for the car's observation of the position of the other cars
    :param keyword_args: multiple key-value arguments, namely previous location and
                        observational noise of all other vehicles.
    :return:
    """
    # print "\n Estimate new Observation!! \n"
    estmated_actions = []
    car_numbers = []
    # new_poses = np.array([0, 0])
    car_number = -1  # for index_node in range(nr_vehicles):
    previous_status = None
    for keyword, value in keyword_args.iteritems():
        # print "%s = %s" % (keyword, value)
        if keyword is 'previous_status':
            previous_status = value
        elif keyword is 'car_nr':
            car_number = value
            # print "Obtaining estimation for car %s " % value
        else:
            # print "Estimated action %s = %s" % (keyword, value)
            # print 'Car Number : ' + str(player_nr)
            player_nr = int(keyword[5:])
            car_numbers.append(player_nr)
            estmated_actions.append(value)

    if car_number is -1:
        print 'No car number specified'
        return None

    # print "Received status \n" + str(previous_status)
    # print 'Car observations'
    index_obs = 0
    for movement_index in range(nr_vehicles):
        # print 'Car ' + str(movement_index)

        if movement_index is car_number:
            # print 'Own car'
            temp_vel = previous_status[1][movement_index]
            temp_pose = previous_status[0][movement_index]
            # print 'Car pos ' + str(temp_pose)
        else:
            # print "Prev pos %s - Estimated action %s" % (
            #    previous_status[0][movement_index], estmated_actions[index_obs])
            for player_index in range(nr_vehicles - 1):
                if car_numbers[player_index] == movement_index:
                    car_move = estmated_actions[player_index]
                    #  car_move = estmated_actions[index_obs]
            temp_vel = car_move + previous_status[1][movement_index]
            if temp_vel[1] < 0:
                temp_vel[1] = 0
            if temp_vel[1] > max_vel:
                temp_vel[1] = max_vel
            # print "Temp vel" + str(temp_vel)
            # print "Temp vel" + str(temp_vel)
            temp_pose = adjust_loc(temp_vel + previous_status[0][movement_index])
            # print "Temp pose" + str(temp_pose)
            index_obs += 1
        if movement_index is 0:
            new_status = [temp_pose, temp_vel]
        else:
            new_poses = np.vstack((new_status[0], temp_pose))
            new_velocities = np.vstack((new_status[1], temp_vel))
            new_status = [new_poses, new_velocities]
        # print "New pos %s" % new_poses[movement_index]

    # print 'new_loc'
    # print "Estimated New status \n" + str(new_status[0])

    observation = get_observations(new_status, nr_vehicles, car_number, north, east, west, blocked_north)

    # print 'Estimated Observation'
    # print ' fl: ' + str(observation[0][0]) + ' f: ' + str(observation[0][1]) + ' fr: ' + str(observation[0][2]) +\
    #       ' rl: ' + str(observation[0][3]) + ' rr: ' + str(observation[0][4]) + ' lane: ' + str(observation[0][5])

    return observation


##################################################
# THE NODES - first time 0, then 1 to T in a loop
##################################################

params_f = {}
continuous_f = False
continuous_t = True

#  movement_node =
#  DeterNode('Root0', new_localization, params_f, continuous_f, space=state_space, basename='Root', time=0)
movement_node = DeterNode('Root0', compute_new_status, params_f, continuous_t, basename='Root', time=0)
# Observational noise, does not have a parent node
# Create an observational noise for each car : ChanceNode CcarX, no parents

node_set = set([movement_node])
# BUILD time steps 0 to T-1 iteratively
for time_step in range(0, max_time):

    # Makes decision based on status (paren FSeek or FHide)
    obs_nodes_prev = []
    # param_obs_prev = {'previous_status': movement_node}
    for index_node in range(nr_vehicles):
        node_basename = 'car' + str(index_node)
        node_name = node_basename + str(time_step)
        param_obs_prev = {'car_nr': index_node, 'previous_status': movement_node}
        obs_nodes_prev.append(DeterNode('O' + node_name, get_node_observation, param_obs_prev, continuous_f,
                                        space=observation_space, basename='O' + node_basename, time=time_step))

    # Observational noise, does not have a parent node
    chance_nodes = []
    for index_node in range(nr_vehicles):
        # Create an observational noise for each car : ChanceNode CcarX, no parents
        if training_values is not None:
            obsnoiseCPT = training_values[index_node]
            par_obs = [obs_nodes_prev[index_node]]

        else:
            obsnoiseCPT = prob_dist_actions
            par_obs = []

            # obsnoiseCPT = initial_policy
            # par_obs = [obs_nodes_prev[index_node]]
        CPTi = (obsnoiseCPT, par_obs, action_space)
        node_basename = 'car' + str(index_node)
        node_name = node_basename + str(time_step)
        # print 'Add node' + node_name
        chance_nodes.append(ChanceNode('C' + node_name, CPTip=CPTi, basename='C' + node_basename, time=time_step))

    # Mixes movement (node F) with observational noise (node Cseek)
    obs_car_nodes = []
    for index_node in range(nr_vehicles):
        node_basename = 'car' + str(index_node)
        node_name = node_basename + str(time_step)
        # param_obs = {'loc': root_node, 'noise': chance_nodes, 'car_nr': index_node}
        param_obs = {'car_nr': index_node, 'previous_status': movement_node}  # , 'decision_nodes': car_decision_nodes}

        for index_chance_node in range(nr_vehicles):
            # adding the other nodes
            if index_chance_node is not index_node:
                # print 'Chance node ' + str(index_chande_node)
                param_obs['Cnode' + str(index_chance_node)] = chance_nodes[index_chance_node]
        #  obs_car_nodes.append(DeterNode('O' + node_name, adjust_car_obs, param_obs, continuous_f,
        #                               space=state_space, basename='O' + node_basename, time=time_step))
        obs_car_nodes.append(DeterNode('EO' + node_name, estimate_new_obs, param_obs, continuous_f,
                                       space=observation_space, basename='EO' + node_basename, time=time_step))

    # Makes decision based on status (paren FSeek or FHide)
    car_decision_nodes = []
    for index_node in range(nr_vehicles):
        node_basename = 'car' + str(index_node)
        node_name = node_basename + str(time_step)
        car_decision_nodes.append(DecisionNode('D' + node_name, node_basename, action_space,
                                               parents=[obs_car_nodes[index_node]],
                                               basename='D' + node_basename, time=time_step))

    # Moves according to decision (Parent, previous position and decision on this timestep)
    params_loc = {'previous_status': movement_node}  # , 'decision_nodes': car_decision_nodes}

    for index_node in range(nr_vehicles):
        # adding the other nodes
        # print 'Dec node ' + str(index_node)
        params_loc['Dnode' + str(index_node)] = car_decision_nodes[index_node]

    # print params_loc
    movement_node = DeterNode('F%s' % time_step, compute_new_status, params_loc,
                              continuous_t, basename='F', time=time_step)

    # movement_node = DeterNode('F%s' % time_step, new_localization, params_loc,
    #                           continuous_f, space=state_space, basename='F', time=time_step)

    for index_node in range(nr_vehicles):
        # adding the other nodes
        # print 'Add node: ' + str(chance_nodes[index_node])
        # print 'Add node: ' + str(obs_car_nodes[index_node])
        # print 'Add node: ' + str(car_decision_nodes[index_node])
        node_set.update([obs_nodes_prev[index_node], chance_nodes[index_node],
                         obs_car_nodes[index_node], car_decision_nodes[index_node]])
    # adding new set of nodes from time step t to nodeset
    node_set.update([movement_node])


##########################
# REWARD FUNCTIONS
##########################

def get_reward(F, player, decision):
    """
    seeker's reward function
    :param F: Node with status of all players
    :param player: player number
    :param decision: decision taken by player
    :return:
    """

    player_nr = int(player[3:])
    # print 'Player Number : ' + str(player_nr)

    # reward = 0
    # F = None

    # print '\n Decision : ' + str(decision) + '\n'

    position_player = F[0][player_nr]
    # print 'Position player \n' + str(position_player)
    reward = 0  # position_player[1]  # * position_player[1]  # position_player[1]

    if position_player[0] == east:
        if position_player[1] > north - blocked_north - 1.5:
            reward += 5*position_player[1]  # position_player[1]  # position_player[1]
        if np.array_equal(decision, left) is True:
            reward = -350

    if position_player[0] == west:
        if np.array_equal(decision, right) is True:
            reward = -350
        if position_player[1] >= north - blocked_north - 1:
            reward = -350

    f_not_player = None
    for f_b in range(len(F)):
        # print 'Item  : ' + str(f_b)
        if not f_b == player_nr:
            if f_not_player is None:
                f_not_player = F[0][f_b]
            else:
                f_not_player = np.vstack((f_not_player, F[0][f_b]))

    if np.array_equal(position_player, np.array([east, north])) is False:
        if check_collision(position_player, f_not_player):
            # print 'Collision!'
            reward = -350

    # if reward > 0:
    #     sys.stdout.write('\n')
    #     sys.stdout.write('  pla : ' + str(player_nr) +
    #                      '  pos : ' + str(position_player) +
    #                      '  Dec : ' + str(decision) +
    #                      '  Rew : ' + str(reward))
    # print 'Reward ' + str(reward)

    return reward


def car_reward_0(F, player, Dcar0):
    """
        seeker's reward function
        :param F: Node with status of all players
        :param player: player number
        :return:
    """
    return get_reward(F, player, Dcar0)


def car_reward_1(F, player, Dcar1):
    """
        seeker's reward function
        :param F: Node with status of all players
        :param player: player number
        :return:
    """

    return get_reward(F, player, Dcar1)


def car_reward_2(F, player, Dcar2):
    """
        seeker's reward function
        :param F: Node with status of all players
        :param player: player number
        :return:
    """
    return get_reward(F, player, Dcar2)


def car_reward_3(F, player, Dcar3):
    """
        seeker's reward function
        :param F: Node with status of all players
        :param player: player number
        :return:
    """
    return get_reward(F, player, Dcar3)


def car_reward_4(F, player, Dcar4):
    return get_reward(F, player, Dcar4)


def car_reward_5(F, player, Dcar5):
    return get_reward(F, player, Dcar5)


def car_reward_6(F, player, Dcar6):
    return get_reward(F, player, Dcar6)


def car_reward_7(F, player, Dcar7):
    return get_reward(F, player, Dcar7)


def car_reward_8(F, player, Dcar8):
    return get_reward(F, player, Dcar8)


def car_reward_9(F, player, Dcar9):
    return get_reward(F, player, Dcar9)


# rewards dictionary
reward_funcs = {}
for index_node in range(nr_vehicles):
    # adding the other nodes
    car_name = 'car' + str(index_node)
    func_name = 'car_reward_' + str(index_node)
    print 'Car name: ' + car_name
    print 'Func name: ' + func_name
    # reward_funcs[car_name] = func_na00,me
    reward_funcs[car_name] = locals()[func_name]
##################################
# CREATING THE iterSemiNFG
##################################
G = iterSemiNFG(node_set, reward_funcs)

# making a set of the names of the first two time steps for visualization
# draw_set = set([n.name for n in G.time_partition[0]]).union(set([n.name for n in G.time_partition[1]]))
# draw_set = set([n.name for n in G.time_partition[0]])
# G.draw_graph(draw_set)

# visualizing the first two time steps of the net

###########################################
# MANIPULATING CPTs
###########################################
# Giving nodes a uniform CPT
for index_node in range(nr_vehicles):
    # adding the other nodes
    decision_node_name = 'Dcar' + str(index_node)
    print 'Decision node name: ' + decision_node_name
    # print G.bn_part[decision_node_name][0].CPT
    # Create an observational noise for each car : ChanceNode CcarX, no parents
    if training_values is not None:
        G.bn_part[decision_node_name][0].CPT = training_values[index_node]
    else:
        G.bn_part[decision_node_name][0].randomCPT(mixed=True)
        # G.bn_part[decision_node_name][0].uniformCPT()
        # G.bn_part[decision_node_name][0].CPT = initial_policy

# pointing all CPTs to time 0 CPT
cptdict = G.get_decisionCPTs(mode='basename')
G.set_CPTs(cptdict)

# Generate the dictionary of inputs
N = 1000
mcrl_params = mcrl_dict(G, 1, np.linspace(1000, 1, N), N, 1, np.linspace(.5, 1, N),
                        np.linspace(.2, 1, N), L0Dist='uniform', pureout=True)
#
MCRL_solved = EwmaMcrl(G, mcrl_params)
MCRL_solved.solve_game(setCPT=True)


store_training_values(nr_vehicles, MCRL_solved.Game)
store_game(MCRL_solved, 'Game_stored_4_level3.pickle')


print 'try non solved'
valuedict = G.sample_timesteps(G.starttime, G.endtime, basenames=['Root', 'F',
                                                                  'Ocar0', 'Ocar1',
                                                                  'Ccar0', 'Ccar1',
                                                                  'EOcar0', 'EOcar1',
                                                                  'Dcar0', 'Dcar1'])
print valuedict['Root'][0][0]
for time_index in range(0, max_time):
    print 'time step ' + str(time_index)
    for vehicle_index in range(nr_vehicles):
        print 'pos: ' + str(valuedict['F'][time_index][0][vehicle_index]) \
              + ' - vel: ' + str(valuedict['F'][time_index][1][vehicle_index])

print 'try solved'
valuedict = MCRL_solved.Game.sample_timesteps(G.starttime, G.endtime, basenames=['Root', 'F'])
print valuedict['Root'][0][0]
for time_index in range(0, max_time):
    print 'time step ' + str(time_index)
    for vehicle_index in range(nr_vehicles):
        print 'pos: ' + str(valuedict['F'][time_index][0][vehicle_index]) \
              + ' -- vel: ' + str(valuedict['F'][time_index][1][vehicle_index])

print 'try solved'

valuedict = MCRL_solved.Game.sample_timesteps(G.starttime, G.endtime, basenames=['Root', 'F'])
print valuedict['Root'][0][0]
for time_index in range(0, max_time):
    print 'time step ' + str(time_index)
    for vehicle_index in range(nr_vehicles):
        print 'pos: ' + str(valuedict['F'][time_index][0][vehicle_index]) \
              + ' -- vel: ' + str(valuedict['F'][time_index][1][vehicle_index])

G1 = copy.deepcopy(MCRL_solved.Game)

############################################
# PGT INTELLIGENCE ESTIMATION
############################################


def captures(G):
    """
    defining a welfare metric on G
    :param G: Game to evaluate
    :return: Aveage number of captures
    """
    T0 = G.starttime
    T = G.endtime
    G.sample()
    num_captures = G.npv_reward('car0', T0, 1)
    # print '  Cap: %s' % str(num_captures) + '\r'
    return num_captures


def density(iq):
    """
    Defining a PGT posterior on iq profiles (dict)
    :param iq:
    :return:
    """
    x = iq.values()
    y = np.power(x, 2)
    z = np.prod(y)
    return z


GG = copy.deepcopy(G1)
# NOTE: the CPTs of G are seeds for MH and MC sampling

# number of samples
S = 20

# number of samples of utility of G in calculating iq
X = 10
# number of alternative strategies sampled in calculating iq
M = 20
# noise in the perturbations of G for MH or MC sampling
noise = .2
# satisfying distribution noise for iq calculations
innoise = noise
# number of draws to burn for MH
burn = 10


# tipoff = time.time()
# starting a timer

# Importance Samping estimation of PGT posterior
intelMC, funcoutMC, weightMC = policy_MC(GG, S, noise, X, M,
                                         innoise=.2,
                                         delta=1,
                                         integrand=captures,
                                         mix=False,
                                         satisfice=GG)
# halftime = time.time()
# print (halftime-tipoff)

# Metropolis-Hastings estimation of PGT posterior
# intelMH, funcoutMH, densMH = policy_MH(GG, S, density, noise, X, M,
#                                        innoise=.2,
#                                        delta=1,
#                                        integrand=captures,
#                                        mix=False,
#                                        satisfice=GG)
# buzzer = time.time()  # type: float

# Printing elapsed times
# T = halftime-tipoff
# print ('MC took:', T,  'sec., ', T/60, 'min., or', T/3600, 'hr.')
# T = buzzer-halftime
# print ('MH took:', T,  'sec., ', T/60, 'min., or', T/3600, 'hr.')

###########################################
# PLOTTING PGT RESULTS
###########################################
# creating the importance sampling weights from MC
MCweight = [density(intelMC[s])/np.prod(weightMC[s].values()) for s in xrange(1, S+1)]
# the PGT distributions over welfare values

fig1, ax1 = plt.subplots(1)
ax1.hist(funcoutMC.values(), weights=MCweight, alpha=.5)
# fig2, ax2 = plt.subplots(1)
# ax2.hist(funcoutMH.values()[burn::], alpha=.5)
plt.show()

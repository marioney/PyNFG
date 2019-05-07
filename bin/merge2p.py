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
from pynfg_ros.levelksolutions.mcrlGPU import *
from pynfg_ros.pgtsolutions.intelligence.policy import *
import numpy as np

from aux_functions import check_collision, read_training_values, store_training_values

###########################################
# PARAMETERS AND FUNCTIONS
###########################################
# boundaries of the grid
west = 0
east = 1
north = 4
south = 0

# Blocked cells from the north
blocked_north = 2

# Blocked cells from the west
blocked_west = 1

# actions of the players

left = np.array([-1, 1])
right = np.array([1, 1])
remain = np.array([0, 1])
brake = np.array([0, 0])

# Probabilities of other vehicles actions (must sum 1)
# Todo: Check how to find correct values for this.

#  prob_left = 0.15.00
prob_right = 0.33
prob_remain = 0.33
prob_brake = 1 - prob_remain - prob_right

# space of actions that players can choose
action_space = [right, remain, brake]

# time steps
max_time = 6

# Number of players
nr_vehicles = 3  # egovehicle + 3 other cars

# the space for the state nodes below, Froot and F
state_space = []

pose_ind = 0

possible_states = [np.array([w, x])
                   for w in range(east + 1) for x in range(north - blocked_north + 1)]

blocked_states = [np.array([w, x])
                  for w in range(west + blocked_west, east + 1) for x in range(north - blocked_north + 1, north + 1)]

possible_states = np.vstack((possible_states, blocked_states))

print 'Possible states: \n ' + str(possible_states)

training_values = read_training_values(nr_vehicles)

for vehicle_index in range(nr_vehicles):
    print 'Adding sates of vehice : ' + str(vehicle_index)
    print 'length of space of states : ' + str(len(state_space))
    if vehicle_index is 0:
        state_space = possible_states
    else:
        state_space_copy = state_space
        state_space = []
        for p_state in possible_states:
            for state in state_space_copy:
                state = np.vstack((state, p_state))
                # print 'state: \n ' + str(state)
                state_space.append(state)

# print 'state space: \n ' + str(state_space)

##
# starting locations

# Ego-vehicle starts at [0, 0]
starting_poses = np.array([west, south])
pose_ind = 1
while pose_ind < nr_vehicles:
    new_pose = np.array([0, 0])
    new_pose[0] = np.random.randint(0, high=east + 1, size=1)
    new_pose[1] = np.random.randint(0, high=south + int((nr_vehicles + 1) / 2), size=1)
    print 'new_pose' + str(new_pose)

    if not check_collision(new_pose, starting_poses):
        starting_poses = np.vstack((starting_poses, new_pose))  # axis=0))
        pose_ind += 1
    # print 'Pose index : ' + str(pose_ind)

# starting locations
print 'starting_poses : '
print starting_poses


# observational noise CPT (left, right, stay)
# obsnoiseCPT = np.array([prob_left, prob_right, prob_remain, prob_brake])


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
    elif location[1] > north:
        location[1] = north

    if location[0] == west:
        if location[1] > int(north - blocked_north):
            location[1] = north - blocked_north

    return location


def new_localization(**keyword_args):
    """
    Combines start loc. and moves to give new loc.
    Used by DeterNode F
    :param :param keyword_args: multiple key-value arguments, namely previous location and
                        observational noise of all other vehicles.
    :return:
    """

    car_movements = []
    # new_poses = np.array([0, 0])
    # new_poses = []
    previous_poses = None
    for keyword, value in keyword_args.iteritems():
        # print "%s = %s" % (keyword, value)
        if keyword is 'previous_poses':
            previous_poses = value
            # print "%s = %s" % (keyword, value)
        else:
            # print "%s = %s" % (keyword, value)
            car_movements.append(value)

    if previous_poses is None:
        previous_poses = copy.deepcopy(starting_poses)
        # print "No prev pose"

    for movement_index in range(nr_vehicles):
        # print "Car nr %s" % movement_index
        # print "Prev pos %s - mov %s" % (previous_poses[movement_index], car_movements[movement_index])
        if len(car_movements) is 0:
            car_move = np.array([0, 0])
        else:
            car_move = car_movements[movement_index]
        # car_move_gpu = gpuarray.to_gpu(car_move.astype(np.float32))
        # prev_pos_gpu = gpuarray.to_gpu(previous_poses[movement_index].astype(np.float32))
        # new_p = (car_move_gpu + prev_pos_gpu).get()
        # new_p = adjust_loc(new_p)
        new_p = adjust_loc(car_move + previous_poses[movement_index])
        if movement_index is 0:
            new_poses = np.array([new_p[0], new_p[1]])
        else:
            new_poses = np.vstack((new_poses, new_p))
        # print "New pos %s" % new_poses[movement_index]

    # print 'new_loc'
    # print new_poses
    return new_poses


def adjust_car_obs(**keyword_args):
    """
    Combines observational noise and previous locations of cars to give a valid location on-grid
    for the car's observation of the position of the other cars
    :param keyword_args: multiple key-value arguments, namely previous location and
                        observational noise of all other vehicles.
    :return:
    """

    obs_noise = []
    # new_poses = np.array([0, 0])
    car_number = -1# for index_node in range(nr_vehicles):
#     car_name = 'Dcar' + str(index_node)
#     print 'Training: ' + car_name + 'at level 2'
#     MCRL_solved.train_node(car_name, 2, setCPT=True)
    previous_poses = None
    for keyword, value in keyword_args.iteritems():
        # print "%s = %s" % (keyword, value)
        if keyword is 'previous_poses':
            previous_poses = value
        elif keyword is 'car_nr':
            car_number = value
        else:
            obs_noise.append(value)
    if car_number is -1:
        print 'No car number specified'
        return None

    other_car_pos = copy.deepcopy(previous_poses)
    index_obs = 0
    # print 'Car observations'
    for index_vehicles in range(nr_vehicles):
        # print 'Car ' + str(index_vehicles)
        if index_vehicles is car_number:
            # print 'Own car'
            other_car_pos[index_vehicles] = previous_poses[index_vehicles]
        else:
            opp_pos = adjust_loc(obs_noise[index_obs] + previous_poses[index_vehicles])
            # print "Opponent = %s" % opp_pos
            other_car_pos[index_vehicles] = opp_pos
            index_obs += 1
        # print "other_car_pos : %s" % other_car_pos

    # print other_car_pos
    return other_car_pos


##################################################
# THE NODES - first time 0, then 1 to T in a loop
##################################################

params_f = {}
continuous_f = False

movement_node = DeterNode('Root0', new_localization, params_f, continuous_f, space=state_space, basename='Root', time=0)
# Observational noise, does not have a parent node
# Create an observational noise for each car : ChanceNode CcarX, no parents

node_set = set([movement_node])
# BUILD time steps 0 to T-1 iteratively
for time_step in range(0, max_time):
    # Observational noise, does not have a parent node
    # Create an observational noise for each car : ChanceNode CcarX, no parents
    if training_values is not None:
        obsnoiseCPT = training_values[0]
        par_obs = [movement_node]
    else:
        # obsnoiseCPT = np.array([prob_left, prob_right, prob_remain, prob_brake])
        obsnoiseCPT = np.array([prob_right, prob_remain, prob_brake])
        par_obs = []

    CPTi = (obsnoiseCPT, par_obs, action_space)
    chance_nodes = []
    for index_node in range(nr_vehicles):
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
        param_obs = {'car_nr': index_node, 'previous_poses': movement_node}  # , 'decision_nodes': car_decision_nodes}

        for index_chance_node in range(nr_vehicles):
            # adding the other nodes
            if index_chance_node is not index_node:
                # print 'Chance node ' + str(index_chande_node)
                param_obs['Cnode' + str(index_chance_node)] = chance_nodes[index_chance_node]
        obs_car_nodes.append(DeterNode('O' + node_name, adjust_car_obs, param_obs, continuous_f,
                                       space=state_space, basename='O' + node_basename, time=time_step))

    # Makes decision based on status (paren FSeek or FHide)
    car_decision_nodes = []
    for index_node in range(nr_vehicles):
        node_basename = 'car' + str(index_node)
        node_name = node_basename + str(time_step)
        car_decision_nodes.append(DecisionNode('D' + node_name, node_basename, action_space,
                                               parents=[obs_car_nodes[index_node]],
                                               basename='D' + node_basename, time=time_step))

    # Moves according to decision (Parent, previous position and decision on this timestep)
    params_loc = {'previous_poses': movement_node}  # , 'decision_nodes': car_decision_nodes}

    for index_node in range(nr_vehicles):
        # adding the other nodes
        # print 'Dec node ' + str(index_node)
        params_loc['Dnode' + str(index_node)] = car_decision_nodes[index_node]

    # print params_loc
    movement_node = DeterNode('F%s' % time_step, new_localization, params_loc,
                              continuous_f, space=state_space, basename='F', time=time_step)

    for index_node in range(nr_vehicles):
        # adding the other nodes
        # print 'Add node: ' + str(chance_nodes[index_node])
        # print 'Add node: ' + str(obs_car_nodes[index_node])
        # print 'Add node: ' + str(car_decision_nodes[index_node])
        node_set.update([chance_nodes[index_node], obs_car_nodes[index_node], car_decision_nodes[index_node]])
    # adding new set of nodes from time step t to nodeset
    node_set.update([movement_node])


##########################
# REWARD FUNCTIONS
##########################


def car_reward(F, player):
    """
    seeker's reward function
    :param F: Node with status of all players
    :param player: player number
    :return:
    """
    # print 'Player'
    # print player
    player_nr = int(player[3:])
    # print 'Player Number'
    # print player_nr
    # reward = 0
    # F = None

    position_player = F[player_nr]
    reward = position_player[1] * position_player[1]
    reward *= 4
    # if position_player[0] == west:
    #     reward = -position_player[1]
    #     if position_player[1] > int(north - blocked_north):
    #         reward -= 150

    f_not_player = None
    for f_b in range(len(F)):
        #  print f_b
        if not f_b == player_nr:
            if f_not_player is None:
                f_not_player = F[f_b]
            else:
                f_not_player = np.vstack((f_not_player, F[f_b]))
    # print position_player
    # print np.array([east, north])
    if np.array_equal(position_player, np.array([east, north])) is False:
        if check_collision(position_player, f_not_player):
            # print 'Collision'
            reward = -250

    #  sys.stdout.write('\n')
    #  sys.stdout.write(' p: ' + str(player_nr) + ' pos : ' + str(position_player[0]) +
    #                   ' - ' + str(position_player[1]) + ' r: ' + str(reward))
    #  sys.stdout.write('\n')
    return reward


# rewards dictionary
reward_funcs = {}
for index_node in range(nr_vehicles):
    # adding the other nodes

    car_name = 'car' + str(index_node)
    # print 'Car name: ' + car_name
    reward_funcs[car_name] = car_reward

##################################
# CREATING THE iterSemiNFG
##################################
G = iterSemiNFG(node_set, reward_funcs)

# making a set of the names of the first two time steps for visualization
# drawset = set([n.name for n in G.time_partition[0]]).union(set([n.name for n in G.time_partition[1]]))
drawset = set([n.name for n in G.time_partition[0]])

G.draw_graph(drawset)

# visualizing the first two time steps of the net

###########################################
# MANIPULATING CPTs
###########################################
# Giving nodes a uniform CPT
for index_node in range(nr_vehicles):
    # adding the other nodes
    decision_node_name = 'Dcar' + str(index_node)
    print 'Decision node name: ' + decision_node_name
    G.bn_part[decision_node_name][0].randomCPT(mixed=True)  # .uniformCPT()
    # G.bn_part[decision_node_name][0].uniformCPT()

# pointing all CPTs to time 0 CPT
cptdict = G.get_decisionCPTs(mode='basename')
G.set_CPTs(cptdict)

###########################################
# SAMPLING
###########################################
# Sample the entire Bayesian Network
# G.sample()
# sample entire net and return a dict of sampled values for node Dhide8 and F1
# valuedict = G.sample(nodenames=['Dhide3', 'F1'])
# print 'sample entire net and return a dict of sampled values for node Dhide8 and F1 \n '

# print (valuedict)
# Sample timesteps 3 through 6 and returning a dict with values for specific basenames
# valuedict = G.sample_timesteps(0, 3, basenames=['Dhide', 'F', 'Cseek'])

# print (valuedict)
# sample F4 and all of its descendants
# valuedict = G.sample(start=['F0'])
# print 'sample entire '
# print (valuedict)
# We can also train a player to the next level

###########################################
# GETTING VALUES
###########################################
# value_dict = G.get_values(nodenames=['Cseek0', 'Dhide8'])

#####################################################
# TRAINING LEVEL 1 with ewma_mcrl
#####################################################

# Generate the dictionary of inputs
N = 5
mcrl_params = mcrl_dict(G, 2, np.linspace(300, 1, N), N, 1, np.linspace(.5, 1, N),
                        np.linspace(.2, 1, N), L0Dist='uniform', pureout=True)

MCRL_solved = EwmaMcrl(G, mcrl_params)
MCRL_solved.solve_game(setCPT=True)

# for index_node in range(nr_vehicles):
#     car_name = 'Dcar' + str(index_node)
#     print 'Training: ' + car_name + 'at level 2'
#     MCRL_solved.train_node(car_name, 2, setCPT=True)
#
# for index_node in range(nr_vehicles):
#     car_name = 'Dcar' + str(index_node)
#     print 'Training: ' + car_name + 'at level 2'
#     MCRL_solved.train_node(car_name, 3, setCPT=True)


store_training_values(nr_vehicles, MCRL_solved.Game)

print 'try non solved'
valuedict = G.sample_timesteps(G.starttime, G.endtime, basenames=['F'])
print valuedict['F']

print 'try solved'
valuedict = MCRL_solved.Game.sample_timesteps(G.starttime, G.endtime, basenames=['F'])
print valuedict['F']

print 'try solved'
valuedict = MCRL_solved.Game.sample_timesteps(G.starttime, G.endtime, basenames=['F'])
print valuedict['F']

print 'try solved'
valuedict = MCRL_solved.Game.sample_timesteps(G.starttime, G.endtime, basenames=['F'])
print valuedict['F']

print 'try solved'
valuedict = MCRL_solved.Game.sample_timesteps(G.starttime, G.endtime, basenames=['F'])
print valuedict['F']

print 'try solved'
valuedict = MCRL_solved.Game.sample_timesteps(G.starttime, G.endtime, basenames=['F'])
print valuedict['F']

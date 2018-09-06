#!/usr/bin/env python

# -*- coding: utf-8 -*-
"""
Implements a simple hide-and-seek iterSemiNFG:
There is a hider and a seeker. In each time step, each player is at a location
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
import numpy as np

###########################################
# PARAMETERS AND FUNCTIONS
###########################################
# boundaries of the grid
west = 0
east = 1
north = 15
south = 0

# actions of the players

left = np.array([-1, 1])
right = np.array([1, 1])
remain = np.array([0, 1])


remain_f = np.array([0, 2])
slow = np.array([0, 1])

# space of actions that players can choose
actionspace_s = [left, right, remain]
actionspace_f = [left, right, slow, remain_f]
# time steps
T = 7

# starting locations
startingloc = np.array([[west, south], [west, south+2]])


# observational noise CPT (left, right, stay)
obsnoiseCPT_f = np.array([.2, .2, .1, .5])
obsnoiseCPT_s = np.array([.2, .2, .6])

# the space for the state nodes below, Froot and F
statespace = [np.array([[w, x], [y, z]])
              for w in range(east+1) for x in range(north+1)
              for y in range(east+1) for z in range(north+1)]


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

    return location


def new_localization(fast_car_move=np.array([0, 0]), slow_car_move=np.array([0, 0]), previous_loc=startingloc):
    """
    Combines start loc. and moves to give new loc.
    Used by DeterNode F
    :param fast_car_move:
    :param slow_car_move:
    :param previous_loc:
    :return:
    """
    loc_fast_car = adjust_loc(fast_car_move+previous_loc[0])
    loc_slow_car = adjust_loc(slow_car_move+previous_loc[1])
    return np.vstack((loc_fast_car, loc_slow_car))


def adjust_fast_car_obs(noise, loc):
    """
    Combines observational noise and location of the slow car to give a valid location on-grid
    for the fast car observation of the position of the other car
    :param noise:
    :param loc:
    :return:
    """
    opponent = adjust_loc(noise+loc[1])
    return np.vstack((loc[0], opponent))


def adjust_slow_car_obs(noise, loc):
    """
    Combines observational noise and location of the fast car to give a valid location on-grid
    for the slow car observation of the position of the other car
    :param noise:
    :param loc:
    :return:
    """
    opponent = adjust_loc(noise+loc[0])
    return np.vstack((opponent, loc[1]))

##################################################
# THE NODES - first time 0, then 1 to T in a loop
##################################################
# STATE ROOT NODE, DeterNode F


params_f = {}
continuous_f = False
root_node = DeterNode('Froot0', new_localization, params_f, continuous_f, space=statespace, basename='Froot', time=0)

# OBSERVATIONAL NOISE for fast car : ChanceNode Cfast, no parents
par_obs_f = []
CPTi_f = (obsnoiseCPT_f, par_obs_f, actionspace_f)
C_fast = ChanceNode('Cfast0', CPTip=CPTi_f, basename='Cfast', time=0)

# OBSERVATIONAL NOISE for slow car : ChanceNode Cslow, no parents
par_obs_s = []
CPTi_s = (obsnoiseCPT_s, par_obs_s, actionspace_s)
C_slow = ChanceNode('Cslow0', CPTip=CPTi_s, basename='Cslow', time=0)

# COMBINE OBS NOISE FOR fast car, DeterNode Ffast.
# Parents chance node of slow car (C_slow) and previous localization (root_node)
params_Obs_fast = {'noise': C_slow, 'loc': root_node}
Obs_fast = DeterNode('Ffast0', adjust_fast_car_obs, params_Obs_fast, continuous_f,
                     space=statespace, basename='Ffast0', time=0)

# COMBINE OBS NOISE FOR slow car, DeterNode Fslow.
# Parents chance node of fast car (C_fast) and previous localization (root_node)
params_Obs_slow = {'noise': C_fast, 'loc': root_node}
Obs_slow = DeterNode('Fslow0', adjust_slow_car_obs, params_Obs_slow, continuous_f,
                     space=statespace, basename='Fhide', time=0)

# Decision Node of fast car: DecisionNode D_fast.
# Parents are observation of fast car Obs_fast
D_fast = DecisionNode('Dfast0', 'fast_car', actionspace_f, parents=[Obs_fast], basename='Dfast', time=0)
# Decision Node of slow car: DecisionNode D_slow.
# Parents are observation of slow car Obs_slow

D_slow = DecisionNode('Dslow0', 'slow_car', actionspace_s, parents=[Obs_slow], basename='Dslow', time=0)


# STATE ROOT NODE, DeterNode F
params_loc = {'fast_car_move': D_fast, 'slow_car_move': D_slow, 'previous_loc': root_node}
F = DeterNode('F0', new_localization, params_loc, continuous_f, space=statespace, basename='F', time=0)

# adding time 0 nodes to nodeset
# nodeset = set([F, Froot, Fseek, Fhide, Cseek, Chide, Dseek, Dhide])
nodeset = set([F, root_node, C_fast, C_slow, Obs_fast, Obs_slow, D_fast, D_slow])

# BUILD time steps 1 to T-1 iteratively
for t in range(1, T):
    # Observational noise, does not have a parent node
    C_fast = ChanceNode('Cfast%s' % t, CPTip=CPTi_f, basename='Cfast', time=t)
    C_slow = ChanceNode('Cslow%s' % t, CPTip=CPTi_s, basename='Cslow', time=t)

    # Mixes movement (node F) with observational noise (node Cseek)
    params_Obs_fast = {'noise': C_fast, 'loc': F}
    Obs_fast = DeterNode('Ffast%s' % t, adjust_fast_car_obs, params_Obs_fast, continuous_f,
                         space=statespace, basename='Ffast', time=t)

    params_Obs_slow = {'noise': C_slow, 'loc': F}
    Obs_slow = DeterNode('Fslow%s' % t, adjust_slow_car_obs, params_Obs_slow, continuous_f,
                         space=statespace, basename='Fslow ', time=t)

    # Makes decision based on status (paren FSeek or FHide)
    D_fast = DecisionNode('Dfast%s' % t, 'fast_car', actionspace_f,
                          parents=[Obs_fast], basename='Dfast', time=t)

    D_slow = DecisionNode('Dslow%s' % t, 'slow_car', actionspace_s,
                          parents=[Obs_slow], basename='Dslow', time=t)

    # Moves according to decision (Parent, previous position and decision on this timestep)
    paramsf = {'fast_car_move': D_fast, 'slow_car_move': D_slow, 'previous_loc': F}

    F = DeterNode('F%s' % t, new_localization, paramsf, continuous_f, space=statespace,
                  basename='F', time=t)
    # adding new set of nodes from time step t to nodeset
    nodeset.update([C_fast, C_slow, Obs_fast, Obs_slow, D_fast, D_slow, F])


##########################
# REWARD FUNCTIONS
##########################


def fast_car_reward(F):
    """
    seeker's reward function
    :param F:
    :return:
    """

    reward = 0
    if np.array_equal(F[0], F[1]):
        reward -= 1000
    if F[0][0] == 1:
        reward -= 1
    reward += F[0][1]
    sys.stdout.write('\r')
    sys.stdout.write(' s: ' + str(F[0]) + ' h: ' + str(F[1]) + ' r: ' + str(reward))
    return reward


def slow_car_reward(F):
    """
    hider's reward function
    :param F:
    :return:
    """
    reward = 0
    if np.array_equal(F[0], F[1]):
        reward -= 1000
    if F[1][0] == 1:
        reward -= 1
    reward += F[0][1]
    sys.stdout.write('\r')
    sys.stdout.write(' s: ' + str(F[0]) + ' h: ' + str(F[1]) + ' r: ' + str(reward))
    return reward


# rewards dictionary


reward_funcs = {'fast_car': fast_car_reward, 'slow_car': slow_car_reward}

##################################
# CREATING THE iterSemiNFG
##################################
G = iterSemiNFG(nodeset, reward_funcs)

# making a set of the names of the first two time steps for visualization
# drawset = set([n.name for n in G.time_partition[0]]).union(set([n.name for n in G.time_partition[1]]))
drawset = set([n.name for n in G.time_partition[0]])

G.draw_graph(drawset)

# visualizing the first two time steps of the net

###########################################
# MANIPULATING CPTs
###########################################
# Giving hider a uniform CPT
G.bn_part['Dfast'][0].uniformCPT()
# Giving seeker a pure random CPT
G.bn_part['Dslow'][0].uniformCPT()
# G.bn_part['Dseek'][0].randomCPT(mixed=False)
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
#valuedict = G.get_values(nodenames=['Cseek0', 'Dhide8'])

#####################################################
# TRAINING LEVEL 1 with ewma_mcrl
#####################################################

# Generate the dictionary of inputs
N = 40
mcrl_params = mcrl_dict(G, 1, np.linspace(50, 1, N), N, 1, np.linspace(.5, 1, N),
                        np.linspace(.2, 1, N),  L0Dist='uniform', pureout=True)

MCRL_solved = EwmaMcrl(G, mcrl_params)
MCRL_solved.solve_game(setCPT=True)


# Show convergence for hider

# MCRL_solved.figs['Dhide']['1'].show()

# We can also train a player to the next level

MCRL_solved.train_node('Dfast', 2, setCPT=True)
MCRL_solved.train_node('Dslow', 2, setCPT=True)
MCRL_solved.train_node('Dfast', 3, setCPT=True)

valuedict = G.sample_timesteps(G.starttime, G.endtime, basenames=['F'])
print valuedict['F']

print 'try'
valuedict = G.sample_timesteps(G.starttime, G.endtime, basenames=['F'])
print valuedict['F']


print 'try'
valuedict = G.sample_timesteps(G.starttime, G.endtime, basenames=['F'])
print valuedict['F']


plt.show()

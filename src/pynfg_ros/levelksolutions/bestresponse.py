# -*- coding: utf-8 -*-
"""
Implements Best Response Level-K calculations for SemiNFG and iterSemiNFG

Part of: PyNFG - a Python package for modeling and solving Network Form Games

Created on Fri May 24 07:01:05 2013

Copyright (C) 2013 James Bono

GNU Affero General Public License

"""
from __future__ import division

import copy
import numpy as np
from pynfg_ros.utilities.utilities import convert_2_pureCPT, mceu, input_dict, iterated_input_dict
import warnings



class BestResponse(object):
    """ Finds the **uncoordinated** best response solution
    for a semi-NFGame.

    :arg Game:  A semi-NFG
    :type Game: semiNFG
    :arg specs: dictionary of dictionaries containing
        the level, level-0 strategy, tolerance
        and degrees of rationality of each player.
        See below for details.
    :type specs: dict
    :type N: bool

    specs is a triply-nested dictionary.  The first set of keys
    are the player names.  For each player key, there are keys:

    Level : int
        The player's Level
    delta : float
        The discount factor

    For each player, there is a key for each node that the player controls.
    For each node entry, there is a  dictionary with  three entries and
     one optional entry:


    L0Dist : ndarray, str, None
        If ndarray, then the level 0 CPT is set to
        L0Dist. If L0Dist is 'uniform', then all Level 0 CPTs are set to
        the uniform distribution.  If L0Dist is None, then the level 0 CPT
        is set to the CPT of the inputted game.
    tol : int
        the minimum number of samples per parent value
    N : int
        The max number of iterations for the estimation.

    beta : float
        (Optional)  Logit best response parameter

    """

    def __init__(self, Game, specs):
        self.Game = copy.deepcopy(Game)
        self.specs = specs
        self.high_level = self._set_new_attributes()
        self._set_L0_CPT()


    def _set_new_attributes(self):
        Game = self.Game
        ps = self.specs
        levels = []
        for player in ps:
            node_set = list(Game.partition[player])
            for node in node_set:
                nodename = node.name
                node.Level, node.delta, node.tol, node.N, node.beta =  \
                    ps[player]['Level'], ps[player]['delta'],\
                    ps[player][nodename]['tol'], ps[player][nodename]['N'], \
                    ps[player][nodename]['beta']
                try:
                    node.LevelCPT
                except AttributeError:
                    node.LevelCPT = {}
            levels.append(ps[player]['Level'])
        return max(levels)

    def _set_L0_CPT(self):
        """ Sets the level 0 CPT"""
        Game = self.Game
        ps = self.specs
        for player in ps:
            node_set = list(Game.partition[player])
            for node in node_set:
                try:
                    node.LevelCPT[0]
                except KeyError:
                    nodename = node.name
                    if ps[player][nodename]['L0Dist'] == 'uniform':
                        node.LevelCPT[0] = \
                            node.uniformCPT(setCPT=False)
                    elif ps[player][nodename]['L0Dist'] is None:
                        warnings.warn("No entry for L0Dist for player %s,\
                        setting to current CPT" % player)
                        node.LevelCPT[0] = Game.node_dict[nodename].CPT
                    elif type(ps[player][nodename]['L0Dist']) == np.ndarray:
                        node.LevelCPT[0] = \
                            ps[player][nodename]['L0Dist']

    def train_node(self, nodename, level, logit=False, setCPT=False, verbose=False):
        """Compute level-k best response at the DN given Game

        :arg nodename: the name of the decision node where MCEUs are estimated
        :type nodename: str
        :arg level: The level at which to train that player
        :type level: int
        :arg setCPT: If the trained CPT should be set as the current CPT.
            Otherwise, it can be accessed through node.LevelCPT.  Default is
            False
        :type setCPT: bool

        Notes
        -----

        If training a player at level k, the other players' CPT will be accessed
        through self.Game.node_dict[other_player].LevelCPT[k-1]

        """
        print 'Training ' + nodename + ' at level ' + str(level)
        Game = copy.deepcopy(self.Game)  # copy in order to maintain original CPT
        ps = self.specs
        for node in Game.node_dict.values():  # Game changes, self.Game doesn't
            if type(node) is pynfg.DecisionNode:
                try:
                    node.CPT = node.LevelCPT[level - 1]
                except KeyError:
                    raise KeyError('Need to train other players at level %s'
                                   % str(level-1))
        EUtable = mceu(Game, nodename, Game.node_dict[nodename].N,
                       Game.node_dict[nodename].tol, Game.node_dict[nodename].delta,
                       verbose=verbose)
        if not logit:
            self.Game.node_dict[nodename].LevelCPT[level] = \
                  convert_2_pureCPT(EUtable)
            if setCPT:
                self.Game.node_dict[nodename].CPT = convert_2_pureCPT(EUtable)
        else:
            weight = np.exp(Game.node_dict[nodename].beta*EUtable)
            norm = np.sum(weight, axis=-1)
            self.Game.node_dict[nodename].LevelCPT[level] = \
            weight/norm[..., np.newaxis]
            if setCPT:
                self.Game.node_dict[nodename].CPT = weight/norm[..., np.newaxis]


    def solve_game(self, setCPT=False, verbose=False):
        """ Solves the game for specified player levels"""
        Game = self.Game
        for level in np.arange(1, self.high_level):
            for player in Game.players:
                for controlled in Game.partition[player]:
                    self.train_node(controlled.name, level, verbose=verbose)
        for player in Game.players:
            for controlled in Game.partition[player]:
                if controlled.Level == self.high_level:
                    self.train_node(controlled.name, self.high_level,
                                    verbose=verbose)
        if setCPT:
            for player in Game.players:
                for node in Game.partition[player]:
                    Game.node_dict[node.name].CPT = Game.node_dict[node.name].\
                        LevelCPT[Game.node_dict[node.name].Level]


def br_dict(Game, N, Level, L0Dist=None, delta=1, tol=30, beta=None):
    """A helper function to generate the player_spec dictionary
    for best response.  If optional arguments are specified, they are
    set for all decision nodes and all players.  The idea is to use br_dict
    to great a shell of parameters and then modify them accordingly.

    :arg Game: A SemiNFG
    :type Game: SemiNFG

    .. seealso::
        See the BestResponse documentation (above) for details of the  optional arguments
    """
    if beta is None:
        return input_dict(Game, [('Level', Level), ('delta', delta)],
                          [('L0Dist', L0Dist), ('N', N), ('tol', tol)])
    else:
        return input_dict(Game, [('Level', Level), ('delta', delta)],
                      [('L0Dist', L0Dist), ('N', N), ('tol', tol),
                       ('beta', beta)])



from pynfg_ros.utilities.extra_functions import check_col
from pynfg_ros.utilities.observation import Observation
import numpy as np


class WorldModel:
    def __init__(self):
        # Limits of the road
        self.south = 0
        self.west = 0
        self.goal_lane = 1
        self.n_lanes = 2
        self.east = self.west + self.n_lanes - 1

        # distance to Blocked (end of lane)
        self.dis_blocked = 25

        # time steps
        self.max_time = 10

        # Maximum velocity
        self.max_vel = 2

        # Vehicle length
        self.veh_len = 5

        # Min Gap
        self.min_gap = 2.0
        self.max_accepted_gap = 2 * self.veh_len

        # Number of players
        self.nr_vehicles = 4  # ego-vehicle + 3 other cars
        self.current_status = None

    def get_starting_status(self):
        # Starting positions
        # Ego-vehicle starts at west south
        # print 'new_pose'
        pose_ind = 0
        reset_count = 0
        accepted_gap = np.random.randint(self.max_accepted_gap, size=1) + 1.0
        # print 'accepted_gap :  ' + str(accepted_gap)
        while pose_ind < self.nr_vehicles:
            new_pose = np.array([0, 0])
            # print 'new_pose' + str(new_pose)
            if pose_ind is 0:
                # starting_poses = new_pose
                new_pose[0] = np.random.randint(self.n_lanes, size=1)
                new_pose[1] = np.random.randint(self.south + self.veh_len, size=1)
                pose_ind += 1
                starting_poses = np.array([new_pose[0], 15 + new_pose[1]])
                if new_pose[0] == 1:
                    self.goal_lane = 0
                elif new_pose[0] == 0:
                    self.goal_lane = 1
            else:
                new_pose[0] = self.goal_lane
                new_pose[1] = np.random.randint(self.south + 3.5 * (self.veh_len + accepted_gap), size=1)
                if not self.check_collision(new_pose, starting_poses, accepted_gap):
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

        while pose_ind < self.nr_vehicles:
            new_vel = np.array([0, 1.0])
            # print 'new_velocity' + str(new_vel)
            starting_velocities = np.vstack((starting_velocities, new_vel))  # axis=0))
            pose_ind += 1
            # print 'Pose index : ' + str(pose_ind)

        # print 'Starting velocities : \n ' + str(starting_velocities)

        self.current_status = [starting_poses, starting_velocities]
        return self.current_status

    def check_collision(self, current_pose, pose_array, allowed_gap=0.0):
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
            if check_col(current_pose, pose_array, self.veh_len + allowed_gap) is True:
                # print ' Collission in pose' + str(current_pose)
                return True
        else:
            [rows, cols] = pose_array.shape
            if cols is not 2:
                print 'pose array should have 2 cols - it has ' + str(cols)
                return True

            for index_compare in range(0, rows):
                if check_col(current_pose, pose_array[index_compare], self.veh_len + allowed_gap) is True:
                    return True
        return False

    def get_observation(self, vehicle_nr, given_status=None):

        if given_status is None:
            current_poses = self.current_status[0]
            current_velocities = self.current_status[1]
        else:
            current_poses = given_status[0]
            current_velocities = given_status[1]

        # my_vel = current_velocities[vehicle_nr][1]
        my_pos = current_poses[vehicle_nr]

        # print '\n NEW OBSERVATION \n'
        # print 'current_velocities : \n ' + str(current_velocities)
        # print 'My vel : \n ' + str(my_vel)
        observation = Observation()

        # fl = -1
        # f = -1
        # fr = -1
        # cl = -1
        # cr = -1
        # rl = -1
        # rr = -1
        # bl = -1
        # br = -1
        # dgoal = 0

        current_lane = my_pos[1]
        diff_lane = self.goal_lane - current_lane

        # Compute lane observation
        if diff_lane is 0:
            observation.lane = 0
        elif diff_lane > 0:
            observation.lane = 1
        else:
            observation.lane = -1

        # Compute dgoal observation

        current_pos_x = my_pos[0]
        diff_goal = self.dis_blocked - current_pos_x

        # Compute lane observation
        if diff_goal <= 0:
            observation.dgoal = 0
        elif 0 < diff_goal <= 2:
            observation.dgoal = 1
        elif 2 < diff_goal <= 4:
            observation.dgoal = 2
        elif 4 < diff_goal <= 6:
            observation.dgoal = 3
        elif 6 < diff_goal <= 8:
            observation.dgoal = 4
        elif 8 < diff_goal <= 10:
            observation.dgoal = 4

        for index_node in range(self.nr_vehicles):
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
                    if current_velocities[index_node][1] > observation.fr:
                        observation.fr = 1
                if 0.0 <= y < 5.0:
                    if current_velocities[index_node][1] > observation.cr:
                        observation.cr = 1
                if -4.0 <= y < 1.0:
                    if current_velocities[index_node][1] > observation.rr:
                        observation.rr = 1
                if -8.0 <= y < -3.0:
                    if current_velocities[index_node][1] > observation.br:
                        observation.br = 1

            if x == 0:
                if 4.0 <= y <= 9.0:
                    if current_velocities[index_node][1] > observation.f:
                        observation.f = 1

            if x == 1:
                if 4.0 <= y <= 9.0:
                    if current_velocities[index_node][1] > observation.fl:
                        observation.fl = 1
                if 0.0 <= y < 5.0:
                    if current_velocities[index_node][1] > observation.cl:
                        observation.cl = 1
                if -4.0 <= y < 1.0:
                    if current_velocities[index_node][1] > observation.rl:
                        observation.rl = 1
                if -8.0 <= y < -3.0:
                    if current_velocities[index_node][1] > observation.bl:
                        observation.bl = 1

        # print 'observation space : \n ' + str(observation_space)

        # print 'my pos : ' + str(my_pos)

        # if my_pos[1] == 0:
        #    observation_space[1, :] = 100
        # if my_pos[1] == north:
        #     fl = 100
        #     fr = 100
        #     f = 100

        # observations = np.array([[fl, f, fr, cl, cr, rl, rr, bl, br, dgoal, lane]])
        # print 'Vehicle nr : ' + str(vehicle_nr) + ' pos : ' + str(my_pos)
        # print 'my vel : ' + str(my_vel)
        # print 'Current poses: \n' + str(current_poses)
        # print 'Current vel: \n' + str(current_velocities)
        # print ' fl: ' + str(fl) + ' f: ' + str(f) + ' fr: ' + str(fr) +\
        #       ' rl: ' + str(rl) + ' rr: ' + str(rr) + ' vel: ' + str(vel) + '\n'
        observation.reload_observation_value()
        return observation.observation

    def adjust_loc(self, location):
        """"
         function that adjusts for moves off the grid
        """
        if location[0] < self.west:
            location[0] = self.west

        elif location[0] > self.east:
            location[0] = self.east

        if location[1] < self.south:
            location[1] = self.south
        # elif location[1] > north:
        #     location[1] = north

        if location[0] != self.goal_lane:
            if location[1] > self.dis_blocked + 1.5:
                location[1] = self.dis_blocked + 1.5

        return location


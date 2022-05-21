import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np

from planning_utils import a_star, heuristic, create_grid, prune_path_bresenham, plane_heading
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local

# configurable parameters
# (global variables aren't great but they make using the simulator easier)
#
# REVIEWER: Change the target location here
TARGET_LAT = 37.792835 # degrees
TARGET_LON = -122.397497 #degrees
#
# Other configurable parameters
TARGET_ALTITUDE = 20 # meters
SAFETY_DISTANCE = 10 # meters
TAKEOFF_SCALE = 0.95 # unitless
MIN_DEADBAND_SPEED = 1.0 # meters/second
MIN_DEADBAND = 0.5 # meters
DEADBAND_SCALE = 0.15 # unitless
LANDING_WAYPOINT_SCALE = 0.5 # unitless
READY_TO_LAND_SPEED = 1.0 # meters/second
LANDING_COMPLETE_VERTICAL_SPEED = 1.0 # meters/second
LANDING_COMPLETE_POSITION_TOLERANCE = 0.05 # meters

class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    PLANNING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()


class MotionPlanning(Drone):

    def __init__(self, connection):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0, 0.0])
        self.goal_altitude = 0.0
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # register callbacks
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > TAKEOFF_SCALE * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            linear_speed = np.linalg.norm(self.local_velocity[0:2])
            deadband = 0.0
            if linear_speed < MIN_DEADBAND_SPEED:
                deadband = MIN_DEADBAND
            else:
                deadband = DEADBAND_SCALE * linear_speed
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < deadband:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if linear_speed < READY_TO_LAND_SPEED:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if abs(self.local_velocity[2]) < LANDING_COMPLETE_VERTICAL_SPEED and \
                    abs((-1.0 * self.local_position[2]) - self.goal_altitude) < LANDING_COMPLETE_POSITION_TOLERANCE:
                self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                if len(self.waypoints) > 0:
                    self.takeoff_transition()
                else:
                    self.disarming_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")

        self.target_position[2] = TARGET_ALTITUDE

        # TODO: read lat0, lon0 from colliders into floating point values
        with open('colliders.csv') as f:
            line = f.readline()
        lat0_token = 'lat0 '
        lon0_token = 'lon0 '
        lat0 = float(line[line.find(lat0_token) + len(lat0_token): line.find(',')])
        lon0 = float(line[line.find(lon0_token) + len(lon0_token):])

        # TODO: set home position to (lon0, lat0, 0)
        self.set_home_position(lon0, lat0, 0.0)
        global_home = [lon0, lat0, 0.0]

        # TODO: retrieve current global position
        cur_global_pos = self.global_position

        # TODO: convert to current local position using global_to_local()
        cur_local_pos = global_to_local(cur_global_pos, global_home)

        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
        
        # Define a grid for a particular altitude and safety margin around obstacles
        grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))

        # Define starting point on the grid
        # TODO: convert start position to current position rather than map center
        grid_start = (int(cur_local_pos[0])-north_offset, int(cur_local_pos[1])-east_offset)
        
        # Set goal as some arbitrary position on the grid
        # TODO: adapt to set goal as latitude / longitude position and convert
        grid_goal_global_full = [TARGET_LON, TARGET_LAT, TARGET_ALTITUDE]
        grid_goal_local_full = global_to_local(grid_goal_global_full, global_home)
        grid_goal = (int(grid_goal_local_full[0])-north_offset, int(grid_goal_local_full[1])-east_offset)
        self.goal_altitude = grid[grid_goal[0], grid_goal[1]]
        print(f'Goal altitude: {self.goal_altitude}')

        # Run A* to find a path from start to goal
        # TODO: add diagonal motions with a cost of sqrt(2) to your A* implementation
        print('Local Start and Goal: ', grid_start, grid_goal)
        path, _ = a_star(grid, heuristic, grid_start, grid_goal)
        # TODO: prune path to minimize number of waypoints
        pruned_path = prune_path_bresenham(path, grid)

        # Convert path to waypoints
        waypoints = list()
        prev_north = cur_local_pos[0]
        prev_east = cur_local_pos[1]
        cur_north = cur_local_pos[0]
        cur_east = cur_local_pos[1]
        for p in pruned_path:
            cur_north = p[0] + north_offset
            cur_east = p[1] + east_offset
            # orient the heading in the direction of the next waypoint
            cur_heading = plane_heading(prev_north, prev_east, cur_north, cur_east)
            waypoints.append([cur_north, cur_east, TARGET_ALTITUDE, cur_heading])
            prev_north = cur_north
            prev_east = cur_east

        # add a waypoint at the same horizontal location as the final waypoint at a fraction of the height to the
        # landing altitude at this horizontal location; intent here is to smooth the landing
        if len(pruned_path) > 0:
            waypoints.append([cur_north, \
                              cur_east, \
                              int(self.goal_altitude + LANDING_WAYPOINT_SCALE * (TARGET_ALTITUDE - self.goal_altitude)), \
                              0.0])

        # Set self.waypoints
        self.waypoints = waypoints
        # TODO: send waypoints to sim (this is just for visualization of waypoints)
        if len(waypoints) > 0:
            self.send_waypoints()

    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    drone = MotionPlanning(conn)
    time.sleep(1)

    drone.start()

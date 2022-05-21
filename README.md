# Project: 3D Motion Planning
![Quad Image](./misc/enroute.png)

---

For ease of testing, all configurable parameters (e.g. target latitude and longitude, target altitude, safety distance) are specified at the top of motion_planning.py.

---
## Explain the Starter Code

### planning_utils.py
* create_grid: Creates a grid based on obstacle data (e.g. from colliders.csv) and a specified drone height and safety distance.
Obstacles are determined using a 2.5D approach which includes evaluating if the drone height (plus a safety distance) is above a feature; if not, the drone is not above the feature and the feature is considered an obstacle.
* Action: Defines an enum that specifies the possible actions/motions for the A* path search algorithm.
* valid_actions: Determines valid actions (list of Action enums) for a specified node; valid actions are those that remain on the grid and do not collide with an obstacle when the action is performed at the specified node.
* a_star: Performs A* path search algorithm through the specified grid from the specified start to the specified goal, avoiding obstacles (i.e. only performing valid actions).
If a path through the grid is found, this is returned along with the path cost.
* heuristic: Defines a heuristic for the A* path search algorithm.  Implemented as the Euclidean distance from a specified position to a specified goal position.

### motion_planning.py
* Overall, this is very similar to backyard_flyer, especially in terms of the registered callbacks (position , velocity, and state) and state transitions.  The key differences between this and backyard_flyer are in plan_path(), as described below.
* plan_path: Instead of using fixed waypoints like in in backyard_flyer, this function generates waypoints by using the A* path search algorithm to search through a grid of obstacles (based on the data in colliders.csv) to find a path from a starting location to a goal location.
This function also defines a flight state that is not in backyard_flyer, PLANNING, for path planning.  This state is entered after arming.

---
## Implementing Your Path Planning Algorithm

* Read the first line of the csv file, extract lat0 and lon0 as floating point values and use the self.set_home_position() method to set global home
  * See motion_planning.py, plan_path(): lines 148-158
* Determine local position relative to global home
  * See motion_planning.py, plan_path(): lines 160-164
* Change start point to be the current local position
  * See motion_planning.py, plan_path(): lines 173-175
* Be able to choose any (lat, lon) within the map and have it rendered to a goal location on the grid
  * See motion_planning.py, plan_path(): lines 177-183
  * As noted above, to specify the target latitude and longitude, change the configurable parameters at the top of motion_planning.py (TARGET_LAT and TARGET_LON)
* Update the A* implementation to include diagonal motions on the grid that have a cost of sqrt(2)
  * See planning_utils.py
    * Action(): lines 71-74
      * Added the following actions, all with a cost of sqrt(2): NORTH_EAST, SOUTH_EAST, SOUTH_WEST, NORTH_WEST 
    * valid_actions(): lines 104-111
  * See motion_planning.py, plan_path(): lines 185-188
* Cull waypoints from the path determined using search
  * See planning_utils.py
    * bresenham(): lines 189-249
      * This is a generalized version of bresenham, i.e. works for all x1, y1, x2, y2, rather than just the case where x2 > x1 and y2 > y1 
    * prune_path_bresenham(): lines 267-286
    * Note that collinaer pruning is also implemented but this was only used for testing; bresenham does a better job of keeping the drone away from obstacles.
  * See motion_planning.py, plan_path(): lines 189-190

---
## Executing The Flight
* Convert the pruned path to waypoints and send the waypoints to the autopilot
  * See motion_planning.py   
    * plan_path(): lines 192-219
    * send_waypoints(): lines 137-140

---
## Project Additions
* Set heading in the direction of travel
  * See planning_utils.py, plane_heading(): lines 289-291
  * See motion_planning.py, plan_path(): lines 202-203
* Obstacle height is encoded in the 2.5D grid
  * This allows landing on a building that is below the TARGET_ALTITUDE
  * See planning_utils.py, create_grid(): lines 45-52
  * See motion_planning.py
    * plan_path(): lines 182-183
    * velocity_callback(): lines 82-83
* Deadbands around waypoints
  * Allows for smoother waypoint transition
  * Deadbands are speed dependent with a minimum deadband below a minimum speed, otherwise deadbands are proportional to the current speed
  * See motion_planning.py, local_position_callback(): lines 67-73
* Extra waypoint at a fraction of the distance from the current height to the landing height
  * Allows for a smoother landing
  * See motion_planning.py, plan_path(): lines 207-213

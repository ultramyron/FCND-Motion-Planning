## Project: 3D Motion Planning
#### 1. Explain the functionality of what's provided in `motion_planning.py` and `planning_utils.py`
First I will go over what is in the motion_planning script. The motion planning script, at a high level, generates a finite state machine embedded in a Python class. This class, called MotionPlanning, has six states which are Manual, Arming, Takeoff, Waypoint, Landing, Disarming, and Planning. 
Essentially, what the MotionPlanning class does is that once one state ends then another one will start. For example, the first state is the Manual state where nothing is happening. The MotionPlanning class has methods to transition from state to state depending on what state it currently is on right now. 
The final big aspect of the MotionPlanning class is the plan_path method. This method first extracts our current position and figures out what our start and end coordinates will be for our path planning algorithm. These coordinates along with our grid and heuristic function are fed into our A star algorithm where our waypoints will be outputted. The plan_path method willthen cull the waypoints by using the collinearity test or Bresenham's algorithm (I used the collinearity test in my implementation.)
The planning_utils script gives us the functions to create the grid, an Action class, a function to determine which actions are valid at a certain coordinate, the A star algorithm to calculate our waypoints, heuristic function to determine cost of each cell, and collinear test to prune our path.

#### 2. Setting home position
***Code Location: Lines 126 - 133 of motion_planning.py***
To accomplish setting the home position, I had to load the data from the colliders.csv file and pick the first two values of the sheet (which were the initial lateral and longitude values).
After allocating the two values into their own variables, I set the home position with self.set_home_position(initial longitude, initial latitude, initial altitude)


#### 3. Setting current local position
***Code Location: Line 136 of motion_planning.py***
To do this, I had to concatenate the current longitude, latitude and altitude (denoted by self._longitude, self._latitude, and self._altitude) into a numpy array. 
The function the translates global coordinates to local coordinates is the global_to_local function where it takes two parameters - current global position and global home position. In receiving this values, the function calculates the relative distance between the two coordinates and switches it to NED coordinates. This output is our local position.

#### 4. Set grid start position from local position
***Code Location: Line 143 of motion_planning.py***
Setting the current position as the start position is a matter of adding the local coordinates that we calcaulated from the last step with the north and east offsets. We have to floor() this addition because our A star algorithm only works with integers.

#### 5. Set grid goal position from geodetic coords
***Code Location: Lines 146 - 152 of motion_planning.py***
To accomplish this, we can use the global_to_local function to input any latitude/longitude coordinates and get the relative distance from our home position. After getting these relative coordinates, we can once again add them to the north and east offset to find our goal coordinates.

#### 6. Modify A* to include diagonal motion
***Code Location: Lines 110 - 159 of planning_utils.py***
Modifying A star to have diagonal motion first starts with adding North East, North West, South East, and South West motion into our Action class and change the cost to square root of two instead of one. After that, we have to add those diagonal movements into our valid actions function with their own set of conditions. After that, A star will now work with diagonal motion.

#### 6. Cull waypoints 
***Code Location: Lines 166 - 169 and Lines 174 - 187 of planning_utils.py***
Culling the waypoints starts with the collinear test which will see if three given points are collinear with each other or not. The collinear test does this by find the determinant of these three points and seeing if the determinant is zero. If the determinant is zero then that means the rank of this matrix is rank deficient, leading to a loss in dimensionality - meaning that the points are 1-D (collinear) instead of 2-D (not collinear).
After developing this collinear function we just loop through all the waypoints to see which points are collinear using a for loop.
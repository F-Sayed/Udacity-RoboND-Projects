Packages used in this project:

1. turtlebot/turtlebot_interactions/turtlebot_simulator
These packages are used to initialise our robot (roomba) and the playground world that  it will be exploring and performing algorithms learned in previous projects, namely SLAM and Localization.

2. slam_gmapping
This package is used to map the world as well as localize the robot in the world using AMCL.

3. pick_objects
This package is used to navigate the robot to the assigned pickup/dropoff locations in the world. The robot will first navigate to the pickup location and wait for 5 seconds (to simulate an action), and then navigate to its dropoff location.

4. add_markers
This package is used to create a virtual object simulated as a pick-up area rather than an actual object. This package will first publish the marker (green box) at the pickup location, and then be hidden until the robot finishes its action (same as pick_objects) and finally publish another marker at the drop location. Finally, the marker is hidden.

Note: add_markers is subscribed to 'odom' to read the pose of the robot, and calculate the euclidean distance between the robot to the pickup/drop zone.

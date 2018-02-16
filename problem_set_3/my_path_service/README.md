# my_path_service
This package has a client that lists points for the robot to go to, and it has a service that changes the robot's angle, calculates the distance to the next point, and goes there.

## Example usage
Eventually the client will read from a text file or get info from another path-making node, and then give the path in the correct format to the service that drives the robot.

## Running tests/demos
Start up the STDR simulator:
`roslaunch stdr_launchers server_with_map_and_gui_plus_robot.launch`
Start the path client:
 `rosrun my_path_service my_path_client`
Start the path service:
`rosrun my_path_service my_path_service`



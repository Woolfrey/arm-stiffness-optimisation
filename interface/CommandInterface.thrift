struct Transform
{
}
(
	yarp.name="yarp::sig::Matrix"
	yarp.includefile="yarp/sig/Matrix.h"
)

service CommandInterface
{
	bool perform_joint_space_action(1:string actionName);                                       # Move the joints by a prescribed action
	
	bool move_joints_to_position(1:list<double> position,
	                             2:double time);                                                # Move the joints to the given position
	                             
	bool optimise_stiffness(1:list<double> direction);                                           # Optimise the stiffness in the given direction
	
	void stop();                                                                                # Stop the robot moving immediately

	void shut_down();                                                                           # Shut down the command server
}

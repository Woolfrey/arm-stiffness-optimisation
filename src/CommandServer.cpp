    ///////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                               //
  //                    Enables interfacing with the iCub/ergoCub through YARP                     //
 //                                                                                               //
///////////////////////////////////////////////////////////////////////////////////////////////////

#include <CommandInterface.h>                                                                       // thrift-generated class
#include <iostream>                                                                                 // std::cerr, std::cout
#include <map>                                                                                      // std::map
#include <PositionControl.h>                                                                        // For control of ergoCub, iCub robots
#include <Utilities.h>                                                                              // JointTrajectory object structure
#include <yarp/os/Property.h>                                                                       // Load configuration files
#include <yarp/os/RpcServer.h>                                                                      // Allows communication over yarp ports

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                 Overrides the functions specified in CommandInterface.thrift                  //
///////////////////////////////////////////////////////////////////////////////////////////////////
class CommandServer : public CommandInterface
{
	public:
	
		CommandServer(PositionControl *_robot,
		              std::map<std::string,JointTrajectory> *_jointActionMap)
		             :
		             robot(_robot),
		             jointActionMap(_jointActionMap) {}        
		
		std::string errorMessage = "[ERROR] [iCUB COMMAND SERVER] ";
		
		////////// NOTE: These are directly copied from CommandInterface.h //////////

		// Move the configuration of the joints by a prescribed action
		bool perform_joint_space_action(const std::string& actionName)
		{
			auto jointConfig = this->jointActionMap->find(actionName);
			
			if(jointConfig == this->jointActionMap->end())
			{
				std::cerr << errorMessage << " move_to_named_configuration(): "
				          << "Could not find a joint configuration named '"
				          << actionName << "' in the list.\n";
				
				return false;
			}
			else
			{
				return this->robot->move_to_positions(jointConfig->second.waypoints,
				                                      jointConfig->second.times);
				return true;
			}
		}

		// Move the joints
		bool move_joints_to_position(const std::vector<double>& position, const double time)
		{
			return this->robot->move_to_position(Eigen::VectorXd::Map(&position[0], position.size()), time);
		}
		
		bool optimise_stiffness(const std::vector<double> &direction)
		{
			if(direction.size() != 6)
			{
				std::cerr << errorMessage << "Expected 6 elements for the the input vector "
				          << "but it had " << direction.size() << ".\n";
				
				return false;
			}
			else
			{
				Eigen::Matrix<double,6,1> temp(direction.data());
				
				return this->robot->optimise_stiffness(temp);
			}
		}

		void stop() { this->robot->halt(); }
		
		void shut_down() { this->serverActive = false; }	
		
		///////////////////// Not defined in CommandInterface.h ///////////////////////////
		bool is_active() const { return this->serverActive; }
	    
	private:
		bool serverActive = true;
		
		PositionControl *robot;                                                             // Pointer to robot object
		
		std::map<std::string, JointTrajectory> *jointActionMap;                             // Map of prescribed joint configurations
};                                                                                                  // Semicolon needed after class declaration

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                             MAIN                                               //
////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char* argv[])
{
	std::string errorMessage = "[ERROR] [iCUB COMMAND SERVER] ";
	
	// Default for argc is 1, but I don't know why ¯\_(ツ)_/¯
	if(argc != 5)
	{
		std::cerr << errorMessage << "Port name, path to URDF, and path to config file are required. "
		          << "Usage: ./command_server /serverPortName /robotPortName /path/to/model.urdf /path/to/config.ini\n";
		
		return 1;                                                                           // Close with error
	}

	std::string serverPortName  = argv[1];
	std::string robotPortPrefix = argv[2];                                                      // Get the port names
	std::string pathToURDF      = argv[3];                                                      // Get the file path
	std::string pathToConfig    = argv[4];                                                      // Path to the configuration file
	
	// Generate port list prefixes
	std::vector<std::string> portList;
	portList.push_back(robotPortPrefix + "/torso");
	portList.push_back(robotPortPrefix + "/left_arm");
	portList.push_back(robotPortPrefix + "/right_arm");
	
	yarp::os::Property parameter; parameter.fromConfigFile(pathToConfig);                       // Load the properties from the config file
	
	try // to start up the robot
	{
		std::string robotModel = parameter.find("model_name").asString();                   // Get the name
		
		// Get the joint list from the config file
		yarp::os::Bottle *bottle; bottle = parameter.find("joint_names").asList();
		
		if(bottle == nullptr)
		{
			std::cerr << errorMessage << "No list of joint names was specified in " << pathToConfig << ".\n";
			return 1;
		}
		
		std::vector<std::string> jointNames = string_from_bottle(bottle);                   // Function specified in Utils.h

		// Load the joint-space actions from the config file
		
		std::map<std::string, JointTrajectory> jointActionMap;                              // NOTE: JointTrajectory is a data structure in Utils.H
		
		bottle->clear(); bottle = &parameter.findGroup("JOINT_SPACE_ACTIONS");
		
		if(bottle == nullptr)
		{
			std::cerr << errorMessage << "No group called JOINT_SPACE_ACTIONS could be found in "
			          << pathToConfig << ".\n";
			
			return 1;
		}
		if(not load_joint_configurations(bottle,jointActionMap)) return 1;
	
		PositionControl robot(pathToURDF, jointNames, portList, robotModel);                // Start up the robot
		
		// Set the Cartesian gains
		double kp = parameter.findGroup("CARTESIAN_GAINS").find("proportional").asFloat64();
		double kd = parameter.findGroup("CARTESIAN_GAINS").find("derivative").asFloat64();	
		if(not robot.set_cartesian_gains(kp,kd)) return 1;
		
		// Set the joint gains
		kp = parameter.findGroup("JOINT_GAINS").find("proportional").asFloat64();
		kd = parameter.findGroup("JOINT_GAINS").find("derivative").asFloat64();
		if(not robot.set_joint_gains(kp,kd)) return 1;
		
		// Set the singularity avoidance parameters
		double maxDamping = parameter.findGroup("SINGULARITY_AVOIDANCE").find("maxDamping").asFloat64();
		double threshold  = parameter.findGroup("SINGULARITY_AVOIDANCE").find("threshold").asFloat64();
		if(not robot.set_singularity_avoidance_params(maxDamping,threshold)) return 1;
		
		// Set the redundant task parameters
		double scalar = parameter.findGroup("REDUNDANT_TASK").find("scalar").asFloat64();
		std::string task = parameter.findGroup("REDUNDANT_TASK").find("task").asString();
		
		if(not robot.set_redundant_task(task,scalar))
		{
			std::cerr << "[ERROR] [iCUB COMMAND SERVER] Unable to set the redundant task.\n";
			
			return 1;
		}		
		
		// Set the desired position for the joints when running in Cartesian mode
		bottle->clear(); bottle = parameter.find("desired_position").asList();
		if(bottle == nullptr)
		{
			std::cerr << "[ERROR] [iCUB COMMAND SERVER] "
			          << "Couldn't find 'desired_position' listed in the config file.\n";
			return 1;
		}
		robot.set_desired_joint_position(vector_from_bottle(bottle));
		
		
		// Establish communication over YARP
		yarp::os::Network yarp;
		yarp::os::Port port;
		CommandServer commandServer(&robot, &jointActionMap);  // Create command server
		                            
		commandServer.yarp().attachAsServer(port);
		
		if(not port.open(serverPortName))
		{
			std::cerr << errorMessage << "Could not open port /commandServer.\n";
			return 1;
		}
		
		while(commandServer.is_active())
		{
			std::cout << "\nWorker bees can leave.\n";
			yarp::os::Time::delay(5);
			std::cout << "Even drones can fly away.\n";
			yarp::os::Time::delay(5);
			std::cout << "The Queen is their slave.\n";
			yarp::os::Time::delay(20);
		}
		
		std::cout << "[INFO] [iCUB COMMAND SERVER] Shutting down. Arrivederci.\n";
		
		port.close();
		
		robot.close();
		
		return 0;                                                                           // No problems with main
	}
	catch(std::exception &exception)
	{
		std::cerr << errorMessage << "There was a problem with initialization.\n";
		          
		std::cout << exception.what() << std::endl;                                         // Inform the user
		
		return 1;                                                                           // Close with error
	}
}

#include <PositionControl.h>


  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                          Optimise stiffness in the given direction                             //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool PositionControl::optimise_stiffness(const Eigen::Matrix<double,6,1> &direction)
{
	if(isRunning()) stop();
	
	this->_direction = direction;

	this->controlSpace = cartesian;                                                                 // Switch to joint control mode
	
	start();
	
	return true;
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                        Constructor                                             //
////////////////////////////////////////////////////////////////////////////////////////////////////
PositionControl::PositionControl(const std::string              &pathToURDF,
	                         const std::vector<std::string> &jointList,
	                         const std::vector<std::string> &portList,
	                         const std::string              &robotModel)
		                 :
		                 iCubBase(pathToURDF, jointList, portList, robotModel)
{
	// YARP ports
	this->actualPosition.open("/actualPosition");
	this->referencePosition.open("/referencePosition");
	this->positionError.open("/positionError");	
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                 Initialise the control thread                                  //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool PositionControl::threadInit()
{
	if(isRunning())
	{
		std::cout << "[ERROR] [POSITION CONTROL] threadInit(): "
		          << "A control thread is still running!\n";
		return false;
	}
	else
	{
		// Reset values
		QPSolver::clear_last_solution();                                                    // Remove last solution
		this->isFinished = false;                                                           // New action started
		this->qRef = this->q;                                                               // Start from current joint position
		this->startTime = yarp::os::Time::now();                                            // Used to time the control loop
		return true;                                                                        // jumps immediately to run()
	}
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                                     MAIN CONTROL LOOP                                         //
///////////////////////////////////////////////////////////////////////////////////////////////////
void PositionControl::run()
{
	if(update_state())
	{
		double elapsedTime = yarp::os::Time::now() - this->startTime;                       // Time since start of control
		
		if(elapsedTime > this->endTime) this->isFinished = true;                            
		
		Eigen::VectorXd dq(this->numJoints); dq.setZero();                                  // We want to solve for this		
		
		if(this->controlSpace == joint)
		{
			Eigen::VectorXd desiredPosition(this->numJoints);                           // From the trajectory object
			Eigen::VectorXd lowerBound(this->numJoints);                                // Lower limit on joint motion
			Eigen::VectorXd upperBound(this->numJoints);                                // Upper limit on joint motion
				
			for(int i = 0; i < this->numJoints; i++)
			{
				desiredPosition(i) = this->jointTrajectory[i].evaluatePoint(elapsedTime); // From the trajectory object
				
				compute_joint_limits(lowerBound(i),upperBound(i),i);                // Instantaneous limits on the joint step				
			}

			for(int i = 0; i < this->numJoints; i++)
			{
				dq(i) = desiredPosition(i) - this->qRef(i);                         // Difference between current reference point and desired
				
				     if(dq(i) <= lowerBound(i)) dq(i) = lowerBound(i) + 0.01;       // Just above the lower bound
				else if(dq(i) >= upperBound(i)) dq(i) = upperBound(i) - 0.01;       // Just below the upper bound
			}
		}
		else // this->controlSpace == Cartesian
		{
			double manipulability = sqrt((this->J*this->J.transpose()).determinant());
			
			if(manipulability > this->threshold)
			{
				Eigen::Vector3d t = this->rightPose.translation();
				
				Eigen::Vector2d e;
				e(0) = this->yd - t(1);
				e(1) = this->zd - t(2);
				
				//std::cout << "Here is the pose error:\n";
				//std::cout << e.transpose() << std::endl;
				
				Eigen::Vector2d dx;
				dx = this->kp*e;
				
				Eigen::MatrixXd Jacobian = this->J.block(6,10,6,7);
				
				Eigen::VectorXd lowerBound(7), upperBound(7);
				
				Eigen::VectorXd redundantTask(7);
				
				Eigen::VectorXd wrench = this->K*this->_direction;
				
				Eigen::VectorXd tau = Jacobian.transpose()*wrench;
				
				for(int i = 0; i < 7; i++)
				{
					compute_joint_limits(lowerBound(i),upperBound(i),10+i);
					
					Eigen::MatrixXd dJ = partial_derivative(Jacobian,i);
					
					//std::cout << "Here is the partial derivative of the Jacobian with respect to joint " << i << std::endl;
					//std::cout << dJ << std::endl;
					
					Eigen::VectorXd v = dJ.transpose()*wrench;
					
					redundantTask(i) = -this->kr*v.dot(tau);
				}
					
			/*	std::cout << "Here is the direction:\n";
				std::cout << this->_direction.transpose() << std::endl;
				std::cout << "Here is the wrench:\n";
				std::cout << wrench.transpose() << std::endl;
				std::cout << "Here is the torque:\n";
				std::cout << tau.transpose() << std::endl;
				std::cout << "Here is the redundant task:\n";
				std::cout << redundantTask << std::endl;*/
				
				
				Eigen::VectorXd startPoint;
				
				if(QPSolver::last_solution_exists())
				{
					startPoint = QPSolver::last_solution().tail(7);
					
					for(int i = 0; i < 7; i++)
					{
						     if(startPoint(i) <= lowerBound(i)) startPoint(i) = lowerBound(i) + 0.001;
						else if(startPoint(i) >= upperBound(i)) startPoint(i) = upperBound(i) - 0.001;
					}
				}
				else startPoint = 0.5*(lowerBound + upperBound);
				
				try
				{
					dq.tail(7) = QPSolver::redundant_least_squares( redundantTask,
					                                                this->M.block(10,10,7,7),
					                                                dx,
					                                                Jacobian.block(1,0,2,7),
					                                                lowerBound,
					                                                upperBound,
					                                                startPoint);
				}
				catch(const std::exception &exception)
				{
					std::cout << exception.what() << std::endl;
				}
			}
		}
	
		this->qRef += dq;                                                                   // Update reference position for joint motors
		
		if(not send_joint_commands(qRef)) std::cout << "[ERROR] [POSITION CONTROL] Could not send joint commands for some reason.\n";
		
		// Publish data for analysis
		yarp::sig::Vector &actualVector = this->actualPosition.prepare();
		yarp::sig::Vector &referenceVector = this->referencePosition.prepare();
		yarp::sig::Vector &errorVector = this->positionError.prepare();
		
		for(int i = 0; i < this->numJoints; i++)
		{
			actualVector.push_back(this->q(i));
			referenceVector.push_back(this->qRef(i));
			errorVector.push_back(this->qRef(i) - this->q(i));
		}
		
		this->actualPosition.write();
		this->referencePosition.write();
		this->positionError.write();
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                             Executed after a control thread is stopped                         //
////////////////////////////////////////////////////////////////////////////////////////////////////
void PositionControl::threadRelease()
{
	send_joint_commands(this->q);                                                               // Maintain current joint positions
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                             Compute instantenous position limits                               //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool PositionControl::compute_joint_limits(double &lower, double &upper, const unsigned int &jointNum)
{
	// NOTE TO FUTURE SELF: Need to compute a limit on the step size dq 
	
	if(jointNum > this->numJoints)
	{
		std::cerr << "[ERROR] [POSITION CONTROL] compute_joint_limits(): "
		          << "Range of joint indices is 0 to " << this->numJoints - 1 << ", "
		          << "but you called for " << jointNum << ".\n";

		return false;
	}
	else
	{
		                 
		 lower = this->positionLimit[jointNum][0] - this->qRef[jointNum];
		 upper = this->positionLimit[jointNum][1] - this->qRef[jointNum];
		
		if(lower >= upper)
		{
			std::cerr << "[ERROR] [POSITION CONTROL] compute_joint_limits(): "
				  << "Lower limit " << lower << " is greater than upper limit " << upper << ". "
				  << "How did that happen???\n";
			
			return false;
		}
		else	return true;
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                        Solve a discrete time step for Cartesian control                        //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix<double,12,1> PositionControl::track_cartesian_trajectory(const double &time)
{
	// NOTE TO FUTURE SELF:
	// There are no checks here to see if the trajectory is queried correctly.
	// This could cause problems later
	
	// Variables used in this scope
	Eigen::Matrix<double,12,1> dx; dx.setZero();                                                // Value to be returned
	Eigen::Isometry3d pose;                                                                     // Desired pose
	Eigen::Matrix<double,6,1> vel, acc;                                                         // Desired velocity & acceleration
	
	if(this->isGrasping)
	{
		this->payloadTrajectory.get_state(pose,vel,acc,time);                               // Get the desired object state for the given time              
		
		dx = this->G.transpose()*(this->dt*vel + this->K*pose_error(pose,this->payload.pose())); // Feedforward + feedback control		
	}
	else
	{
		this->leftTrajectory.get_state(pose,vel,acc,time);                                  // Desired state for the left hand
		dx.head(6) = this->dt*vel + this->K*pose_error(pose,this->leftPose);                // Feedforward + feedback on the left hand

		this->rightTrajectory.get_state(pose,vel,acc,time);                                 // Desired state for the right hand
		dx.tail(6) = this->dt*vel + this->K*pose_error(pose,this->rightPose);               // Feedforward + feedback on the right hand
	}
	
	return dx;
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                     Solve the step size to track the joint trajectory                          //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXd PositionControl::track_joint_trajectory(const double &time)
{
	Eigen::VectorXd dq(this->numJoints); dq.setZero();                                          // Value to be returned
	
	for(int i = 0; i < this->numJoints; i++) dq[i] = this->jointTrajectory[i].evaluatePoint(time) - this->q[i];
	
	return dq;
}



    ///////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                               //
  //                         Position control functions for the iCub/ergoCub                       //
 //                                                                                               //
///////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef POSITION_CONTROL_H_
#define POSITION_CONTROL_H_

#include <iCubBase.h>
#include <yarp/os/BufferedPort.h>                                                                   // For publishing data
#include <yarp/sig/Vector.h>                                                                        // For publishing data

class PositionControl : public iCubBase
{
	public:
		PositionControl(const std::string              &pathToURDF,
			        const std::vector<std::string> &jointList,
			        const std::vector<std::string> &portList,
			        const std::string              &robotModel);


		//////////////////////// Inherited from iCubBase class ////////////////////////////
		bool compute_joint_limits(double &lower, double &upper, const unsigned int &jointNum);
		
		Eigen::VectorXd track_joint_trajectory(const double &time);
		
		Eigen::Matrix<double,12,1> track_cartesian_trajectory(const double &time);
		///////////////////////////////////////////////////////////////////////////////////
		
		bool set_desired_point(const double &y, const double &z);
		
		bool optimise_stiffness(const Eigen::Matrix<double,6,1> &direction);
		
	protected:
	
		double yd = 0.0;
		double zd = 0.30;
		
		Eigen::Matrix<double,6,1> _direction;
		
		Eigen::VectorXd qRef;                                                               // Reference joint position to send to motors
		
		yarp::os::BufferedPort<yarp::sig::Vector> actualPosition, referencePosition, positionError;                       
		
		/////////////////////// Inherited from PeriodicThread class ///////////////////////
		bool threadInit();
		void run();
		void threadRelease();	
};                                                                                                  // Semicolon needed after class declaration

#endif

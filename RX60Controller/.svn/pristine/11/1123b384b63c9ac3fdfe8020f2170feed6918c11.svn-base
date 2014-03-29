//
// C++ Interface: RX60Driver
//
// Description: 
//
//
// Author: kraft,,, <kraft@tek-4714>, (C) 2008
//
//
//

#ifndef RX60DRIVER_HPP_
#define RX60DRIVER_HPP_

#include "ros/ros.h"
#include "std_msgs/String.h"

#include "RX60TCPSocket.hpp"
#include "rx60controller/command.h"

class RX60Driver {

// Variable definition
public:
	/// Type of coordinates
	enum coordinateType {
		/// Joint angle movement
		eCT_JOINT_ANGLES = 0,
		/// Cartesian movement
		eCT_CARTESIAN_COORDINATES = 1,
		/// Tool relative (differential coordinates)
		eCT_TOOLRELATIVE = 2,
	};

	enum interpolationType {
		/// straight line in joint space
		eIT_JOINT_SPACE = 0,
		/// straight line (of flange/tool) in Cartesian space
		eIT_CARTESIAN_SPACE = 1
	};

	enum digitalOutput {
		eDO_VALVE1 = 1,
		eDO_VALVE2 = 2
	};

	/// possible instructions encoded as number to be sent to the CS8
	enum instructions {
		eINSTR_MOVE = 0,
		eINSTR_GET_POSITION = 1,
		eINSTR_SET_MOVE_PARAMS = 2,
		eINSTR_GET_MOVE_PARAMS = 3,
		eINSTR_GET_MOVE_QUEUE_EMPTY = 4,
		eINSTR_GET_IS_SETTLED = 5,
		eINSTR_RESET_MOTION = 6,
		eINSTR_SET_DIGITAL_OUTPUT = 7,
		eINSTR_GET_DIGITAL_OUTPUT = 8,
		eINSTR_SET_RETREAT = 12
	};

	enum moveParams {
		eMP_VEL_P = 1,
		eMP_ACC_P = 2,
		eMP_DEACC_P = 4,
		eMP_VELR = 8,
		eMP_VELT = 16,
		eMP_BLEND = 32,
		eMP_REACH = 64,
		eMP_LEAVE = 128
	};

	  std::string rx60_controller_ip;
	  std::string rx60_controller_port;

protected:

private:
	RX60TCPSocket _socket;

// Function definition
public:
	RX60Driver(const std::string serverAddress = "172.16.1.1",
			const std::string serverPort = "22222");
	~RX60Driver();

	bool connect(const std::string serverAddress = "172.16.1.1",
			const std::string serverPort = "22222");

	bool isConnected();

	bool move(const double * targetCoordinates, const coordinateType ctype,
			const bool block, const interpolationType itype);

	bool getPosition(double * coordinates, const coordinateType ctype);

	bool setMovementParameters(const int selection, double velocityPercentage,
			double accelerationPercentage, double decelerationPercentage,
			double maxRotationalVelocity, double maxCartesianVelocity,
			const bool blend, double reach, double leave);

	bool setMovementParameters(double velocityPercentage,
			double accelerationPercentage, double decelerationPercentage,
			double maxRotationalVelocity, double maxCartesianVelocity,
			const bool blend, double reach, double leave);

	bool setMaxSpeedPercentage(double velocityPercentage);

	bool setMaxAccelerationPercentage(double accelerationPercentage);

	bool setMaxDecelerationPercentage(double decelerationPercentage);

	bool getMovementParameters(double & velocityPercentage,
			double & accelerationPercentage, double & decelerationPercentage,
			double & maxRotationalVelocity, double & maxCartesianVelocity,
			bool & blend, double & reach, double & leave);

	bool getMaxSpeedPercentage(double & velocityPercentage);

	/// (Old isMoving command)
	bool movementQueueEmpty(bool & empty);

	/// Checks if the robot is stopped and the position has stabilized
	/// Not sure about movement authorization
	/// See VAL3 manual
	bool isSettled(bool & settled);

	/// Stops the movement of the arm and cancels all further movement commands.
	/// Authorizes further movements (pause/resume)
	/// (Old stop command)
	bool resetMotion(void);

	bool setRetreatConfiguration(bool mode, const double * retreatCoordinates);

	bool setRetreatConfiguration(bool mode);

	bool setDigitalOutput(int digitalOutput, bool value);

	/// ROS STUFF
	bool commandServiceHandler(	rx60controller::command::Request  &req,
	         	 	 	 	 	 	 	 	rx60controller::command::Response &res);
	void moveJoints();

	void movePositions();

	ros::ServiceServer rx60_service;

protected:

private:
	void checkBounds(double & value, const double max, const double min,
			const std::string variableName);
	std::vector<double> _positionVector; 
};

#endif

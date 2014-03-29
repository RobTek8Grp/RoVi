//
// C++ Implementation: RX60Driver
//
// Description: 
//
//
// Author: kraft,,, <kraft@tek-4714>, (C) 2008
//
//
//

#include "RX60Driver.hpp"
#include "RX60TCPSocket.hpp"
#include <sstream>
#include <float.h>

RX60Driver::RX60Driver(const std::string serverAddress,
		const std::string serverPort) :
		_socket() {

	_socket.open(serverAddress, serverPort);

}

RX60Driver::~RX60Driver() {

	_socket.close();

}

bool RX60Driver::isConnected() {
	return _socket.isConnected();
}

bool RX60Driver::connect(const std::string serverAddress,
		const std::string serverPort) {

	if (isConnected()) {
		std::cout << "Connection is already open!" << std::endl;
		return false;
	} else {
		return _socket.open(serverAddress, serverPort);
	}

}

bool RX60Driver::move(const double * targetCoordinates,
		const coordinateType ctype, const bool block,
		const interpolationType itype) {

	if ((ctype == eCT_JOINT_ANGLES) && (itype == eIT_CARTESIAN_SPACE)) {
		std::cout
				<< "The robot controller does not support interpolation in Cartesian space with joint coordinates!"
				<< std::endl;
		std::cout
				<< "Just using the direct kinematic here could lead to unexpected results. Therefore this is not supported by the low level driver!"
				<< std::endl;
		return false;
	}

	std::ostringstream buf;

	buf << std::noshowpoint;
	buf << std::noboolalpha;

	buf << (int) eINSTR_MOVE;

	buf << " ";

	buf << (int) ctype;

	buf << " ";

	buf << (int) itype;

	buf << " ";

	buf << block;

	buf.precision(4);

	for (int i = 0; i < 6; i++) {
		buf << " ";
		buf << targetCoordinates[i];
	}

	buf << "\n";

	std::string buffer;

	_socket.write(buf.str());

	buffer = "";

	int bytesRead = _socket.read(buffer);

	while (bytesRead < 1) {
		bytesRead = _socket.read(buffer);
	}

	std::istringstream stream(buffer);

	int command = -1, result = -1;

	stream >> command;

	stream >> result;

	if (command != eINSTR_MOVE) {
		std::cout << "Echoed Command " << command << " is wrong in "
				<< __PRETTY_FUNCTION__ << std::endl;
		return false;
	} else {
		if (result != 0) {
			std::cout << "Result " << result << " indicates an error in "
					<< __PRETTY_FUNCTION__ << std::endl;
			return false;
		}
		return true;
	}

}

bool RX60Driver::getPosition(double * coordinates, const coordinateType ctype) {

	if ((ctype != eCT_JOINT_ANGLES) && (ctype != eCT_CARTESIAN_COORDINATES)) {

		std::cout
				<< __PRETTY_FUNCTION__
				<< " is only defined for joint angles and cartesian coordinates!"
				<< std::endl;

		return false;

	}

	std::ostringstream buf;

	buf << std::noshowpoint << std::noboolalpha;

	buf << (int) eINSTR_GET_POSITION << " " << (int) ctype << "\n";

	_socket.write(buf.str());

	std::string buffer;

	buffer = "";

	int bytesRead = _socket.read(buffer);

	while (bytesRead < 1) {
		bytesRead = _socket.read(buffer);
	}

	std::istringstream stream(buffer);

	int command = -1, result = -1;

	stream >> command;

	stream >> result;

	bool parseError = false;

	for (int i = 0; i < 6; i++) {
		coordinates[i] = DBL_MAX;
		stream >> coordinates[i];
		if (coordinates[i] == DBL_MAX) {
			parseError = true;
		}
	}

	if (command != eINSTR_GET_POSITION) {
		std::cout << "Echoed Command " << command << " is wrong in "
				<< __PRETTY_FUNCTION__ << std::endl;
		return false;
	} else {
		if (result != 0) {
			std::cout << "Result " << result << " indicates an error in "
					<< __PRETTY_FUNCTION__ << std::endl;
			return false;
		}
	}

	if (parseError) {
		std::cout << "Parse error in " << __PRETTY_FUNCTION__ << std::endl;
	}

	return !parseError;
}

bool RX60Driver::setMovementParameters(const int selection,
		double velocityPercentage, double accelerationPercentage,
		double decelerationPercentage, double maxRotationalVelocity,
		double maxCartesianVelocity, const bool blend, double reach,
		double leave) {

	checkBounds(velocityPercentage, 10000, 0.01, "velocityPercentage");
	checkBounds(accelerationPercentage, 10000, 0.01, "accelerationPercentage");
	checkBounds(decelerationPercentage, 10000, 0.01, "decelerationPercentage");

	checkBounds(maxRotationalVelocity, 9999999999.0, 0,
			"maxRotationalVelocity");
	checkBounds(maxCartesianVelocity, 9999999999.0, 0, "maxCartesianVelocity");

	checkBounds(reach, 9999999999.0, 0, "reach");
	checkBounds(leave, 9999999999.0, 0, "leave");

	std::ostringstream buf;

	buf << std::noshowpoint << std::noboolalpha;

	buf.precision(4);

	buf << (int) eINSTR_SET_MOVE_PARAMS << " " << selection << " "
			<< velocityPercentage << " " << accelerationPercentage << " "
			<< decelerationPercentage << " " << maxRotationalVelocity << " "
			<< maxCartesianVelocity << " " << blend << " " << reach << " "
			<< leave << "\n";

	_socket.write(buf.str());

	std::string buffer;

	buffer = "";

	int bytesRead = _socket.read(buffer);

	while (bytesRead < 1) {
		bytesRead = _socket.read(buffer);
	}

	std::istringstream stream(buffer);

	int command = -1, result = -1;

	stream >> command;

	stream >> result;

	if (command != eINSTR_SET_MOVE_PARAMS) {
		std::cout << "Echoed Command " << command << " is wrong in "
				<< __PRETTY_FUNCTION__ << std::endl;
		return false;
	} else {
		if (result != 0) {
			std::cout << "Result " << result << " indicates an error in "
					<< __PRETTY_FUNCTION__ << std::endl;
			return false;
		}
	}

	return true;

}

bool RX60Driver::setMovementParameters(double velocityPercentage,
		double accelerationPercentage, double decelerationPercentage,
		double maxRotationalVelocity, double maxCartesianVelocity,
		const bool blend, double reach, double leave) {

	return setMovementParameters(
			eMP_VEL_P | eMP_ACC_P | eMP_DEACC_P | eMP_VELR | eMP_VELT
					| eMP_BLEND | eMP_REACH | eMP_LEAVE, velocityPercentage,
			accelerationPercentage, decelerationPercentage,
			maxRotationalVelocity, maxCartesianVelocity, blend, reach, leave);

}

bool RX60Driver::setMaxSpeedPercentage(double velocityPercentage) {

	return setMovementParameters(eMP_VEL_P, velocityPercentage, 1.0, 1.0, 1.0,
			1.0, false, 1.0, 1.0);
}

bool RX60Driver::setMaxAccelerationPercentage(double accelerationPercentage) {

	return setMovementParameters(eMP_ACC_P, 1.0, accelerationPercentage, 1.0,
			1.0, 1.0, false, 1.0, 1.0);
}

bool RX60Driver::setMaxDecelerationPercentage(double decelerationPercentage) {

	return setMovementParameters(eMP_DEACC_P, 1.0, 1.0, decelerationPercentage,
			1.0, 1.0, false, 1.0, 1.0);
}

bool RX60Driver::getMovementParameters(double & velocityPercentage,
		double & accelerationPercentage, double & decelerationPercentage,
		double & maxRotationalVelocity, double & maxCartesianVelocity,
		bool & blend, double & reach, double & leave) {

	std::ostringstream buf;

	buf << std::noshowpoint << std::noboolalpha;

	buf << (int) eINSTR_GET_MOVE_PARAMS << "\n";

	_socket.write(buf.str());

	std::string buffer;

	buffer = "";

	int bytesRead = _socket.read(buffer);

	while (bytesRead < 1) {
		bytesRead = _socket.read(buffer);
	}

	std::istringstream stream(buffer);

	int command = -1, result = -1;

	stream >> command;

	stream >> result;

	bool parseError = false;

	velocityPercentage = DBL_MAX;
	accelerationPercentage = DBL_MAX;
	decelerationPercentage = DBL_MAX;
	maxRotationalVelocity = DBL_MAX;
	maxCartesianVelocity = DBL_MAX;
	double blendD = DBL_MAX;
	reach = DBL_MAX;
	leave = DBL_MAX;

	stream >> velocityPercentage >> accelerationPercentage
			>> decelerationPercentage >> maxRotationalVelocity
			>> maxCartesianVelocity >> blendD >> reach >> leave;

	if (velocityPercentage == DBL_MAX)
		parseError = true;

	if (accelerationPercentage == DBL_MAX)
		parseError = true;

	if (decelerationPercentage == DBL_MAX)
		parseError = true;

	if (maxCartesianVelocity == DBL_MAX)
		parseError = true;

	if (velocityPercentage == DBL_MAX)
		parseError = true;

	if (reach == DBL_MAX)
		parseError = true;

	if (leave == DBL_MAX)
		parseError = true;

	if (blendD == 0) {
		blend = false;
	} else if (blendD == 1) {
		blend = true;
	} else {
		parseError = true;
	}

	if (command != eINSTR_GET_MOVE_PARAMS) {
		std::cout << "Echoed Command " << command << " is wrong in "
				<< __PRETTY_FUNCTION__ << std::endl;
		return false;
	} else {
		if (result != 0) {
			std::cout << "Result " << result << " indicates an error in "
					<< __PRETTY_FUNCTION__ << std::endl;
			return false;
		}
	}

	if (parseError) {
		std::cout << "Parse error in " << __PRETTY_FUNCTION__ << std::endl;
	}

	return !parseError;

}

bool RX60Driver::getMaxSpeedPercentage(double & velocityPercentage) {

	double dummy1, dummy2, dummy3, dummy4, dummy5, dummy6;
	bool bDummy;

	return getMovementParameters(velocityPercentage, dummy1, dummy2, dummy3,
			dummy4, bDummy, dummy5, dummy6);

}

bool RX60Driver::movementQueueEmpty(bool & empty) {

	std::ostringstream buf;

	buf << std::noshowpoint;
	buf << std::noboolalpha;

	buf << (int) eINSTR_GET_MOVE_QUEUE_EMPTY;

	buf << "\n";

	std::string buffer;

	_socket.write(buf.str());

	buffer = "";

	int bytesRead = _socket.read(buffer);

	while (bytesRead < 1) {
		bytesRead = _socket.read(buffer);
	}

	std::istringstream stream(buffer);

	int command = -1;
	double emptyD = -1;

	stream >> command;

	stream >> emptyD;

	bool parseError = false;

	if (emptyD == 0) {
		empty = false;
	} else if (emptyD == 1) {
		empty = true;
	} else {
		parseError = true;
	}

	if (command != eINSTR_GET_MOVE_QUEUE_EMPTY) {
		std::cout << "Echoed Command " << command << " is wrong in "
				<< __PRETTY_FUNCTION__ << std::endl;
		return false;
	}

	if (parseError) {
		std::cout << "Parse error in " << __PRETTY_FUNCTION__ << std::endl;
	}

	return !parseError;

}

bool RX60Driver::isSettled(bool & settled) {

	std::ostringstream buf;

	buf << std::noshowpoint;
	buf << std::noboolalpha;

	buf << (int) eINSTR_GET_IS_SETTLED;

	buf << "\n";

	std::string buffer;

	_socket.write(buf.str());

	buffer = "";

	int bytesRead = _socket.read(buffer);

	while (bytesRead < 1) {
		bytesRead = _socket.read(buffer);
	}

	std::istringstream stream(buffer);

	int command = -1;
	double settledD = -1;

	stream >> command;

	stream >> settledD;

	bool parseError = false;

	if (settledD == 0) {
		settled = false;
	} else if (settledD == 1) {
		settled = true;
	} else {
		parseError = true;
	}

	if (command != eINSTR_GET_IS_SETTLED) {
		std::cout << "Echoed Command " << command << " is wrong in "
				<< __PRETTY_FUNCTION__ << std::endl;
		return false;
	}

	if (parseError) {
		std::cout << "Parse error in " << __PRETTY_FUNCTION__ << std::endl;
	}

	return !parseError;

}

bool RX60Driver::resetMotion(void) {

	std::ostringstream buf;

	buf << std::noshowpoint;
	buf << std::noboolalpha;

	buf << (int) eINSTR_RESET_MOTION;

	buf << "\n";

	std::string buffer;

	_socket.write(buf.str());

	buffer = "";

	int bytesRead = _socket.read(buffer);

	while (bytesRead < 1) {
		bytesRead = _socket.read(buffer);
	}

	std::istringstream stream(buffer);

	int command = -1;

	stream >> command;

	if (command != eINSTR_RESET_MOTION) {
		std::cout << "Echoed Command " << command << " is wrong in "
				<< __PRETTY_FUNCTION__ << std::endl;
	}

	return (command == eINSTR_RESET_MOTION);

}

bool RX60Driver::setDigitalOutput(int digitalOutput, bool value) {

	std::ostringstream buf;

	buf << std::noshowpoint;
	buf << std::noboolalpha;

	buf << (int) eINSTR_SET_DIGITAL_OUTPUT;

	buf << " ";

	switch (digitalOutput) {
		case eDO_VALVE1:
			buf << (int) eDO_VALVE1;
			break;
		case eDO_VALVE2:
			buf << (int) eDO_VALVE2;
			break;
		default:
			std::cout << "setDigitalOutput called with invalid output" << std::endl;
			break;
	}

	buf << " ";

	buf << (int) value;

	buf << "\n";

	std::string buffer;

	_socket.write(buf.str());

	buffer = "";

	int bytesRead = _socket.read(buffer);

	while (bytesRead < 1) {
		bytesRead = _socket.read(buffer);
	}

	std::istringstream stream(buffer);

	int command = -1;

	stream >> command;

	if (command != eINSTR_SET_DIGITAL_OUTPUT) {
		std::cout << "Echoed Command " << command << " is wrong in "
				<< __PRETTY_FUNCTION__ << std::endl;
	}

	return (command == eINSTR_SET_DIGITAL_OUTPUT);

}

bool RX60Driver::setRetreatConfiguration(bool mode,
		const double * retreatCoordinates) {

	std::ostringstream buf;

	buf << std::noshowpoint;
	buf << std::noboolalpha;

	buf << (int) eINSTR_SET_RETREAT << " ";

	buf << mode;

	buf << " ";

	buf.precision(4);

	for (int i = 0; i < 6; i++) {
		buf << " ";
		buf << retreatCoordinates[i];
	}

	buf << "\n";

	std::string buffer;

	_socket.write(buf.str());

	buffer = "";

	int bytesRead = _socket.read(buffer);

	while (bytesRead < 1) {
		bytesRead = _socket.read(buffer);
	}

	std::istringstream stream(buffer);

	int command = -1, result = -1;

	stream >> command;

	stream >> result;

	if (command != eINSTR_SET_RETREAT) {
		std::cout << "Echoed Command " << command << " is wrong in "
				<< __PRETTY_FUNCTION__ << std::endl;
		return false;
	}

	if (result != 0) {
		std::cout << "Result " << result << " indicates an error in "
				<< __PRETTY_FUNCTION__ << std::endl;
		return false;
	}

	return true;
}

bool RX60Driver::setRetreatConfiguration(bool mode) {
	if (mode) {
		std::cout
				<< "Please use the function to specify targetCoordinates as well if you want to switch on retreat!"
				<< std::endl;
		return false;
	}
	double a[] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

	return setRetreatConfiguration(mode, a);
}

void RX60Driver::checkBounds(double & value, const double max, const double min,
		const std::string variableName) {

	if (value > max) {
		value = max;
		std::cout << variableName << " changed to " << value << std::endl;
	} else if (value < min) {
		value = min;
		std::cout << variableName << " changed to " << value << std::endl;
	}
}

bool RX60Driver::commandServiceHandler(	rx60controller::command::Request  &req,
         	 	 	 	 	 	 	 	 	rx60controller::command::Response &res)
{
	std::cout <<"RX60Driver commandServiceHandler" << std::endl;	
	if (!isConnected())
	{
		connect(rx60_controller_ip, rx60_controller_port);
	}
	if (isConnected())
	{
		double joints[6];
		double position[6];
		switch (req.command_number) {
				case rx60controller::command::Request::GET_JOINT_CONFIGURATION:
					getPosition(joints, eCT_JOINT_ANGLES);
					res.joint1 = joints[0];
					res.joint2 = joints[1];
					res.joint3 = joints[2];
					res.joint4 = joints[3];
					res.joint5 = joints[4];
					res.joint6 = joints[5];
					break;
				case rx60controller::command::Request::SET_JOINT_CONFIGURATION:
					joints[0] = req.joint1;
					joints[1] = req.joint2;
					joints[2] = req.joint3;
					joints[3] = req.joint4;
					joints[4] = req.joint5;
					joints[5] = req.joint6;
					std::cout <<"set joint " << joints[0] <<" " << req.joint1 << std::endl;
				//	joints[0] = -10.0;
					move(joints, RX60Driver::eCT_JOINT_ANGLES, false,
								 RX60Driver::eIT_JOINT_SPACE);
					break;
				case rx60controller::command::Request::SET_TOOL_FLANGE_CARTESIAN_POSITION:
					position[0] = req.x;
					position[1] = req.y;
					position[2] = req.z;
					position[3] = req.theta_x;
					position[4] = req.theta_y;
					position[5] = req.theta_z;
					move(position, RX60Driver::eCT_CARTESIAN_COORDINATES, false,
								 RX60Driver::eIT_CARTESIAN_SPACE);
					break;
				case rx60controller::command::Request::GET_PARAMETERS:
					bool blend;
					getMovementParameters(	res.velocity_procentage,
											res.acceleration_procentage,
											res.deceleration_procentage,
											res.max_rotational_velocity,
											res.max_cartesian_velocity,
											blend,
											res.reach,
											res.leave);
					res.blend = (uint8_t)blend;
					break;
				case rx60controller::command::Request::SET_PARAMETERS:
					setMovementParameters(	req.velocity_procentage,
											req.acceleration_procentage,
											req.deceleration_procentage,
											req.max_rotational_velocity,
											req.max_cartesian_velocity,
											req.blend,
											req.reach,
											req.leave);
					break;
				case rx60controller::command::Request::GET_TOOL_FLANGE_POSITION:
					getPosition(position,eCT_CARTESIAN_COORDINATES);
					res.x = position[0];
					res.y = position[1];
					res.z = position[2];
					res.theta_x = position[3];
					res.theta_y = position[4];
					res.theta_z = position[5];
					break;

				case rx60controller::command::Request::IS_SETTLED:
					bool is_settled;
					isSettled(is_settled);
					res.is_settled = is_settled;
					break;

				case rx60controller::command::Request::SET_MAX_SPEED:
					setMaxSpeedPercentage(req.velocity_procentage);
					break;

				case rx60controller::command::Request::SET_MAX_ACCELERATION:
					setMaxAccelerationPercentage(req.acceleration_procentage);
					break;

				case rx60controller::command::Request::SET_MAX_DEACCELERATION:
					setMaxDecelerationPercentage(req.deceleration_procentage);
					break;

				case rx60controller::command::Request::SET_VALVE1:
					setDigitalOutput(eDO_VALVE1,req.output_state);
					break;

				case rx60controller::command::Request::SET_VALVE2:
					setDigitalOutput(eDO_VALVE2,req.output_state);
					break;

				case rx60controller::command::Request::RESET_MOTION:
					resetMotion();
					break;

				case rx60controller::command::Request::SET_JOINT_CONFIGURATIONS:
					_positionVector = req.positions;
					res.status = true;	
					std::cout <<"position vector size " << _positionVector.size() << std::endl;
					break;
				case rx60controller::command::Request::SET_TOOL_FLANGE_CARTESIAN_POSITIONS:
					res.status = true;	
					_positionVector = req.positions;
					break;

				case rx60controller::command::Request::SET_MOVE_JOINTS:
					res.status = true;	
					moveJoints();
					break;
				
				case rx60controller::command::Request::SET_MOVE_POSITIONS:
					res.status = true;	
					movePositions();
					break;
				
				default:
					ROS_ERROR("RX60 controller service called with unknown command number");
					break;
		}
		return true;
	}
}

void RX60Driver::moveJoints()
{
		
	double joints[6];
	if(_positionVector.size()/6 < 1)
	{
		ROS_ERROR("Error in size of position vector");
		return;	
	}
	for(int i = 0; i < _positionVector.size(); i = i + 6){	
		ROS_INFO("%lf %lf %lf %lf %lf %lf", _positionVector[i],_positionVector[i+1],
				_positionVector[i+2],_positionVector[i+3],_positionVector[i+4],_positionVector[i+5]);
		joints[0] =_positionVector[i];
		joints[1] =_positionVector[i+1];
		joints[2] =_positionVector[i+2];
		joints[3] =_positionVector[i+3];
		joints[4] =_positionVector[i+4];
		joints[5] =_positionVector[i+5];
		move(joints, RX60Driver::eCT_JOINT_ANGLES, false, RX60Driver::eIT_JOINT_SPACE);
		//ROS_INFO("Next point ready");
	}
//send _positionVector to device
//send _positionVector to device
//bool is_settled;
//isSettled(is_settled);
}

void RX60Driver::movePositions()
{
/*
	double position[6];
	position[0] = req.x;
	position[1] = req.y;
	position[2] = req.z;
	position[3] = req.theta_x;
	position[4] = req.theta_y;
	position[5] = req.theta_z;
	move(position, RX60Driver::eCT_CARTESIAN_COORDINATES, false, RX60Driver::eIT_CARTESIAN_SPACE);
*/
}

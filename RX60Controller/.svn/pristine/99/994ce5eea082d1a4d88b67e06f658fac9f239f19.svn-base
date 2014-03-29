//
// C++ Implementation: test
//
// Description: 
//
//
// Author: kraft,,, <kraft@tek-4714>, (C) 2008
//
//
//

#include "RX60Driver.hpp"
#include <iostream>

int main() {

	RX60Driver * robot;

	printf("Press Key\n");
	getchar();

	robot = new RX60Driver("172.16.1.25","22222");

	if (!robot->isConnected()) {
		std::cout << "Could not connect to robot!" << std::endl;
		abort();
	}
	else
	{
		std::cout << "Connection established" << std::endl;
	}

	double a[6];

	for (int j = -10; j <= 10; j++) {
		printf("Press Key\n");
		getchar();

		for (int i = 0; i < 6; i++) {
			a[i] = j;
		}

		robot->setMaxSpeedPercentage(50 + 5 * abs(j));

		printf("Press Key\n");
		getchar();

		robot->move(a, RX60Driver::eCT_JOINT_ANGLES, false,
				RX60Driver::eIT_JOINT_SPACE);

		bool isEmpty, isSettled, isEmpty2, isSettled2;

		robot->movementQueueEmpty(isEmpty);
		robot->isSettled(isSettled);

		printf("Press Key\n");
		getchar();

		robot->resetMotion();

		robot->movementQueueEmpty(isEmpty2);
		robot->isSettled(isSettled2);

		std::cout << isEmpty << isSettled << isEmpty2 << isSettled2
				<< std::endl;

		printf("Press Key\n");
		getchar();

		robot->getPosition(a, RX60Driver::eCT_CARTESIAN_COORDINATES);

		std::cout << "POS: ";

		for (int i = 0; i < 6; i++) {
			std::cout << a[i] << " ";
		}

		std::cout << std::endl;

		double velocityPercentage;
		double accelerationPercentage;
		double decelerationPercentage;
		double maxRotationalVelocity;
		double maxCartesianVelocity;
		bool blend;
		double reach;
		double leave;

		printf("Press Key\n");
		getchar();

		robot->getMovementParameters(velocityPercentage, accelerationPercentage,
				decelerationPercentage, maxRotationalVelocity,
				maxCartesianVelocity, blend, reach, leave);

		std::cout << "PARAMS: " << velocityPercentage << " "
				<< accelerationPercentage << " " << decelerationPercentage
				<< " " << maxRotationalVelocity << " " << maxCartesianVelocity
				<< " " << blend << " " << reach << " " << leave << std::endl;
	}

	printf("Press Key\n");
	getchar();

	for (int i = 0; i < 6; i++) {
		a[i] = 0;
	}

	robot->move(a, RX60Driver::eCT_JOINT_ANGLES, false,
			RX60Driver::eIT_JOINT_SPACE);

	printf("Press Key\n");
	getchar();

	delete robot;

	printf("Press Key\n");
	getchar();

}

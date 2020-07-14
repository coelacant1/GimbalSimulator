#include "Vector.h"
#include "Rotation.h"
#include <iostream>

EulerOrder eO = EulerConstants::EulerOrderXYXR;

Quaternion currentRotation = Quaternion(1, 0, 0, 0);

Vector3D currentVectorRotation = Vector3D(0,   0,   0);
Vector3D startVectorRotation   = Vector3D(0,   0,   0);
Vector3D endVectorRotation     = Vector3D(360, 360, 180);

Vector3D currentPosition = Vector3D(0, 0, 0);
Vector3D startPosition = Vector3D(0, 0, 0);
Vector3D endPosition = Vector3D(500, 0, 0);

Vector3D currentLinearVelocity = Vector3D(1, 0, 0);
Vector3D startLinearVelocity   = Vector3D(1, 0, 0);
Vector3D targetLinearVelocity  = Vector3D(50, 0, 0);
Vector3D endLinearVelocity     = Vector3D(1, 0, 0);

Vector3D currentAngularVelocity = Vector3D(0, 0, 0);

Vector3D mpu1Offset = Vector3D( 10,  30, 10);
Vector3D mpu2Offset = Vector3D( 10, -30, 10);
Vector3D mpu3Offset = Vector3D(-10,  30, 10);
Vector3D mpu4Offset = Vector3D(-10, -30, 10);

Vector3D mpu1GlobalPosition;
Vector3D mpu2GlobalPosition;
Vector3D mpu3GlobalPosition;
Vector3D mpu4GlobalPosition;

Vector3D mpu1LocalAcceleration;//global accleration + coriolis + centrifugal + euler

Vector3D mpu1LocalAngularVelocity;

//double time = 10.0;
double rampUpRatio = 0.1;
double rampDownRatio = 0.9;
double currentTime = 0.0;
double timeIncrement = 0.001;
double globalRatio = 0.0;

void main() {
	//RAMP UP
	for (double i = 0.0; i < endPosition.X * rampUpRatio; i += currentLinearVelocity.X * timeIncrement) {
		currentPosition.X = i;
		double ratio = (endPosition.X * rampUpRatio - currentPosition.X) / (endPosition.X * rampUpRatio);
		globalRatio = currentPosition.X / endPosition.X;

		currentRotation = Rotation(EulerAngles(Vector3D::LERP(startVectorRotation, endVectorRotation, globalRatio), eO)).GetQuaternion();

		currentLinearVelocity = Vector3D::LERP(startLinearVelocity, targetLinearVelocity, 1.0 - ratio);// (1 - 50) / 50

		std::cout << Mathematics::DoubleToCleanString(globalRatio) << " " << currentLinearVelocity.ToString() << " " << currentPosition.ToString() << " " << currentRotation.ToString() << std::endl;
	}

	//CONSTANT
	for (double i = endPosition.X * rampUpRatio; i < endPosition.X * rampDownRatio; i += currentLinearVelocity.X * timeIncrement) {
		currentPosition.X = i;
		globalRatio = currentPosition.X / endPosition.X;

		currentRotation = Rotation(EulerAngles(Vector3D::LERP(startVectorRotation, endVectorRotation, globalRatio), eO)).GetQuaternion();

		std::cout << Mathematics::DoubleToCleanString(globalRatio) << " " << currentLinearVelocity.ToString() << " " << currentPosition.ToString() << " " << currentRotation.ToString() << std::endl;
	}

	Vector3D rampDownVel = currentLinearVelocity;
	Vector3D rampDownPosition = currentPosition;

	//RAMP DOWN
	for (double i = rampDownPosition.X; i < endPosition.X; i += currentLinearVelocity.X * timeIncrement) {
		currentPosition.X = i;
		double ratio = (currentPosition.X - rampDownPosition.X) / (endPosition.X - rampDownPosition.X);// 450 -> 500 (450 - 450 + ) / (500 - 450) = 0 + currentPosition / 50
		globalRatio = currentPosition.X / endPosition.X;

		currentRotation = Rotation(EulerAngles(Vector3D::LERP(startVectorRotation, endVectorRotation, globalRatio), eO)).GetQuaternion();

		currentLinearVelocity = Vector3D::LERP(rampDownVel, endLinearVelocity, ratio);// (1 - 50) / 50
		
		std::cout << Mathematics::DoubleToCleanString(globalRatio) << " " << currentLinearVelocity.ToString() << " " << currentPosition.ToString() << " " << currentRotation.ToString() << std::endl;
	}

}

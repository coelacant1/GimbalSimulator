#include "Vector.h"
#include "Rotation.h"
#include <iostream>
#include <random>
#include "VectorKalmanFilter.h"

EulerOrder eO = EulerConstants::EulerOrderXYXR;

Quaternion currentRotation = Quaternion(1, 0, 0, 0);

Vector3D currentVectorRotation = Vector3D(0,   0,   0);
Vector3D startVectorRotation   = Vector3D(0,   0,   0);
Vector3D endVectorRotation     = Vector3D(720,  90, 90);

Vector3D currentPosition = Vector3D(0, 0, 0);
Vector3D startPosition = Vector3D(0, 0, 0);
Vector3D endPosition = Vector3D(0.5, 0, 0);

Vector3D currentLinearVelocity  = Vector3D(0.001, 0, 0);
Vector3D previousLinearVelocity = Vector3D(0.001, 0, 0);
Vector3D startLinearVelocity    = Vector3D(0.001, 0, 0);
Vector3D targetLinearVelocity   = Vector3D(1.5,  0, 0);
Vector3D endLinearVelocity      = Vector3D(0.001, 0, 0);

Vector3D currentAngularVelocity  = Vector3D(0, 0, 0);
Vector3D previousAngularVelocity = Vector3D(0, 0, 0);

Vector3D mpu1Offset = Vector3D(-0.012, 0.015,  0.030);
Vector3D mpu2Offset = Vector3D(-0.012, 0.015,  0.010);
Vector3D mpu3Offset = Vector3D(-0.012, 0.015, -0.010);
Vector3D mpu4Offset = Vector3D(-0.012, 0.015, -0.030);

Vector3D mpu1GlobalPosition;
Vector3D mpu2GlobalPosition;
Vector3D mpu3GlobalPosition;
Vector3D mpu4GlobalPosition;

//double time = 10.0;
double rampUpRatio = 0.1;
double rampDownRatio = 0.9;
double currentTime = 0.0;
double timeIncrement = 0.001;

Quaternion previousRotation          = Quaternion(1, 0, 0, 0);
AxisAngle currentAxisAngle           = AxisAngle(0, Vector3D(0, 0, 0));
Vector3D currentAngularAcceleration  = Vector3D(0, 0, 0);
Vector3D rotatedAngularVelocity      = Vector3D(0, 0, 0);
Vector3D currentLinearAcceleration   = Vector3D(0, 0, 0);
Vector3D currentRotatedAcceleration  = Vector3D(0, 0, 0);
Vector3D mpu1CentrifugalAcceleration = Vector3D(0, 0, 0);
Vector3D mpu2CentrifugalAcceleration = Vector3D(0, 0, 0);
Vector3D mpu3CentrifugalAcceleration = Vector3D(0, 0, 0);
Vector3D mpu4CentrifugalAcceleration = Vector3D(0, 0, 0);
Vector3D mpu1CoriolisAcceleration    = Vector3D(0, 0, 0);
Vector3D mpu2CoriolisAcceleration    = Vector3D(0, 0, 0);
Vector3D mpu3CoriolisAcceleration    = Vector3D(0, 0, 0);
Vector3D mpu4CoriolisAcceleration    = Vector3D(0, 0, 0);
Vector3D mpu1EulerAcceleration       = Vector3D(0, 0, 0);
Vector3D mpu2EulerAcceleration       = Vector3D(0, 0, 0);
Vector3D mpu3EulerAcceleration       = Vector3D(0, 0, 0);
Vector3D mpu4EulerAcceleration		 = Vector3D(0, 0, 0);

Vector3D mpu1LocalAcceleration = Vector3D(0, 0, 0);
Vector3D mpu2LocalAcceleration = Vector3D(0, 0, 0);
Vector3D mpu3LocalAcceleration = Vector3D(0, 0, 0);
Vector3D mpu4LocalAcceleration = Vector3D(0, 0, 0);

Vector3D mpu1LinearVelocity = Vector3D(0, 0, 0);
Vector3D mpu2LinearVelocity = Vector3D(0, 0, 0);
Vector3D mpu3LinearVelocity = Vector3D(0, 0, 0);
Vector3D mpu4LinearVelocity = Vector3D(0, 0, 0);

KalmanFilter kfPos = KalmanFilter(0.001, 1000);
VectorKalmanFilter vkfAccel = VectorKalmanFilter(0.001, 1000);
VectorKalmanFilter vkfAVel = VectorKalmanFilter(0.001, 1000);

double mpuSuspendedMass = 0.01;// 0.000000001;//kg -> ~1 microgram

std::default_random_engine generator(2);

void Calculate(double ratio);

double CosineInterpolate(double y1, double y2, double mu) {
	double mu2 = (1.0 - cos(mu * 3.14159)) / 2.0;

	return(y1 * (1.0 - mu2) + y2 * mu2);
}


double getRand(double range) {
	std::normal_distribution<double> distribution(-range / 2.0, range / 2.0);

	return distribution(generator);
}

Vector3D randomize(double scale) {
	double rand1 = scale * getRand(0.05);
	double rand2 = scale * getRand(0.05);
	double rand3 = scale * getRand(0.05);

	rand1 = getRand(1) > 0.45 ? rand1 + getRand(scale / 4.0) : rand1;
	rand2 = getRand(1) > 0.45 ? rand2 + getRand(scale / 4.0) : rand2;
	rand3 = getRand(1) > 0.45 ? rand3 + getRand(scale / 4.0) : rand3;

	return Vector3D(rand1, rand2, rand3);
}

Vector3D offset1 = randomize(2.0);
Vector3D offset2 = randomize(20.0);
Vector3D scalar1 = Vector3D(1, 1, 1).Add(randomize(0.05));
Vector3D scalar2 = Vector3D(1, 1, 1).Add(randomize(0.5));

void main() {
	srand(1);

	for (double i = 0.0; i < 1.0; i += timeIncrement) {
		if (i < 0.1 || i > 0.9)
			currentLinearVelocity.X = CosineInterpolate(0.0, 0.05, i * 10.0);
		else
			currentLinearVelocity.X = 0.05;
		
		currentPosition.X = currentPosition.X + currentLinearVelocity.X * timeIncrement;

		Calculate(i);
	}
}

void Calculate(double ratio) {
	currentRotation = Rotation(EulerAngles(Vector3D::LERP(startVectorRotation, endVectorRotation, ratio), eO)).GetQuaternion();
	currentAxisAngle = Rotation(currentRotation * previousRotation.Conjugate()).GetAxisAngle();
	//currentAngularVelocity = currentAxisAngle.Axis * currentAxisAngle.Rotation / timeIncrement;
	currentAngularVelocity = ((currentRotation - previousRotation) * previousRotation.Conjugate() / timeIncrement).GetBiVector() * 2.0;
	rotatedAngularVelocity = currentRotation.RotateVector(currentAngularVelocity);
	currentAngularAcceleration = (currentAngularVelocity - previousAngularVelocity) / timeIncrement;

	currentLinearAcceleration = ((currentLinearVelocity - previousLinearVelocity) / timeIncrement) + Vector3D(0, -9.81, 0);//inertial frame
	currentRotatedAcceleration = currentRotation.RotateVector(currentLinearAcceleration);//rotating frame of reference

	mpu1LinearVelocity = currentLinearVelocity + currentAngularVelocity.CrossProduct(mpu1Offset);
	mpu2LinearVelocity = currentLinearVelocity + currentAngularVelocity.CrossProduct(mpu2Offset);
	mpu3LinearVelocity = currentLinearVelocity + currentAngularVelocity.CrossProduct(mpu3Offset);
	mpu4LinearVelocity = currentLinearVelocity + currentAngularVelocity.CrossProduct(mpu4Offset);
	
	mpu1CentrifugalAcceleration = rotatedAngularVelocity.CrossProduct(rotatedAngularVelocity.CrossProduct(currentRotation.RotateVector(mpu1Offset))) * (-1.0 * mpuSuspendedMass);
	mpu2CentrifugalAcceleration = rotatedAngularVelocity.CrossProduct(rotatedAngularVelocity.CrossProduct(currentRotation.RotateVector(mpu2Offset))) * (-1.0 * mpuSuspendedMass);
	mpu3CentrifugalAcceleration = rotatedAngularVelocity.CrossProduct(rotatedAngularVelocity.CrossProduct(currentRotation.RotateVector(mpu3Offset))) * (-1.0 * mpuSuspendedMass);
	mpu4CentrifugalAcceleration = rotatedAngularVelocity.CrossProduct(rotatedAngularVelocity.CrossProduct(currentRotation.RotateVector(mpu4Offset))) * (-1.0 * mpuSuspendedMass);

	mpu1CoriolisAcceleration = (rotatedAngularVelocity * (-2.0 * mpuSuspendedMass)).CrossProduct(mpu1LinearVelocity);
	mpu2CoriolisAcceleration = (rotatedAngularVelocity * (-2.0 * mpuSuspendedMass)).CrossProduct(mpu2LinearVelocity);
	mpu3CoriolisAcceleration = (rotatedAngularVelocity * (-2.0 * mpuSuspendedMass)).CrossProduct(mpu3LinearVelocity);
	mpu4CoriolisAcceleration = (rotatedAngularVelocity * (-2.0 * mpuSuspendedMass)).CrossProduct(mpu4LinearVelocity);

	mpu1EulerAcceleration = currentAngularAcceleration.CrossProduct(currentRotation.RotateVector(mpu1Offset)) * (-1.0 * mpuSuspendedMass);
	mpu2EulerAcceleration = currentAngularAcceleration.CrossProduct(currentRotation.RotateVector(mpu2Offset)) * (-1.0 * mpuSuspendedMass);
	mpu3EulerAcceleration = currentAngularAcceleration.CrossProduct(currentRotation.RotateVector(mpu3Offset)) * (-1.0 * mpuSuspendedMass);
	mpu4EulerAcceleration = currentAngularAcceleration.CrossProduct(currentRotation.RotateVector(mpu4Offset)) * (-1.0 * mpuSuspendedMass);

	mpu1LocalAcceleration = currentRotatedAcceleration + mpu1CentrifugalAcceleration + mpu1CoriolisAcceleration + mpu1EulerAcceleration;
	mpu2LocalAcceleration = currentRotatedAcceleration + mpu2CentrifugalAcceleration + mpu2CoriolisAcceleration + mpu2EulerAcceleration;
	mpu3LocalAcceleration = currentRotatedAcceleration + mpu3CentrifugalAcceleration + mpu3CoriolisAcceleration + mpu3EulerAcceleration;
	mpu4LocalAcceleration = currentRotatedAcceleration + mpu4CentrifugalAcceleration + mpu4CoriolisAcceleration + mpu4EulerAcceleration;

	//std::cout << Mathematics::DoubleToCleanString(globalRatio) << " " << currentLinearVelocity.ToString() << " " << currentPosition.ToString() << " " << currentRotation.ToString() << " " << currentLinearAcceleration.ToString() << std::endl;
	std::cout << Mathematics::DoubleToCleanString(ratio) << ", " << mpu1LocalAcceleration.ToString() << " " << mpu1LocalAcceleration.Add(randomize(2.0)).Add(offset1).Multiply(scalar1).ToString() << rotatedAngularVelocity.ToString() << " " << rotatedAngularVelocity.Add(randomize(5)).Add(offset1).Multiply(scalar1).ToString() << " " << std::endl;

	previousRotation = currentRotation;
	previousLinearVelocity = currentLinearVelocity;
	previousAngularVelocity = currentAngularVelocity;
}
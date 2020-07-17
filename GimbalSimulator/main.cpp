#include "Vector.h"
#include "Rotation.h"
#include <iostream>
#include <random>
#include "VectorKalmanFilter.h"

#include "MPU.h"
#include "MPUNoisifier.h"

EulerOrder eO = EulerConstants::EulerOrderXYXR;

Noise noise = Noise(1);

double gVKF = 0.02;
double mGKF = 100;

MPUNoisifier mpu1Noise = MPUNoisifier(noise);
MPUNoisifier mpu2Noise = MPUNoisifier(noise);
MPUNoisifier mpu3Noise = MPUNoisifier(noise);
MPUNoisifier mpu4Noise = MPUNoisifier(noise);

MPU mpu1 = MPU(Vector3D(-0.012, 0.015,  0.030), MPUOS(), gVKF, mGKF);
MPU mpu2 = MPU(Vector3D(-0.012, 0.015,  0.010), MPUOS(), gVKF, mGKF);
MPU mpu3 = MPU(Vector3D(-0.012, 0.015, -0.010), MPUOS(), gVKF, mGKF);
MPU mpu4 = MPU(Vector3D(-0.012, 0.015, -0.030), MPUOS(), gVKF, mGKF);

MPU mpu1N = MPU(Vector3D(-0.012, 0.015,  0.030), MPUOS(Vector3D(0.09, -0.09, -0.09), Vector3D(), Vector3D(10,10,10), Vector3D()), gVKF, mGKF);
MPU mpu2N = MPU(Vector3D(-0.012, 0.015,  0.030), mpu2Noise.GetMPUOS(), gVKF, mGKF);
MPU mpu3N = MPU(Vector3D(-0.012, 0.015, -0.030), mpu3Noise.GetMPUOS(), gVKF, mGKF);
MPU mpu4N = MPU(Vector3D(-0.012, 0.015, -0.030), mpu4Noise.GetMPUOS(), gVKF, mGKF);

Quaternion currentRotation = Quaternion(1, 0, 0, 0);

Vector3D currentVectorRotation = Vector3D(0,   0,   0);
Vector3D startVectorRotation   = Vector3D(0,   0,   0);
Vector3D endVectorRotation     = Vector3D(789,  -1607,  536);

Vector3D currentPosition = Vector3D(0, 0, 0);
Vector3D startPosition = Vector3D(0, 0, 0);
Vector3D endPosition = Vector3D(0.5, 0, 0);

Vector3D currentLinearVelocity  = Vector3D(0.0, 0, 0);
Vector3D previousLinearVelocity = Vector3D(0.0, 0, 0);
Vector3D startLinearVelocity    = Vector3D(0.0, 0, 0);
Vector3D targetLinearVelocity   = Vector3D(0.05,  0, 0);
Vector3D endLinearVelocity      = Vector3D(0.0, 0, 0);

Vector3D currentAngularVelocity  = Vector3D(0, 0, 0);
Vector3D previousAngularVelocity = Vector3D(0, 0, 0);

double timeIncrement = 0.001;

Quaternion previousRotation          = Quaternion(1, 0, 0, 0);
Vector3D currentAngularAcceleration  = Vector3D(0, 0, 0);
Vector3D rotatedAngularVelocity      = Vector3D(0, 0, 0);
Vector3D currentLinearAcceleration   = Vector3D(0, 0, 0);
Vector3D currentRotatedAcceleration  = Vector3D(0, 0, 0);

void Calculate(double globalRatio, double motionRatio, bool print);

void main() {

	for (double i = 0.0; i < 1.0; i += timeIncrement) {
		if (i < 0.1 || i > 0.9)
			currentLinearVelocity.X = Mathematics::CosineInterpolate(0.0, targetLinearVelocity.X, i * 10.0);
		else
			currentLinearVelocity.X = targetLinearVelocity.X;
		
		currentPosition.X = currentPosition.X + currentLinearVelocity.X * timeIncrement;

		Calculate(i, currentPosition.X / endPosition.X, true);
	}

	mpu1.CalculateAverage();
	mpu2.CalculateAverage();
	mpu3.CalculateAverage();
	mpu4.CalculateAverage();
	mpu1N.CalculateAverage();
	mpu2N.CalculateAverage();
	mpu3N.CalculateAverage();
	mpu4N.CalculateAverage();

	Vector3D accScalar1 = mpu1.GetAccScalar(mpu1N.GetAccRange());
	Vector3D accScalar2 = mpu2.GetAccScalar(mpu2N.GetAccRange());
	Vector3D accScalar3 = mpu3.GetAccScalar(mpu3N.GetAccRange());
	Vector3D accScalar4 = mpu4.GetAccScalar(mpu4N.GetAccRange());

	Vector3D accOffset1 = mpu1.GetAccOffset(accScalar1, mpu1N.GetAccAvg(), mpu1.GetAccAvg());
	Vector3D accOffset2 = mpu2.GetAccOffset(accScalar2, mpu2N.GetAccAvg(), mpu2.GetAccAvg());
	Vector3D accOffset3 = mpu3.GetAccOffset(accScalar3, mpu3N.GetAccAvg(), mpu3.GetAccAvg());
	Vector3D accOffset4 = mpu4.GetAccOffset(accScalar4, mpu4N.GetAccAvg(), mpu4.GetAccAvg());

	Vector3D accScalarDif1 = mpu1Noise.GetMPUOS().accScalar - accScalar1;
	Vector3D accScalarDif2 = mpu2Noise.GetMPUOS().accScalar - accScalar2;
	Vector3D accScalarDif3 = mpu3Noise.GetMPUOS().accScalar - accScalar3;
	Vector3D accScalarDif4 = mpu4Noise.GetMPUOS().accScalar - accScalar4;

	Vector3D accOffsetDif1 = mpu1Noise.GetMPUOS().accOffset - accOffset1;
	Vector3D accOffsetDif2 = mpu2Noise.GetMPUOS().accOffset - accOffset2;
	Vector3D accOffsetDif3 = mpu3Noise.GetMPUOS().accOffset - accOffset3;
	Vector3D accOffsetDif4 = mpu4Noise.GetMPUOS().accOffset - accOffset4;

	std::cout << std::endl;

	std::cout << accScalar1.ToString() << " " << accOffset1.ToString() << std::endl;

	std::cout << accScalar2.ToString() << " " << mpu2Noise.GetMPUOS().accScalar.ToString() << std::endl;

	std::cout << std::endl;

	std::cout << accOffset2.ToString() << " " << mpu2Noise.GetMPUOS().accOffset.ToString() << std::endl;

	std::cout << std::endl;

	std::cout << accScalarDif1.Multiply(100).ToString() << " " << accOffsetDif1.Multiply(100).ToString() << std::endl;
	std::cout << accScalarDif2.Multiply(100).ToString() << " " << accOffsetDif2.Multiply(100).ToString() << std::endl;
	std::cout << accScalarDif3.Multiply(100).ToString() << " " << accOffsetDif3.Multiply(100).ToString() << std::endl;
	std::cout << accScalarDif4.Multiply(100).ToString() << " " << accOffsetDif4.Multiply(100).ToString() << std::endl;

	std::cout << std::endl;
}

void Calculate(double globalRatio, double motionRatio, bool print) {
	currentRotation = Rotation(EulerAngles(Vector3D::LERP(startVectorRotation, endVectorRotation, motionRatio), eO)).GetQuaternion();
	currentAngularVelocity = ((currentRotation - previousRotation) * previousRotation.Conjugate() / timeIncrement).GetBiVector() * 2.0;
	rotatedAngularVelocity = currentRotation.RotateVector(currentAngularVelocity);
	currentAngularAcceleration = (currentAngularVelocity - previousAngularVelocity) / timeIncrement;

	currentLinearAcceleration = ((currentLinearVelocity - previousLinearVelocity) / timeIncrement) + Vector3D(0, -9.81, 0);//inertial frame
	currentRotatedAcceleration = currentRotation.RotateVector(currentLinearAcceleration);//rotating frame of reference

	Vector3D mpu1LAcc = mpu1.GetLocalAcceleration(currentLinearVelocity, currentLinearAcceleration, currentAngularVelocity, currentAngularAcceleration, currentRotation);
	Vector3D mpu2LAcc = mpu2.GetLocalAcceleration(currentLinearVelocity, currentLinearAcceleration, currentAngularVelocity, currentAngularAcceleration, currentRotation);
	Vector3D mpu3LAcc = mpu3.GetLocalAcceleration(currentLinearVelocity, currentLinearAcceleration, currentAngularVelocity, currentAngularAcceleration, currentRotation);
	Vector3D mpu4LAcc = mpu4.GetLocalAcceleration(currentLinearVelocity, currentLinearAcceleration, currentAngularVelocity, currentAngularAcceleration, currentRotation);

	Vector3D mpu1LVel = currentAngularVelocity;
	Vector3D mpu2LVel = currentAngularVelocity;
	Vector3D mpu3LVel = currentAngularVelocity;
	Vector3D mpu4LVel = currentAngularVelocity;

	Vector3D mpu1LAccN = mpu1LAcc;//.Add(noise.RandomizeNormal(2.0));
	Vector3D mpu2LAccN = mpu2LAcc;//.Add(noise.RandomizeNormal(2.0));
	Vector3D mpu3LAccN = mpu3LAcc;//.Add(noise.RandomizeNormal(2.0));
	Vector3D mpu4LAccN = mpu4LAcc;//.Add(noise.RandomizeNormal(2.0));

	Vector3D mpu1LVelN = currentAngularVelocity;// .Add(noise.RandomizeNormal(1.0));
	Vector3D mpu2LVelN = currentAngularVelocity;//.Add(noise.RandomizeNormal(1.0));
	Vector3D mpu3LVelN = currentAngularVelocity;// .Add(noise.RandomizeNormal(1.0));
	Vector3D mpu4LVelN = currentAngularVelocity;// .Add(noise.RandomizeNormal(1.0));

	if (print) {
		mpu1.UpdateCalibration(mpu1LAcc, mpu1LVel);
		mpu2.UpdateCalibration(mpu2LAcc, mpu1LVel);
		mpu3.UpdateCalibration(mpu3LAcc, mpu1LVel);
		mpu4.UpdateCalibration(mpu4LAcc, mpu1LVel);

		mpu1N.UpdateCalibration(mpu1LAccN, mpu1LVelN);
		mpu2N.UpdateCalibration(mpu2LAccN, mpu2LVelN);
		mpu3N.UpdateCalibration(mpu3LAccN, mpu3LVelN);
		mpu4N.UpdateCalibration(mpu4LAccN, mpu4LVelN);

		std::cout << Mathematics::DoubleToCleanString(globalRatio) << "," << mpu1LAcc.ToString() << mpu1LAccN.ToString() << currentAngularVelocity.ToString() << std::endl;
	}

	previousRotation = currentRotation;
	previousLinearVelocity = currentLinearVelocity;
	previousAngularVelocity = currentAngularVelocity;
}

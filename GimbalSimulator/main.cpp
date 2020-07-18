#include "Vector.h"
#include "Rotation.h"
#include <iostream>
#include <random>
#include "VectorKalmanFilter.h"
#include <vector>

#include "MPU.h"
#include "MPUCalibrator.h"
#include "MPUNoisifier.h"

//MPUNOISIFIER

EulerOrder eO = EulerConstants::EulerOrderZYZR;

Noise noise = Noise(1);

double gVKF = 0.05;
double mGKF = 200;

MPUNoisifier mpu1Noise = MPUNoisifier(noise);
MPUNoisifier mpu2Noise = MPUNoisifier(noise);
MPUNoisifier mpu3Noise = MPUNoisifier(noise);
MPUNoisifier mpu4Noise = MPUNoisifier(noise);

MPU mpu1 = MPU(Vector3D(-0.012, 0.015,  0.030), MPUOS(), gVKF, mGKF);
MPU mpu2 = MPU(Vector3D(-0.012, 0.015,  0.010), MPUOS(), gVKF, mGKF);
MPU mpu3 = MPU(Vector3D(-0.012, 0.015, -0.010), MPUOS(), gVKF, mGKF);
MPU mpu4 = MPU(Vector3D(-0.012, 0.015, -0.030), MPUOS(), gVKF, mGKF);

MPU mpu1N = MPU(Vector3D(-0.012, 0.015,  0.030), mpu1Noise.GetMPUOS(), gVKF, mGKF);
MPU mpu2N = MPU(Vector3D(-0.012, 0.015,  0.030), mpu2Noise.GetMPUOS(), gVKF, mGKF);
MPU mpu3N = MPU(Vector3D(-0.012, 0.015, -0.030), mpu3Noise.GetMPUOS(), gVKF, mGKF);
MPU mpu4N = MPU(Vector3D(-0.012, 0.015, -0.030), mpu4Noise.GetMPUOS(), gVKF, mGKF);

Quaternion currentRotation = Quaternion(1, 0, 0, 0);

Vector3D currentVectorRotation = Vector3D(0,   0,   0);
Vector3D startVectorRotation   = Vector3D(0,   0,   0);
Vector3D endVectorRotation = Vector3D(360,  -180,  90);

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

	MPUOS mpu1C = MPUCalibrator(&mpu1, &mpu1N).Calibrate();
	MPUOS mpu2C = MPUCalibrator(&mpu2, &mpu2N).Calibrate();
	MPUOS mpu3C = MPUCalibrator(&mpu3, &mpu3N).Calibrate();
	MPUOS mpu4C = MPUCalibrator(&mpu4, &mpu4N).Calibrate();

	std::cout << std::endl << "Accelerometer Calibration: " << std::endl;

	std::cout << mpu1C.AccToString() << std::endl;
	std::cout << mpu2C.AccToString() << std::endl;
	std::cout << mpu3C.AccToString() << std::endl;
	std::cout << mpu4C.AccToString() << std::endl;

	std::cout << std::endl << "Gyroscope Calibration: " << std::endl;
	std::cout << mpu1C.VelToString() << std::endl;
	std::cout << mpu2C.VelToString() << std::endl;
	std::cout << mpu3C.VelToString() << std::endl;
	std::cout << mpu4C.VelToString() << std::endl;

	std::cout << std::endl << "Calibration Verification: " << std::endl;

	std::cout << MPUOS::Verify(mpu1C, mpu1Noise.GetMPUOS()).AccToString() << std::endl;
	std::cout << MPUOS::Verify(mpu2C, mpu2Noise.GetMPUOS()).AccToString() << std::endl;
	std::cout << MPUOS::Verify(mpu3C, mpu3Noise.GetMPUOS()).AccToString() << std::endl;
	std::cout << MPUOS::Verify(mpu4C, mpu4Noise.GetMPUOS()).AccToString() << std::endl;

}

void PrintVectors(std::vector<Vector3D> vectors) {
	for (Vector3D x : vectors) {
		std::cout << Mathematics::DoubleToCleanString(x.X) << ",";
	}

	for (Vector3D x : vectors) {
		std::cout << Mathematics::DoubleToCleanString(x.Y) << ",";
	}

	for (Vector3D x : vectors) {
		std::cout << Mathematics::DoubleToCleanString(x.Z) << ",";
	}

	std::cout << std::endl;
}

void Calculate(double globalRatio, double motionRatio, bool print) {
	currentRotation = Rotation(EulerAngles(Vector3D::LERP(startVectorRotation, endVectorRotation, motionRatio), eO)).GetQuaternion();
	currentAngularVelocity = ((currentRotation - previousRotation) * previousRotation.Conjugate() / timeIncrement).GetBiVector() * 2.0;
	rotatedAngularVelocity = currentRotation.RotateVector(currentAngularVelocity);
	currentAngularAcceleration = (currentAngularVelocity - previousAngularVelocity) / timeIncrement;

	currentLinearAcceleration = ((currentLinearVelocity - previousLinearVelocity) / timeIncrement) + Vector3D(0, -9.81, 0);//inertial frame
	currentRotatedAcceleration = currentRotation.RotateVector(currentLinearAcceleration);//rotating frame of reference


	Vector3D mpu1LAcc = mpu1.GetLocalAcceleration(currentLinearVelocity, currentLinearAcceleration, currentAngularVelocity, currentAngularAcceleration, currentRotation); Vector3D mpu1LAccF = mpu1LAcc;
	Vector3D mpu2LAcc = mpu2.GetLocalAcceleration(currentLinearVelocity, currentLinearAcceleration, currentAngularVelocity, currentAngularAcceleration, currentRotation); Vector3D mpu2LAccF = mpu2LAcc;
	Vector3D mpu3LAcc = mpu3.GetLocalAcceleration(currentLinearVelocity, currentLinearAcceleration, currentAngularVelocity, currentAngularAcceleration, currentRotation); Vector3D mpu3LAccF = mpu3LAcc;
	Vector3D mpu4LAcc = mpu4.GetLocalAcceleration(currentLinearVelocity, currentLinearAcceleration, currentAngularVelocity, currentAngularAcceleration, currentRotation); Vector3D mpu4LAccF = mpu4LAcc;

	Vector3D mpu1LVel = mpu1.GetLocalAngularVelocity(currentAngularVelocity); Vector3D mpu1LVelF = mpu1LVel;
	Vector3D mpu2LVel = mpu2.GetLocalAngularVelocity(currentAngularVelocity); Vector3D mpu2LVelF = mpu2LVel;
	Vector3D mpu3LVel = mpu3.GetLocalAngularVelocity(currentAngularVelocity); Vector3D mpu3LVelF = mpu3LVel;
	Vector3D mpu4LVel = mpu4.GetLocalAngularVelocity(currentAngularVelocity); Vector3D mpu4LVelF = mpu4LVel;

	Vector3D mpu1LAccN = mpu1N.GetLocalAcceleration(currentLinearVelocity, currentLinearAcceleration, currentAngularVelocity, currentAngularAcceleration, currentRotation).Add(noise.RandomizeNormal(0.05)); Vector3D mpu1LAccNF = mpu1LAccN;//0.25 simulated, real
	Vector3D mpu2LAccN = mpu2N.GetLocalAcceleration(currentLinearVelocity, currentLinearAcceleration, currentAngularVelocity, currentAngularAcceleration, currentRotation).Add(noise.RandomizeNormal(0.05)); Vector3D mpu2LAccNF = mpu2LAccN;//0.25 simulated, real
	Vector3D mpu3LAccN = mpu3N.GetLocalAcceleration(currentLinearVelocity, currentLinearAcceleration, currentAngularVelocity, currentAngularAcceleration, currentRotation).Add(noise.RandomizeNormal(0.05)); Vector3D mpu3LAccNF = mpu3LAccN;//0.25 simulated, real
	Vector3D mpu4LAccN = mpu4N.GetLocalAcceleration(currentLinearVelocity, currentLinearAcceleration, currentAngularVelocity, currentAngularAcceleration, currentRotation).Add(noise.RandomizeNormal(0.05)); Vector3D mpu4LAccNF = mpu4LAccN;//0.25 simulated, real

	Vector3D mpu1LVelN = mpu1N.GetLocalAngularVelocity(currentAngularVelocity).Add(noise.RandomizeNormal(0.05)); Vector3D mpu1LVelNF = mpu1LVelN;
	Vector3D mpu2LVelN = mpu2N.GetLocalAngularVelocity(currentAngularVelocity).Add(noise.RandomizeNormal(0.05)); Vector3D mpu2LVelNF = mpu2LVelN;
	Vector3D mpu3LVelN = mpu3N.GetLocalAngularVelocity(currentAngularVelocity).Add(noise.RandomizeNormal(0.05)); Vector3D mpu3LVelNF = mpu3LVelN;
	Vector3D mpu4LVelN = mpu4N.GetLocalAngularVelocity(currentAngularVelocity).Add(noise.RandomizeNormal(0.05)); Vector3D mpu4LVelNF = mpu4LVelN;

	if (print) {
		mpu1.UpdateCalibration(mpu1LAccF, mpu1LVelF);
		mpu2.UpdateCalibration(mpu2LAccF, mpu1LVelF);
		mpu3.UpdateCalibration(mpu3LAccF, mpu1LVelF);
		mpu4.UpdateCalibration(mpu4LAccF, mpu1LVelF);

		mpu1N.UpdateCalibration(mpu1LAccNF, mpu1LVelNF);
		mpu2N.UpdateCalibration(mpu2LAccNF, mpu2LVelNF);
		mpu3N.UpdateCalibration(mpu3LAccNF, mpu3LVelNF);
		mpu4N.UpdateCalibration(mpu4LAccNF, mpu4LVelNF);

		std::vector<Vector3D> vals = { mpu1LAcc, mpu2LAcc, mpu3LAcc, mpu4LAcc, mpu1LAccN, mpu2LAccN, mpu3LAccN, mpu4LAccN, mpu1LAccF, mpu2LAccF, mpu3LAccF, mpu4LAccF, mpu1LAccNF, mpu2LAccNF, mpu3LAccNF, mpu4LAccNF };

		std::cout << Mathematics::DoubleToCleanString(globalRatio) << ",";

		PrintVectors(vals);
	}

	previousRotation = currentRotation;
	previousLinearVelocity = currentLinearVelocity;
	previousAngularVelocity = currentAngularVelocity;
}

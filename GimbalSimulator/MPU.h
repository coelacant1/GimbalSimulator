#pragma once

#include "VectorAverage.h"
#include "VectorKalmanFilter.h"
#include "Rotation.h"
#include "MPUOS.h"

class MPU {
private:
	Vector3D offset;
	Vector3D centrifugalAcceleration;
	Vector3D coriolisAcceleration;
	Vector3D eulerAcceleration;
	Vector3D localAcceleration;
	Vector3D linearVelocity;

	Vector3D accMax;
	Vector3D accMin;
	Vector3D velMax;
	Vector3D velMin;

	Vector3D accAvgValue;
	Vector3D velAvgValue;
	Vector3D accStdDevValue;
	Vector3D velStdDevValue;

	VectorAverage accAvg = VectorAverage();
	VectorAverage velAvg = VectorAverage();

	VectorKalmanFilter accKF = VectorKalmanFilter();
	VectorKalmanFilter velKF = VectorKalmanFilter();

	MPUOS mpuOS;

	double proofMass = 0.000001;

public:
	MPU(Vector3D offset, MPUOS mpuOS, double gVKF, double mGKF, double proofMass = 0.000001);

	Vector3D GetLocalAcceleration(Vector3D gimbalLinearVelocity, Vector3D gimbalLinearAcceleration, Vector3D gimbalAngularVelocity, Vector3D gimbalAngularAcceleration, Quaternion rotation);
	Vector3D GetLocalAngularVelocity(Vector3D localAngularVelocity);

	void UpdateCalibration(Vector3D &localAcceleration, Vector3D &localAngularVelocity);
	void Calculate();

	Vector3D GetAccAvg();
	Vector3D GetVelAvg();
	Vector3D GetAccStdDev();
	Vector3D GetVelStdDev();
	Vector3D GetAccRange();
	Vector3D GetVelRange();
};

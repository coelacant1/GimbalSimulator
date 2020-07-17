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

	VectorAverage accAvg = VectorAverage();
	VectorAverage velAvg = VectorAverage();

	VectorKalmanFilter accKF = VectorKalmanFilter();
	VectorKalmanFilter velKF = VectorKalmanFilter();

	MPUOS mpuOS;

	double proofMass = 0.000001;

public:
	MPU(Vector3D offset, MPUOS mpuOS, double gVKF, double mGKF, double proofMass = 0.000001);

	Vector3D GetLocalAcceleration(Vector3D gimbalLinearVelocity, Vector3D gimbalLinearAcceleration, Vector3D gimbalAngularVelocity, Vector3D gimbalAngularAcceleration, Quaternion rotation);

	void UpdateCalibration(Vector3D &localAcceleration, Vector3D &localAngularVelocity);
	void CalculateAverage();

	Vector3D GetAccAvg();
	Vector3D GetVelAvg();
	Vector3D GetAccRange();
	Vector3D GetVelRange();
	Vector3D GetAccScalar(Vector3D sourceRange);
	Vector3D GetVelScalar(Vector3D sourceRange);
	Vector3D GetAccOffset(Vector3D targetScale, Vector3D sourceAvg, Vector3D targetAvg);
	Vector3D GetVelOffset(Vector3D targetScale, Vector3D sourceAvg, Vector3D targetAvg);
};

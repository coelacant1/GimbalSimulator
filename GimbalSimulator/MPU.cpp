#include "MPU.h"

MPU::MPU(Vector3D offset, MPUOS mpuOS, double gVKF, double mGKF, double proofMass) {
	this->offset = offset;
	this->mpuOS = mpuOS;
	this->proofMass = proofMass;
	this->accKF = VectorKalmanFilter(gVKF, mGKF);
	this->velKF = VectorKalmanFilter(gVKF, mGKF);
}

Vector3D MPU::GetLocalAcceleration(Vector3D gimbalLinearVelocity, Vector3D gimbalLinearAcceleration, Vector3D gimbalAngularVelocity, Vector3D gimbalAngularAcceleration, Quaternion rotation) {
	Vector3D rotatedAngularVelocity = rotation.RotateVector(gimbalAngularVelocity);
	linearVelocity = gimbalLinearVelocity + gimbalAngularVelocity.CrossProduct(offset);

	centrifugalAcceleration = rotatedAngularVelocity.CrossProduct(rotatedAngularVelocity.CrossProduct(rotation.RotateVector(offset))) * (-1.0 * proofMass);

	coriolisAcceleration = (rotatedAngularVelocity * (-2.0 * proofMass)).CrossProduct(linearVelocity);

	eulerAcceleration = gimbalAngularAcceleration.CrossProduct(rotation.RotateVector(offset)) * (-1.0 * proofMass);

	Vector3D localAcceleration = rotation.RotateVector(gimbalLinearAcceleration) + centrifugalAcceleration + coriolisAcceleration + eulerAcceleration;

	return localAcceleration * mpuOS.accScalar + mpuOS.accOffset;
}

Vector3D MPU::GetLocalAngularVelocity(Vector3D localAngularVelocity) {
	return localAngularVelocity * mpuOS.velScalar + mpuOS.velOffset;
}

void MPU::UpdateCalibration(Vector3D &localAcceleration, Vector3D &localAngularVelocity) {
	localAcceleration = accKF.Filter(localAcceleration);
	localAngularVelocity = velKF.Filter(localAngularVelocity);

	accMax = Vector3D::Max(accMax, localAcceleration);
	accMin = Vector3D::Min(accMin, localAcceleration);
	velMax = Vector3D::Max(velMax, localAngularVelocity);
	velMin = Vector3D::Min(velMin, localAngularVelocity);

	accAvg.Add(localAcceleration);
	velAvg.Add(localAngularVelocity);
}

void MPU::Calculate() {
	accAvgValue = accAvg.CalculateAverage();
	velAvgValue = velAvg.CalculateAverage();
	accStdDevValue = accAvg.CalculateStdDev();
	velStdDevValue = velAvg.CalculateStdDev();
}

Vector3D MPU::GetAccStdDev() {
	return accStdDevValue;
}

Vector3D MPU::GetVelStdDev() {
	return velStdDevValue;
}

Vector3D MPU::GetAccAvg() {
	return accAvgValue;
}

Vector3D MPU::GetVelAvg() {
	return velAvgValue;
}

Vector3D MPU::GetAccRange() {
	return accMax - accMin;
}

Vector3D MPU::GetVelRange() {
	return velMax - velMin;
}

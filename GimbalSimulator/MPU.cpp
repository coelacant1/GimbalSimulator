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

	return rotation.RotateVector(gimbalLinearAcceleration) + centrifugalAcceleration + coriolisAcceleration + eulerAcceleration;
}

void MPU::UpdateCalibration(Vector3D &localAcceleration, Vector3D &localAngularVelocity) {
	localAcceleration = (localAcceleration + mpuOS.accOffset) * mpuOS.accScalar;
	localAngularVelocity = (localAngularVelocity + mpuOS.velOffset) * mpuOS.velScalar;

	localAcceleration = accKF.Filter(localAcceleration);
	localAngularVelocity = velKF.Filter(localAngularVelocity);

	accMax = Vector3D::Max(accMax, localAcceleration);
	accMin = Vector3D::Min(accMin, localAcceleration);
	velMax = Vector3D::Max(velMax, localAngularVelocity);
	velMin = Vector3D::Min(velMin, localAngularVelocity);

	accAvg.Add(localAcceleration);
	velAvg.Add(localAngularVelocity);
}

void MPU::CalculateAverage() {
	accAvgValue = accAvg.Calculate();
	velAvgValue = velAvg.Calculate();
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

Vector3D MPU::GetAccScalar(Vector3D sourceRange) {
	return sourceRange / (accMax - accMin);
}

Vector3D MPU::GetVelScalar(Vector3D sourceRange) {
	return sourceRange / (velMax - velMin);
}

Vector3D MPU::GetAccOffset(Vector3D targetScale, Vector3D sourceAvg, Vector3D targetAvg) {
	return (sourceAvg / targetScale) - targetAvg;
}

Vector3D MPU::GetVelOffset(Vector3D targetScale, Vector3D sourceAvg, Vector3D targetAvg) {
	return (sourceAvg / targetScale) - targetAvg;
}

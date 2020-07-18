#pragma once

#include "MPU.h"
#include "MPUOS.h"

class MPUCalibrator {
private:
	MPU* mpuTargetCalibration;
	MPU* mpuUncalibrated;
	MPUOS mpuCalibration;

	void CalibrateAccelerometer();
	void CalibrateGyroscope();

	void CalculateScalarOffset(Vector3D& scalar, Vector3D& offset, Vector3D sDS, Vector3D sDR, Vector3D aS, Vector3D aR);

public:
	MPUCalibrator(MPU* mpuTargetCalibration, MPU* mpuUncalibrated);

	MPUOS Calibrate();

};

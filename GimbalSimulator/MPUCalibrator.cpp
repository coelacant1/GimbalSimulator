#include "MPUCalibrator.h"

MPUCalibrator::MPUCalibrator(MPU* mpuTargetCalibration, MPU* mpuUncalibrated) {
	this->mpuTargetCalibration = mpuTargetCalibration;
	this->mpuUncalibrated = mpuUncalibrated;
}

MPUOS MPUCalibrator::Calibrate() {
	mpuTargetCalibration->Calculate();
	mpuUncalibrated->Calculate();

	CalibrateAccelerometer();
	CalibrateGyroscope();

	return MPUOS(mpuCalibration);
}

void MPUCalibrator::CalibrateAccelerometer() {
	Vector3D scalar, offset;
	Vector3D avgS = mpuTargetCalibration->GetAccAvg();
	Vector3D avgR = mpuUncalibrated->GetAccAvg();
	Vector3D stdDevS = mpuTargetCalibration->GetAccStdDev();
	Vector3D stdDevR = mpuUncalibrated->GetAccStdDev();

	CalculateScalarOffset(mpuCalibration.accScalar, mpuCalibration.accOffset, stdDevS, stdDevR, avgS, avgR);
}

void MPUCalibrator::CalibrateGyroscope() {
	Vector3D scalar, offset;
	Vector3D avgS = mpuTargetCalibration->GetVelAvg();
	Vector3D avgR = mpuUncalibrated->GetVelAvg();
	Vector3D stdDevS = mpuTargetCalibration->GetVelStdDev();
	Vector3D stdDevR = mpuUncalibrated->GetVelStdDev();

	CalculateScalarOffset(mpuCalibration.velScalar, mpuCalibration.velOffset, stdDevS, stdDevR, avgS, avgR);
}

/// <summary>
/// Calculates a scalar and offset value from the standard deviation and average of two separate signals
/// </summary>
/// <param name="scalar">Out scalar</param>
/// <param name="offset">Out offset</param>
/// <param name="sDS">StdDev of Simulated</param>
/// <param name="sDR">StdDev of Real</param>
/// <param name="aS">Average of Simulated</param>
/// <param name="aR">Average of Real</param>
void MPUCalibrator::CalculateScalarOffset(Vector3D& scalar, Vector3D& offset, Vector3D sDS, Vector3D sDR, Vector3D aS, Vector3D aR) {
	scalar = sDS / sDR;
	offset = ((aR * sDS * -1.0 + sDR * aS) / sDR) / scalar;

	//std::cout << aR.X << "," << sDS.X << "," << aS.X << "," << sDR.X << "," << (aR.X * sDS.X) / sDR.X * -1.0 + (sDR.X * aS.X) / sDR.X << std::endl;
}

//=((F1 - AF$6)/AN$4)*AJ$4+$AE$6
//(F1*AJ$4)/AN$4 - (AF$6*AJ$4)/AN$4 + (AN$4*$AE$6)/AN$4

//scalar = stdDev(simulation)/stdDev(real)
//offset = (avg(real)*stdDev(simulation) + stdDev(real)*avg(simulation))/stdDev(simulation)

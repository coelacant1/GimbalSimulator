#pragma once

#include "Noise.h"
#include "Vector.h"
#include "MPUOS.h"

class MPUNoisifier {
private:
	MPUOS mpuOS;

public:
	MPUNoisifier(Noise noise);

	MPUOS GetMPUOS(){
		return MPUOS(mpuOS);
	}
};

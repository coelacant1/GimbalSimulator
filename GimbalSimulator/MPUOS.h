#pragma once
#include "Vector.h"

typedef struct MPUOS {
	Vector3D accOffset = Vector3D(0, 0, 0);
	Vector3D velOffset = Vector3D(0, 0, 0);
	Vector3D accScalar = Vector3D(1, 1, 1);
	Vector3D velScalar = Vector3D(1, 1, 1);

	MPUOS() {}

	MPUOS(const MPUOS& mpuOS) {
		this->accOffset = mpuOS.accOffset;
		this->velOffset = mpuOS.velOffset;
		this->accScalar = mpuOS.accScalar;
		this->velScalar = mpuOS.velScalar;
	}

	MPUOS(Vector3D aO, Vector3D vO, Vector3D aS, Vector3D vS) : accOffset(aO), velOffset(vO), accScalar(aS), velScalar(vS) {}
} MPUOS;
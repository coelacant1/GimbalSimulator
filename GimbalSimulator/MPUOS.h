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

	std::string AccToString() {
		return accScalar.ToString() + " " + accOffset.ToString() + "   ";
	}

	std::string VelToString() {
		return velScalar.ToString() + " " + velOffset.ToString() + "   ";
	}

	static MPUOS Verify(MPUOS os1, MPUOS os2) {
		return MPUOS{
			os1.accOffset + os2.accOffset,
			os1.velOffset + os2.velOffset,
			os1.accScalar * os2.accScalar,
			os1.velScalar * os2.velScalar
		};
	}

} MPUOS;

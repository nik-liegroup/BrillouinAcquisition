#ifndef SCALECALIBRATIONHELPER_H
#define SCALECALIBRATIONHELPER_H

#include "../../lib/math/points.h"

#include <string>

struct ScaleCalibrationData {
	POINT2 micrometerToPixX{ 0, 0 };	// [pix/micrometer]
	POINT2 micrometerToPixY{ 0, 0 };	// [pix/micrometer]

	POINT2 pixToMicrometerX{ 0, 0 };	// [micrometer/pix]
	POINT2 pixToMicrometerY{ 0, 0 };	// [micrometer/pix]

	POINT2 originPix{ 0, 0 };			// [pix] origin on the camera image
};

// A per-objective calibration profile: the existing scale calibration (px<->um, unchanged),
// plus the FOV-center translation needed to keep grids/ROIs/overview tiles pointing at the
// same physical sample location after switching to this objective. Deliberately does NOT
// include the laser-position marker (ScanControl::m_positionScanner / the "blue circle") -
// that is sample/dish-dependent, not a property of the objective's optical path, and is
// already re-set by the operator every session; folding it in here would be wrong.
struct ObjectiveCalibrationData : public ScaleCalibrationData {
	std::string objectiveName{ "" };
	double magnification{ 0.0 };

	// This objective's FOV center relative to a declared reference objective, measured by
	// repeated switch-and-locate round trips (see the objective offset calibration
	// procedure). hasFovOffset is false until a value has actually been measured - callers
	// must treat that as "no correction available", never silently apply {0,0} as if it
	// were a validated zero offset.
	bool hasFovOffset{ false };
	POINT2 fovOffsetUm{ 0, 0 };			// [um] this objective's FOV center minus the reference objective's
	double fovOffsetSigmaUm{ 0.0 };		// [um] measured repeatability (std dev) of fovOffsetUm across calibration round trips
	std::string referenceObjectiveName{ "" };
	std::string calibrationDate{ "" };

	// The nosepiece slot this file was written from (ScanControl::getActiveObjectiveSlot() at
	// save time - see ScaleCalibration::writeCalibrationMetadata()). -1 if unset: either an
	// older file saved before this field existed, or a session with no motorized objective
	// changer at all. Only consumed by the startup calibrations-folder auto-load
	// (BrillouinAcquisition::autoLoadObjectiveCalibrations()) to decide which slot a given
	// file should be registered against without requiring the operator to be standing at that
	// objective right now - the existing manual load()/apply() workflow ignores this field
	// entirely and keeps registering against whichever slot is physically active.
	int objectiveSlot{ -1 };
};

struct Matrix2{
	double a{ 0 };
	double b{ 0 };
	double c{ 0 };
	double d{ 0 };
};

class ScaleCalibrationHelper {

public:
	static void initializeCalibrationFromMicrometer(ScaleCalibrationData* calibration) {
		// Check that the given vectors are actually a basis
		if (!isBasis(calibration->micrometerToPixX, calibration->micrometerToPixY)) {
			throw std::exception("Provided vectors are not a basis.");
		}

		auto micrometerToPix = Matrix2{ calibration->micrometerToPixX.x, calibration->micrometerToPixY.x,
										calibration->micrometerToPixX.y, calibration->micrometerToPixY.y };
		auto inverted = invert(micrometerToPix);

		calibration->pixToMicrometerX = POINT2{ inverted.a, inverted.c };
		calibration->pixToMicrometerY = POINT2{ inverted.b, inverted.d };
	}

	static void initializeCalibrationFromPixel(ScaleCalibrationData* calibration) {
		// Check that the given vectors are actually a basis
		if (!isBasis(calibration->pixToMicrometerX, calibration->pixToMicrometerY)) {
			throw std::exception("Provided vectors are not a basis.");
		}

		auto pixToMicrometer = Matrix2{ calibration->pixToMicrometerX.x, calibration->pixToMicrometerY.x,
										calibration->pixToMicrometerX.y, calibration->pixToMicrometerY.y };
		auto inverted = invert(pixToMicrometer);

		calibration->micrometerToPixX = POINT2{ inverted.a, inverted.c };
		calibration->micrometerToPixY = POINT2{ inverted.b, inverted.d };
	}

	static bool isBasis(POINT2 e_0, POINT2 e_1) {
		// Check that the absolute value of the determinant is not zero
		return abs(determinate({ e_0.x, e_1.x, e_0.y, e_1.y })) > 1e-10;
	}

	static double determinate(Matrix2 matrix) {
		return matrix.a * matrix.d - matrix.b * matrix.c;
	}

	static Matrix2 invert(Matrix2 matrix) {
		auto det = determinate(matrix);
		auto a = matrix.d / det;
		auto b = -1 * matrix.b / det;
		auto c = -1 * matrix.c / det;
		auto d = matrix.a / det;
		return Matrix2{a, b, c, d};
	}
};

#endif //SCALECALIBRATIONHELPER_H
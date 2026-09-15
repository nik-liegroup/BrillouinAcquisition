#ifndef SCALECALIBRATIONHELPER_H
#define SCALECALIBRATIONHELPER_H

#include <cmath>
#include <exception>
#include <string>
#include "../../lib/math/points.h"

struct ScaleCalibrationData {
	POINT2 micrometerToPixX{ 0, 0 };	// [pix/micrometer]
	POINT2 micrometerToPixY{ 0, 0 };	// [pix/micrometer]

	POINT2 pixToMicrometerX{ 0, 0 };	// [micrometer/pix]
	POINT2 pixToMicrometerY{ 0, 0 };	// [micrometer/pix]

	POINT2 originPix{ 0, 0 };			// [pix] origin on the camera image
};

// Keep the grid's scanner anchor independent of the marker's pixel-preserving
// rescaling. Deliberate scanner moves still move a live relative grid.
struct RelativeGridAnchor {
	POINT2 correctionUm{ 0, 0 };
	void preserveAcrossScaleChange(POINT2 oldScanner, POINT2 newScanner) {
		correctionUm += oldScanner - newScanner;
	}
	POINT2 resolve(POINT2 scanner, POINT2 objectiveOffset) const {
		return scanner + correctionUm + objectiveOffset;
	}
};

// A per-objective calibration profile: the existing scale calibration (px<->um, unchanged),
// plus the image-origin translation needed to keep grids/ROIs/overview tiles pointing at the
// same physical sample location after switching to this objective. Deliberately does NOT
// include the laser-position marker (ScanControl::m_positionScanner / the "blue circle") -
// that is sample/dish-dependent, not a property of the objective's optical path, and is
// already re-set by the operator every session; folding it in here would be wrong.
struct ObjectiveCalibrationData : public ScaleCalibrationData {
	std::string objectiveName{ "" };
	double magnification{ 0.0 };

	// Translation from the reference image's um frame into this objective's um frame:
	// A_target*(p_target-origin_target) = A_reference*(p_reference-origin_reference) + offset.
	// It includes the image-origin contribution; it is not simply a centre displacement
	// or a stage jog. hasFovOffset is false until a value has actually been measured - callers
	// must treat that as "no correction available", never silently apply {0,0} as if it
	// were a validated zero offset.
	bool hasFovOffset{ false };
	POINT2 fovOffsetUm{ 0, 0 };			// [um] translation between calibrated image frames
	double fovOffsetSigmaUm{ 0.0 };		// [um] measured repeatability (std dev) of fovOffsetUm across calibration round trips
	std::string referenceObjectiveName{ "" };
	std::string calibrationDate{ "" };

	// [um/pix] repeatability (std dev of the isotropic pixel pitch, see
	// ScaleCalibrationHelper::isotropicPixelPitchUm()) of the pix<->um scale calibration across
	// the automated "Automated calibration: Scale" cycles that produced it - 0 if the scale
	// calibration was only ever measured once (a single Acquire, not a multi-cycle run) or not
	// at all. Purely a QC/repeatability display value, not consumed by anything else.
	double scaleCalibrationSigmaUm{ 0.0 };

	// The nosepiece slot this file was written from (ScanControl::getActiveObjectiveSlot() at
	// save time - see ScaleCalibration::writeCalibrationMetadata()). -1 if unset: either an
	// older file saved before this field existed, or a session with no motorized objective
	// changer at all. Only consumed by the startup calibrations-folder auto-load
	// (BrillouinAcquisition::autoLoadObjectiveCalibrations()) to decide which slot a given
	// file should be registered against without requiring the operator to be standing at that
	// objective right now - the existing manual load()/apply() workflow ignores this field
	// entirely and keeps registering against whichever slot is physically active.
	int objectiveSlot{ -1 };

	// True for exactly one objective across the whole nosepiece - the one every other
	// objective's fovOffsetUm is ultimately relative to (composed through, if measured
	// indirectly - see ScaleCalibration::measureFovOffset()'s composition). Distinct from
	// hasFovOffset==false: hasFovOffset==false means "never measured, not a validated value at
	// all" (callers must not treat it as a validated zero); a reference objective instead
	// explicitly HAS hasFovOffset==true with fovOffsetUm=={0,0} and fovOffsetSigmaUm==0 - a
	// real, deliberately-locked value, not an absent one. This is what lets objectiveSwitched()
	// treat a switch to/from the reference exactly like any other calibrated switch (offset
	// {0,0}, sigma 0) instead of raising the "no FOV-center offset" warning on every single
	// switch involving it. Set exclusively via BrillouinAcquisition's Objective Setup dialog
	// ("Reference" checkbox, mutually exclusive across slots - see
	// ScaleCalibration::writeCalibrationToSlot()), never by measureFovOffset() itself. Optional/
	// existence-checked on read (like scaleCalibrationSigma) - an older file predating this
	// field is read as false, not rejected.
	bool isReferenceObjective{ false };
};

struct Matrix2{
	double a{ 0 };
	double b{ 0 };
	double c{ 0 };
	double d{ 0 };
};

class ScaleCalibrationHelper {

public:
	// Plot coordinates are one-based and bottom-up. Camera buffers/OpenCV are
	// zero-based and top-down: p = D*r + (1,height), D = diag(1,-1).
	static Matrix2 imageLinearTransform(const ScaleCalibrationData& reference,
		const ScaleCalibrationData& target) {
		auto inverse = invert({ target.pixToMicrometerX.x, target.pixToMicrometerY.x,
			target.pixToMicrometerX.y, target.pixToMicrometerY.y });
		return {
			inverse.a * reference.pixToMicrometerX.x + inverse.b * reference.pixToMicrometerX.y,
			-(inverse.a * reference.pixToMicrometerY.x + inverse.b * reference.pixToMicrometerY.y),
			-(inverse.c * reference.pixToMicrometerX.x + inverse.d * reference.pixToMicrometerX.y),
			inverse.c * reference.pixToMicrometerY.x + inverse.d * reference.pixToMicrometerY.y
		};
	}

	// Input: r_target = M_image*r_reference + imageTranslation.
	static POINT2 fovOffsetFromImageTranslation(const ScaleCalibrationData& reference,
		const ScaleCalibrationData& target, double referenceHeight, double targetHeight,
		POINT2 imageTranslation) {
		auto m = imageLinearTransform(reference, target);
		POINT2 plotTranslation{
			imageTranslation.x + 1.0 - m.a + m.b * referenceHeight,
			-imageTranslation.y + targetHeight + m.c - m.d * referenceHeight
		};
		return fovOffsetFromTranslation(reference, target, plotTranslation * -1.0);
	}

	// Registration gives p_target = M*p_reference - translation, where M is
	// the scale transform. Convert its translation using the stored pixel origins.
	static POINT2 fovOffsetFromTranslation(const ScaleCalibrationData& reference,
		const ScaleCalibrationData& target, POINT2 translation) {
		return POINT2{
			-target.pixToMicrometerX.x * translation.x - target.pixToMicrometerY.x * translation.y
			+ reference.pixToMicrometerX.x * reference.originPix.x + reference.pixToMicrometerY.x * reference.originPix.y
			- target.pixToMicrometerX.x * target.originPix.x - target.pixToMicrometerY.x * target.originPix.y,
			-target.pixToMicrometerX.y * translation.x - target.pixToMicrometerY.y * translation.y
			+ reference.pixToMicrometerX.y * reference.originPix.x + reference.pixToMicrometerY.y * reference.originPix.y
			- target.pixToMicrometerX.y * target.originPix.x - target.pixToMicrometerY.y * target.originPix.y
		};
	}
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

	// Isotropic-equivalent pixel pitch [um/pix]: sqrt(|determinant|) of the pix->um matrix - the
	// matrix's area-scale factor, correct regardless of any rotation between the camera's pixel
	// axes and the stage axes (unlike averaging the matrix's diagonal terms, which silently
	// assumes zero rotation). Used both to bring two different objectives' images to a
	// comparable scale before FOV-offset matching (ScaleCalibration::computeFovOffsetShiftUm())
	// and to report repeatability across repeated scale-calibration cycles (ScaleCalibration::
	// startScaleCalibrationCycle()).
	static double isotropicPixelPitchUm(const ScaleCalibrationData& calibration) {
		auto det = calibration.pixToMicrometerX.x * calibration.pixToMicrometerY.y
			- calibration.pixToMicrometerY.x * calibration.pixToMicrometerX.y;
		return std::sqrt(std::abs(det));
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

// Standalone MSVC regression test; no Qt or microscope SDK required.
// From a VS developer prompt:
// cl /EHsc /std:c++17 ObjectiveGridTransformStandalone.cpp /Fe:ObjectiveGridTransformStandalone.exe
#include <cmath>
#include <iostream>
#include <stdexcept>
#include "../BrillouinAcquisition/src/Acquisition/AcquisitionModes/ScaleCalibrationHelper.h"

static POINT2 toUm(const ScaleCalibrationData& c, POINT2 p) {
	p -= c.originPix;
	return p.x * c.pixToMicrometerX + p.y * c.pixToMicrometerY;
}

static POINT2 toPixel(const ScaleCalibrationData& c, POINT2 p) {
	return p.x * c.micrometerToPixX + p.y * c.micrometerToPixY + c.originPix;
}

static void check(POINT2 actual, POINT2 expected) {
	if (std::abs(actual.x - expected.x) > 1e-8 || std::abs(actual.y - expected.y) > 1e-8) {
		throw std::runtime_error("Objective grid projection changed the sample location");
	}
}

static void checkCameraFrameTransform() {
	ScaleCalibrationData reference, target;
	reference.pixToMicrometerX = { .04, -.47 };
	reference.pixToMicrometerY = { .46, .03 };
	target.pixToMicrometerX = { -.03, -.22 };
	target.pixToMicrometerY = { .25, -.02 };
	reference.originPix = { 123, 87 };
	target.originPix = { 71, 95 };
	ScaleCalibrationHelper::initializeCalibrationFromPixel(&reference);
	ScaleCalibrationHelper::initializeCalibrationFromPixel(&target);
	const double referenceHeight = 1024, targetHeight = 768;
	const POINT2 translation{ -615, -293 };
	auto m = ScaleCalibrationHelper::imageLinearTransform(reference, target);
	auto offset = ScaleCalibrationHelper::fovOffsetFromImageTranslation(
		reference, target, referenceHeight, targetHeight, translation);
	for (POINT2 raw : { POINT2{ 10, 80 }, POINT2{ 420, 260 }, POINT2{ 760, 540 } }) {
		POINT2 targetRaw{ m.a * raw.x + m.b * raw.y + translation.x,
			m.c * raw.x + m.d * raw.y + translation.y };
		POINT2 referencePlot{ raw.x + 1, referenceHeight - raw.y };
		POINT2 targetPlot{ targetRaw.x + 1, targetHeight - targetRaw.y };
		check(toPixel(target, toUm(reference, referencePlot) + offset), targetPlot);
	}
	auto reverse = ScaleCalibrationHelper::imageLinearTransform(target, reference);
	POINT2 reverseTranslation{ -reverse.a * translation.x - reverse.b * translation.y,
		-reverse.c * translation.x - reverse.d * translation.y };
	check(ScaleCalibrationHelper::fovOffsetFromImageTranslation(target, reference,
		targetHeight, referenceHeight, reverseTranslation), offset * -1.0);
}

static void exercise(POINT2 referenceOrigin, POINT2 targetOrigin, bool drawOnTarget) {
	ScaleCalibrationData reference, target;
	reference.originPix = referenceOrigin;
	target.originPix = targetOrigin;
	// Rotated axes, as in the supplied 10x/20x calibrations.
	reference.pixToMicrometerX = { 0, -0.468 };
	reference.pixToMicrometerY = { 0.468, 0 };
	target.pixToMicrometerX = { 0, -0.234 };
	target.pixToMicrometerY = { 0.234, 0 };
	ScaleCalibrationHelper::initializeCalibrationFromPixel(&reference);
	ScaleCalibrationHelper::initializeCalibrationFromPixel(&target);
	const POINT2 translation{ 768, 510 }; // raw match, not centre-relative (128,-2)
	auto offset = ScaleCalibrationHelper::fovOffsetFromTranslation(reference, target, translation);
	POINT2 markerPixel{ 603, 487 }; // deliberately off centre
	auto referenceScanner = toUm(reference, markerPixel);
	auto targetScanner = toUm(target, markerPixel);
	RelativeGridAnchor anchor;
	auto firstScanner = drawOnTarget ? targetScanner : referenceScanner;
	auto firstOffset = drawOnTarget ? offset : POINT2{};
	auto firstScale = drawOnTarget ? target : reference;
	POINT2 stage{ 1500, -720 };
	for (POINT2 p : { POINT2{ 400, 300 }, POINT2{ 620, 470 }, POINT2{ 890, 690 } }) {
		// One physical point viewed in both objectives, with a known image transform.
		auto expectedTarget = p * 2.0 - translation;
		auto drawnPixel = drawOnTarget ? expectedTarget : p;
		auto grid = toUm(firstScale, drawnPixel) - anchor.resolve(firstScanner, firstOffset);
		auto absolute = toUm(firstScale, drawnPixel) + stage - firstOffset;
		auto currentScanner = firstScanner;
		auto pausedStart = stage + anchor.resolve(firstScanner, firstOffset);
		auto previousOffset = firstOffset;
		for (int cycle = 0; cycle < 100; ++cycle) {
			for (bool useTarget : { true, false }) {
				auto scanner = useTarget ? targetScanner : referenceScanner;
				auto fov = useTarget ? offset : POINT2{};
				auto scale = useTarget ? target : reference;
				auto expected = useTarget ? expectedTarget : p;
				anchor.preserveAcrossScaleChange(currentScanner, scanner);
				currentScanner = scanner;
				// Both coordinate modes must land on the same physical feature.
				check(toPixel(scale, grid + anchor.resolve(scanner, fov)), expected);
				check(toPixel(scale, absolute + fov - stage), expected);
				// Starting a relative acquisition must leave the preview unchanged.
				auto start = stage + anchor.resolve(scanner, fov);
				check(toPixel(scale, grid + start - stage), expected);
				// A paused acquisition has a captured anchor; only its FOV delta changes.
				pausedStart += fov - previousOffset;
				previousOffset = fov;
				check(toPixel(scale, grid + pausedStart - stage), expected);
				// Moving to the target (backend subtracts scanner) puts it at the laser.
				auto commandedStage = grid + start - scanner;
				check(toPixel(scale, grid + start - commandedStage), markerPixel);
			}
		}
		// Return to the objective on which the next point is drawn.
		anchor.preserveAcrossScaleChange(currentScanner, firstScanner);
	}
	// Deliberately relocating the scanner still moves a relative live grid.
	check(anchor.resolve(firstScanner + POINT2{ 3, -7 }, firstOffset)
		- anchor.resolve(firstScanner, firstOffset), POINT2{ 3, -7 });
}

int main() {
	try {
		checkCameraFrameTransform();
		// The supplied 20x calibration and raw registration give this translation.
		ScaleCalibrationData measuredReference, measuredTarget;
		measuredTarget.pixToMicrometerX = { 0, -0.233900 };
		measuredTarget.pixToMicrometerY = { 0.233918, 0 };
		check(ScaleCalibrationHelper::fovOffsetFromTranslation(
			measuredReference, measuredTarget, { 768, 510 }), { -119.29818, 179.6352 });
		for (bool drawOnTarget : { false, true }) {
			exercise({ 0, 0 }, { 0, 0 }, drawOnTarget);
			exercise({ 640, 512 }, { 640, 512 }, drawOnTarget);
			exercise({ 23, 81 }, { 15, 32 }, drawOnTarget);
		}
		std::cout << "PASS: relative/absolute projection, both drawing objectives, origins, 100 round trips, acquisition start and targeting\n";
	} catch (const std::exception& e) {
		std::cerr << e.what() << '\n';
		return 1;
	}
}

#ifndef DRIFT_H
#define DRIFT_H

#include <cmath>
#include <limits>
#include <vector>

#include "points.h"
#include "registration.h"

/*
 * Sample drift from brightfield overviews: conversion of a pixel shift between two frames into stage micrometers, the combination of
 * several tiles into one step, and the decision whether a measured step is trustworthy enough to move the measurement grid
 * (DriftController). No Qt, no hardware: everything here is plain computation so it can be tested with synthetic images.
 *
 * Conventions (the same as the offline drift correction, brillouin-drift-correction, and bmlab's image export):
 *  - A drift step is the displacement of the SAMPLE in the stage's coordinates between two frames: a feature at stage position s in the
 *    earlier frame is found at s + step in the later one. To measure the same sample location after a drift of D, the stage has to go
 *    to (planned position + D).
 *  - A frame is the raw camera image as it is stored in the file. The scale calibration describes how a stage position is projected
 *    into the live view; the stored image has its rows running the other way, which is the diag(1, -1) in stageMapFromCalibration().
 *  - Each frame is captured with the stage at a known position (the stage position read back at capture time); the frame centre
 *    shows that position, so two frames taken at different stage positions are related by the difference of those positions as well
 *    as by drift (see shiftToDriftUm()).
 */
namespace drift {

	// Maps a stage offset (x, y in um) onto the offset (column, row) of the same scene in the raw frame.
	struct StageMap {
		double colX{ 0.0 };
		double colY{ 0.0 };
		double rowX{ 0.0 };
		double rowY{ 0.0 };

		double determinant() const {
			return colX * rowY - colY * rowX;
		}
		bool isValid() const {
			return std::isfinite(determinant()) && std::abs(determinant()) > 1e-12;
		}
		// [pix/um] of the (isotropic) scale
		double pixelsPerUm() const {
			return std::sqrt(std::abs(determinant()));
		}
	};

	// From the scale calibration of an objective: micrometerToPixX/Y are the pixel vectors (column, row) of one um along stage x and y in
	// the live view (ScanControl::microMeterToPix()); the stored image has its rows inverted against the live view.
	StageMap stageMapFromCalibration(POINT2 micrometerToPixX, POINT2 micrometerToPixY);

	// Sample displacement [um] between a reference frame and a later frame of the same objective, from the registered pixel shift of
	// the reference content in the later frame (registration::Match) and the stage positions the two frames were captured at.
	POINT2 shiftToDriftUm(const StageMap& map, double shiftCol, double shiftRow, POINT2 positionMovedUm, POINT2 positionReferenceUm);

	// One camera frame of an overview (a tile; the centre slice of its z stack) prepared for registration, with the stage position it was
	// captured at.
	struct TileFrame {
		POINT2 positionUm{ 0.0, 0.0 };		// stage position the frame was taken at
		// Where the tile sits in the planned (drift-free) layout: tiles of two overviews are paired by this, not by the stage position,
		// which moves with the drift offset that was applied (a step of more than the pairing distance would otherwise pair nothing).
		// NaN: the stage position is used.
		POINT2 anchorUm{ std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN() };
		registration::PreparedImage image;
		POINT2 pairingPositionUm() const { return std::isfinite(anchorUm.x) && std::isfinite(anchorUm.y) ? anchorUm : positionUm; }
	};

	// The result of comparing the tiles of one overview with those of an earlier one.
	struct StepMeasurement {
		bool valid{ false };			// at least one tile pair gave a match above the confidence limit
		POINT2 driftUm{ 0.0, 0.0 };		// [um] sample displacement since the reference overview
		int tilesCompared{ 0 };			// tile pairs that were registered
		int tilesUsed{ 0 };				// tile pairs that entered the result (above minNcc and not an outlier)
		double meanNcc{ 0.0 };			// mean normalised cross-correlation of the tiles used
		double maxNcc{ 0.0 };			// the best tile's correlation (also if below minNcc)
	};

	// Registers every tile of `moved` against the tile of `reference` at (nearly) the same stage position and combines the results: the
	// median step over the tiles with a correlation >= minNcc, the tiles further than outlierUm from it dropped, and the mean of the rest
	// weighted by correlation. Searches shifts up to maxShiftUm; tiles are paired if their stage positions are within
	// maxTileDistanceUm.
	StepMeasurement measureStep(
		const std::vector<TileFrame>& reference, const std::vector<TileFrame>& moved, const StageMap& map,
		double minNcc = 0.3, double maxShiftUm = 250.0, double maxTileDistanceUm = 60.0, double outlierUm = 3.0
	);

	struct DriftSettings {
		bool enabled{ false };
		double minNcc{ 0.3 };					// lowest mean correlation of an overview step that is applied
		double maxStepUm{ 150.0 };				// [um] larger steps are not believed
		double maxSpeedUmPerMin{ 30.0 };		// [um/min] faster drift than this is not believed
		bool extrapolate{ false };				// continue with the last measured drift speed until the next overview
	};

	enum class DriftDecision {
		REFERENCE = 0,					// first overview: it becomes the reference, drift 0 (or the carried-over drift)
		APPLIED = 1,
		REJECTED_NO_MATCH = 2,			// no tile could be registered
		REJECTED_LOW_CONFIDENCE = 3,	// correlation below the limit
		REJECTED_IMPLAUSIBLE = 4		// step larger or faster than the limits
	};

	struct DriftUpdate {
		DriftDecision decision{ DriftDecision::REJECTED_NO_MATCH };
		POINT2 stepUm{ 0.0, 0.0 };			// [um] the measured step (also if rejected; 0 if there was no match)
		POINT2 totalUm{ 0.0, 0.0 };			// [um] cumulative drift applied to the grid after this update
		double meanNcc{ 0.0 };
		int tilesUsed{ 0 };
		double secondsSinceReference{ 0.0 };
	};

	// Keeps the cumulative drift that is applied to the measurement grid and decides, overview by overview, whether to believe a measured
	// step. A step that is not believed leaves the grid where it is (and the caller keeps the old reference frames, so the next
	// overview is compared with the last accepted one over a longer time).
	class DriftController {
	public:
		explicit DriftController(DriftSettings settings = DriftSettings{}) : m_settings(settings) {}

		void setSettings(const DriftSettings& settings) { m_settings = settings; }
		const DriftSettings& settings() const { return m_settings; }

		// Forgets everything (drift 0, no reference).
		void reset();
		// The first overview of a series: becomes the reference. carriedOverUm keeps a drift measured earlier (the next repetition of a
		// time series starts where the sample was at the end of the previous one).
		DriftUpdate setReference(double timeSeconds, POINT2 carriedOverUm = POINT2{ 0.0, 0.0 });
		// A measured step relative to the current reference, captured at timeSeconds (any monotonic clock, in seconds).
		DriftUpdate update(const StepMeasurement& measurement, double timeSeconds);

		bool hasReference() const { return m_hasReference; }
		double referenceTime() const { return m_referenceTime; }
		// [um] drift applied to the grid at the last update
		POINT2 totalUm() const { return m_total; }
		// [um] drift to apply at timeSeconds: totalUm(), or with extrapolate on, continued at the speed of the last accepted step (never
		// further than maxStepUm and not longer than twice the interval it was measured over).
		POINT2 offsetAt(double timeSeconds) const;

	private:
		DriftSettings m_settings;
		bool m_hasReference{ false };
		double m_referenceTime{ 0.0 };
		POINT2 m_total{ 0.0, 0.0 };
		POINT2 m_lastStep{ 0.0, 0.0 };
		double m_lastStepSeconds{ 0.0 };
		bool m_hasLastStep{ false };
	};

}

#endif // DRIFT_H

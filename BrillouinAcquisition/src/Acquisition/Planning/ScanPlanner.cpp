#include "stdafx.h"
#include "ScanPlanner.h"
#include "src/lib/math/simplemath.h"

namespace {
	bool isPointInPolygon(const POINT2& point, const std::vector<POINT2>& polygon) {
		bool inside = false;
		const auto n = polygon.size();
		if (n < 3) {
			return true;
		}
		for (size_t i = 0, j = n - 1; i < n; j = i++) {
			const auto& pi = polygon[i];
			const auto& pj = polygon[j];

			const bool intersect = ((pi.y > point.y) != (pj.y > point.y))
				&& (point.x < (pj.x - pi.x) * (point.y - pi.y) / ((pj.y - pi.y) + 1e-12) + pi.x);
			if (intersect) {
				inside = !inside;
			}
		}
		return inside;
	}
}

/*
 * ============================================================================================
 * The XY coordinate system, in full - relative vs. absolute mode, idle vs. measuring, and how
 * the blue laser marker, the grid preview and the actual stage moves all stay consistent.
 * ============================================================================================
 *
 * This is the one place that documents the whole thing end to end. The other half of the
 * implementation - the part that actually applies the offset this file computes - is
 * ScanControl::getPositionOffset() (src/Devices/ScanControls/ScanControl.cpp); read the two
 * together.
 *
 * Quantities (all XY, all in the stage's um frame unless noted):
 *   S        stage position right now
 *   B        stored scanner/beam offset - ScanControl::m_positionScanner, objective-invariant
 *            (see ScanControl::locatePositionScanner()); the real, physical distance from the
 *            stage's reported position to where the laser actually lands on the sample
 *   R        position captured at "Start": R = S0 + B, where S0 is the stage position when
 *            Start was pressed (Brillouin::m_startPosition)
 *   O        fixed absolute-grid origin (Brillouin::resolvedGridOriginUm(), XY only - see its
 *            own comment for why z is not part of this split at all)
 *   q        a grid point's own offset from the grid's origin - what buildLegacyCartesianPlan()
 *            below actually computes (`gridPosition`), before either origin is added
 *   F        the active objective's FOV translation (fovOffsetUm) - an image-registration-based
 *            camera-frame correction, not a real stage/beam displacement
 *   project()  the active objective's calibrated um-to-pixel conversion
 *            (ScanControl::microMeterToPix())
 *
 * The XY logic, in one table:
 *
 *   Operation                  Relative mode              Absolute mode
 *   ------------------------   ------------------------   ------------------------
 *   Physical target T          R + q                      O + q
 *   Idle grid pixel            project(q + B + F)          project(O + q - S + F)
 *   Measuring grid pixel       project(R + q - S + F)      project(O + q - S + F)
 *   Commanded stage position   T - B                       T - B
 *   Blue marker pixel          project(B + F)              same
 *
 * Why this is consistent:
 *   - At Start, R - S0 = B, so the relative preview (idle formula) and the measurement overlay
 *     (measuring formula) agree at the moment Start is pressed.
 *   - At a measured point, the stage ends up at S = T - B (that's what "commanded stage
 *     position" means). So: grid pixel = project(T - S + F) = project(B + F) = blue marker
 *     pixel - the point being measured always sits exactly under the marker.
 *   - Click-to-move applies the inverse of the same conversion, including subtracting both F
 *     and B, so clicking exactly on the marker's own drawn pixel is a true no-op.
 *   - Switching between relative and absolute mode preserves the intended physical XY points:
 *     it changes their stored q (grid offset) to account for the different origin (O vs. R),
 *     not the points themselves - see BrillouinAcquisition::preservePhysicalGridForAbsoluteMode().
 *     The F terms cancel in that conversion, since both sides go through the same project().
 *
 * The one thing this model does NOT capture: B is assumed objective-independent. Switching
 * objectives changes project() and F, but leaves B untouched - the marker and a zero-offset
 * relative grid point stay coincident, but B itself does not track a real, objective-specific
 * beam displacement (a physical property of that objective's own optical path, which no
 * FOV-registration calibration can measure). If the beam has actually moved on the new
 * objective, the operator can re-mark it at runtime (see
 * BrillouinAcquisition::on_addFocusMarker_brightfield_clicked()) - that just relocates B, it
 * does not add a second, per-objective B.
 *
 * Anchoring: absolute mode is anchored to O throughout, idle and measuring alike. Relative idle
 * mode is NOT permanently sample-anchored - manually moving the stage while idle leaves the grid
 * sitting around the marker while the sample moves underneath it. It only becomes sample-anchored
 * once a measurement starts and captures R.
 */
ScanPlannerOutput ScanPlanner::buildLegacyCartesianPlan(const ScanPlannerInput& input) {
	ScanPlannerOutput output;

	std::vector<std::vector<double>> directions(3);
	directions[input.scanOrderX] = simplemath::linspace(input.xMin, input.xMax, input.xSteps);
	directions[input.scanOrderY] = simplemath::linspace(input.yMin, input.yMax, input.ySteps);
	directions[input.scanOrderZ] = simplemath::linspace(input.zMin, input.zMax, input.zSteps);

	std::vector<double> position(3);
	std::vector<int> indices(3);
	auto isRoiActive = input.useRoiMask && input.roiPolygonUm.size() >= 3;

	// Serpentine (boustrophedon) traversal: alternate the direction of each inner loop
	// every time the loop above it advances, instead of always restarting from index 0.
	// A plain nested raster otherwise flies back to the start of a row/plane on every
	// increment of the outer loop - real, avoidable stage travel that a snake path removes
	// while visiting exactly the same set of grid points, just in a different order.
	// Downstream consumers key off `indices`/absolute position, not list order, so
	// reordering here is safe.
	//
	// reverseKK is driven by a row counter that keeps running across ii (z-layer)
	// boundaries, not reset per layer - reverseJJ already makes each new layer resume at
	// whichever Y-extreme the previous layer ended on, and continuing the row parity
	// across that boundary too means the X-reversal direction also picks up exactly where
	// the previous layer left off, so the whole 3D path is continuous (a layer change only
	// steps in z, at the same x/y, regardless of whether the row counts are odd or even).
	// Resetting reverseKK per layer (e.g. from the in-layer row index alone) would only get
	// this right when the row count happens to be even.
	size_t globalRowCount = 0;
	for (size_t ii = 0; ii < directions[2].size(); ii++) {
		const auto reverseJJ = (ii % 2) == 1;
		for (size_t jjRaw = 0; jjRaw < directions[1].size(); jjRaw++) {
			const auto jj = reverseJJ ? (directions[1].size() - 1 - jjRaw) : jjRaw;
			auto lineStarted = false;
			const auto reverseKK = (globalRowCount % 2) == 1;
			globalRowCount++;
			for (size_t kkRaw = 0; kkRaw < directions[0].size(); kkRaw++) {
				const auto kk = reverseKK ? (directions[0].size() - 1 - kkRaw) : kkRaw;
				indices[0] = (int)kk;
				indices[1] = (int)jj;
				indices[2] = (int)ii;

				position[0] = directions[0][kk];
				position[1] = directions[1][jj];
				position[2] = directions[2][ii];

				POINT3 gridPosition{
					position[input.scanOrderX],
					position[input.scanOrderY],
					position[input.scanOrderZ]
				};

				const auto origin = input.gridCoordinatesAbsolute
					? input.absoluteGridOriginUm
					: input.startPosition;
				const POINT3 absolutePosition{
					gridPosition.x + origin.x,
					gridPosition.y + origin.y,
					gridPosition.z + origin.z
				};
				const POINT3 relativePosition{
					absolutePosition.x - input.startPosition.x,
					absolutePosition.y - input.startPosition.y,
					absolutePosition.z - input.startPosition.z
				};

				if (isRoiActive) {
					if (!isPointInPolygon(POINT2{ gridPosition.x, gridPosition.y }, input.roiPolygonUm)) {
						// Excluded by the ROI mask - not part of the actual plan, kept only
						// for the preview so it can still show what will be skipped.
						output.excludedPositionsRelative.push_back(relativePosition);
						output.excludedPositionsAbsolute.push_back(absolutePosition);
						continue;
					}
				}

				output.orderedPositionsRelative.push_back(relativePosition);
				output.orderedPositionsAbsolute.push_back(absolutePosition);
				output.orderedIndices.push_back(INDEX3{
					indices[input.scanOrderX],
					indices[input.scanOrderY],
					indices[input.scanOrderZ]
				});
				output.calibrationAllowed.push_back(!lineStarted);
				lineStarted = true;
			}
		}
	}

	// An ROI that excludes every grid point is a legitimate (if likely unintended) result, not
	// an error to silently work around - it means "measure nothing", not "measure everything".
	// The caller is responsible for warning/refusing to start on an empty plan (see
	// Brillouin::startRepetitions()) rather than this function guessing at a fallback.
	return output;
}

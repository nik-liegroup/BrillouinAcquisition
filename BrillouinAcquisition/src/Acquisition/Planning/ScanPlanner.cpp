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
				if (isRoiActive) {
					if (!isPointInPolygon(POINT2{ gridPosition.x, gridPosition.y }, input.roiPolygonUm)) {
						continue;
					}
				}

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

	// Safety fallback: invalid/too-strict ROI must never result in an empty plan.
	if (isRoiActive && output.orderedPositionsRelative.empty()) {
		auto fallbackInput = input;
		fallbackInput.useRoiMask = false;
		return buildLegacyCartesianPlan(fallbackInput);
	}

	return output;
}

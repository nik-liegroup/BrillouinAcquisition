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

	for (size_t ii = 0; ii < directions[2].size(); ii++) {
		for (size_t jj = 0; jj < directions[1].size(); jj++) {
			auto lineStarted = false;
			for (size_t kk = 0; kk < directions[0].size(); kk++) {
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

				const auto absolutePosition = input.gridCoordinatesAbsolute
					? gridPosition
					: gridPosition + input.startPosition;
				const auto relativePosition = absolutePosition - input.startPosition;
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

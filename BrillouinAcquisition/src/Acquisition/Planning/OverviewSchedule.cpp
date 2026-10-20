#include "stdafx.h"
#include "OverviewSchedule.h"

#include <algorithm>

namespace {

	// z index (batch) of the PLANE moments after each point: the plane's last point with z outermost, otherwise batches spread evenly
	std::vector<std::vector<int>> planeBatchesAfterPoint(const std::vector<int>& zIndexOfPoint, int zSteps, bool zOutermost) {
		const auto pointCount = zIndexOfPoint.size();
		auto batches = std::vector<std::vector<int>>(pointCount);
		if (pointCount == 0) {
			return batches;
		}
		const auto zCount = (size_t)std::max(1, zSteps);
		if (zOutermost) {
			auto lastIndexForZ = std::vector<long long>(zCount, -1);
			for (size_t ll = 0; ll < pointCount; ll++) {
				const auto zIndex = (size_t)std::clamp(zIndexOfPoint[ll], 0, (int)zCount - 1);
				lastIndexForZ[zIndex] = (long long)ll;
			}
			for (size_t zIndex = 0; zIndex < zCount; zIndex++) {
				if (lastIndexForZ[zIndex] >= 0) {
					batches[(size_t)lastIndexForZ[zIndex]].push_back((int)zIndex);
				}
			}
		} else {
			for (size_t kk = 0; kk < zCount; kk++) {
				const auto after = std::min<size_t>(pointCount - 1, ((kk + 1) * pointCount + zCount - 1) / zCount - 1);
				batches[after].push_back((int)kk);
			}
		}
		return batches;
	}

}

OverviewSchedule planOverviewSchedule(
	OVERVIEW_TRIGGER trigger, bool overviewBefore, bool overviewAfter, SURFACE_RESCAN rescan,
	const std::vector<int>& zIndexOfPoint, int zSteps, bool zOutermost
) {
	OverviewSchedule schedule;
	const auto pointCount = zIndexOfPoint.size();
	schedule.afterPoint.assign(pointCount, std::vector<OverviewMoment>{});
	schedule.rescanAfterPoint.assign(pointCount, false);
	if (pointCount == 0) {
		return schedule;
	}
	const auto lastZ = std::max(0, zSteps - 1);

	if (overviewBefore) {
		schedule.before.push_back(OverviewMoment{ OVERVIEW_MOMENT_KIND::BEFORE, 0 });
	}

	const auto planes = planeBatchesAfterPoint(zIndexOfPoint, zSteps, zOutermost);
	if (trigger == OVERVIEW_TRIGGER::PER_PLANE) {
		for (size_t ll = 0; ll < pointCount; ll++) {
			for (const auto zIndex : planes[ll]) {
				schedule.afterPoint[ll].push_back(OverviewMoment{ OVERVIEW_MOMENT_KIND::PLANE, zIndex });
			}
		}
	} else if (trigger == OVERVIEW_TRIGGER::PER_REPETITION) {
		schedule.afterPoint[pointCount - 1].push_back(OverviewMoment{ OVERVIEW_MOMENT_KIND::REPETITION, lastZ });
	}
	if (overviewAfter && schedule.afterPoint[pointCount - 1].empty()) {
		schedule.afterPoint[pointCount - 1].push_back(OverviewMoment{ OVERVIEW_MOMENT_KIND::END, lastZ });
	}

	// Repeated surface scans: never after the last point (nothing is left to measure with the new surface)
	if (rescan == SURFACE_RESCAN::WITH_OVERVIEW) {
		for (size_t ll = 0; ll + 1 < pointCount; ll++) {
			schedule.rescanAfterPoint[ll] = !schedule.afterPoint[ll].empty();
		}
	} else if (rescan == SURFACE_RESCAN::PER_PLANE) {
		for (size_t ll = 0; ll + 1 < pointCount; ll++) {
			schedule.rescanAfterPoint[ll] = !planes[ll].empty();
		}
	}
	return schedule;
}

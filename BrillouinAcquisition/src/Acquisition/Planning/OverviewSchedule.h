#ifndef OVERVIEWSCHEDULE_H
#define OVERVIEWSCHEDULE_H

#include <vector>

/*
 * When overviews (and repeated surface scans) happen during one repetition of a Brillouin measurement. Pure planning, no hardware: the
 * measurement loop asks "what is due after point ll" and this answers from the settings and the scan order.
 *
 *  - BEFORE: once before the first point (after the surface scan, if there is one).
 *  - PLANE: after the last point of a z plane (z outermost, the default scan order); with another scan order no plane is finished before
 *    the end, so the same number of moments is spread evenly over the points instead (batch k of n after point ceil((k + 1) N / n) - 1;
 *    this is what the per-z overview always did).
 *  - REPETITION: once, after the last point.
 *  - END: after the last point, only if nothing else is already due there (an "overview after the measurement" never doubles one).
 */

enum class OVERVIEW_TRIGGER {
	PER_PLANE = 0,			// after every z plane
	PER_REPETITION = 1,		// once per repetition, after the last point
	NONE = 2				// not during the measurement (only the optional start and end overviews)
};

enum class SURFACE_RESCAN {
	NEVER = 0,				// only the surface scan at the start of the repetition
	WITH_OVERVIEW = 1,		// again at every overview moment (except before the first point and after the last one)
	PER_PLANE = 2			// again after every z plane (except the last)
};

enum class OVERVIEW_MOMENT_KIND {
	BEFORE = 0,
	PLANE = 1,
	REPETITION = 2,
	END = 3
};

struct OverviewMoment {
	OVERVIEW_MOMENT_KIND kind{ OVERVIEW_MOMENT_KIND::PLANE };
	int zIndex{ 0 };		// the z plane the moment belongs to (the first for BEFORE, the last for REPETITION and END)
};

struct OverviewSchedule {
	// moments before the first point (empty or the single BEFORE moment)
	std::vector<OverviewMoment> before;
	// per measured point (acquisition order): the moments due right after it
	std::vector<std::vector<OverviewMoment>> afterPoint;
	// per measured point: whether the surface scan is to be repeated right after it (after its overview, if there is one)
	std::vector<bool> rescanAfterPoint;
};

// zIndexOfPoint: the z index of every measured point in acquisition order. zSteps: number of z planes. zOutermost: the plan visits the z
// planes one after another.
OverviewSchedule planOverviewSchedule(
	OVERVIEW_TRIGGER trigger, bool overviewBefore, bool overviewAfter, SURFACE_RESCAN rescan,
	const std::vector<int>& zIndexOfPoint, int zSteps, bool zOutermost
);

#endif // OVERVIEWSCHEDULE_H

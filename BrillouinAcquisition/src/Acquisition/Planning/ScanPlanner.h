#ifndef SCANPLANNER_H
#define SCANPLANNER_H

#include "src/lib/math/points.h"

#include <vector>

struct ScanPlannerInput {
	POINT3 startPosition;
	double xMin{ 0.0 };
	double xMax{ 0.0 };
	int xSteps{ 1 };
	double yMin{ 0.0 };
	double yMax{ 0.0 };
	int ySteps{ 1 };
	double zMin{ 0.0 };
	double zMax{ 0.0 };
	int zSteps{ 1 };
	int scanOrderX{ 0 };
	int scanOrderY{ 1 };
	int scanOrderZ{ 2 };
	bool useRoiMask{ false };
	std::vector<POINT2> roiPolygonUm;
	bool gridCoordinatesAbsolute{ false };
	POINT3 absoluteGridOriginUm{ 0.0, 0.0, 0.0 };
};

struct ScanPlannerOutput {
	std::vector<POINT3> orderedPositionsAbsolute;
	std::vector<POINT3> orderedPositionsRelative;
	std::vector<INDEX3> orderedIndices;
	std::vector<bool> calibrationAllowed;
};

class ScanPlanner {
public:
	static ScanPlannerOutput buildLegacyCartesianPlan(const ScanPlannerInput& input);
};

#endif // SCANPLANNER_H

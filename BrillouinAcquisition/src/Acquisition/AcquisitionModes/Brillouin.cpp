#include "stdafx.h"
#include "Brillouin.h"
#include "src/Acquisition/Planning/ScanPlanner.h"
#include "src/lib/math/simplemath.h"
#include "src/helper/logger.h"
#include "filesystem"

#include <chrono>
#include <thread>
#include <limits>
#include <map>
#include <set>
#include <functional>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <optional>
#include <numeric>

using namespace std::filesystem;

namespace {
// Overview mosaic tiling: fixed 20% overlap between neighboring tiles, shared between the
// actual capture positions (Brillouin::overviewTileCentersXY) and the outline drawn in the
// live view (Brillouin::overviewTileOutlinesUm) so the two always agree.
constexpr double kOverviewTileOverlapFraction = 0.2;

// Minimum number of FOV-sized tiles, spaced at (1 - overlap) * fov, needed to span `extent`.
int overviewRequiredTileCount(double extent, double fov, double overlap) {
	if (extent <= fov) {
		return 1;
	}
	const auto pitch = fov * (1.0 - overlap);
	return 1 + (int)std::ceil((extent - fov) / pitch);
}

/*
 * Tile centers along one axis, spaced at exactly (1 - overlap) * fov - i.e. the overlap between
 * neighboring tiles is always exactly `overlap`, never squeezed tighter to fit the point extent
 * precisely. The tiles are centered on the extent's midpoint, so whatever's left over between
 * the tiled span and the point extent overshoots symmetrically on both ends rather than landing
 * exactly on the outermost points.
 */
std::vector<double> overviewTileCentersAlongAxis(double minEdge, double extent, double fov, double overlap, int count) {
	std::vector<double> centers(count);
	const auto center = minEdge + 0.5 * extent;
	if (count <= 1) {
		centers[0] = center;
		return centers;
	}
	const auto pitch = fov * (1.0 - overlap);
	const auto start = center - 0.5 * (count - 1) * pitch;
	for (int i = 0; i < count; i++) {
		centers[i] = start + i * pitch;
	}
	return centers;
}

bool isPointInPolygonUm(const POINT2& point, const std::vector<POINT2>& polygon) {
	if (polygon.size() < 3) {
		return false;
	}
	bool inside = false;
	size_t j = polygon.size() - 1;
	for (size_t i = 0; i < polygon.size(); ++i) {
		const auto& pi = polygon[i];
		const auto& pj = polygon[j];
		const bool intersects = ((pi.y > point.y) != (pj.y > point.y))
			&& (point.x < (pj.x - pi.x) * (point.y - pi.y) / ((pj.y - pi.y) + 1e-12) + pi.x);
		if (intersects) {
			inside = !inside;
		}
		j = i;
	}
	return inside;
}

// Sutherland-Hodgman: clips `poly` (any simple polygon, closed implicitly) against the
// axis-aligned rectangle [xmin,xmax] x [ymin,ymax] - a rectangle is always convex, so this is
// exact regardless of whether `poly` itself is convex or concave. Used by
// Brillouin::additionalBoundaryXYPoints() so a boundary candidate can never fall outside the
// grid, even when the ROI polygon itself does.
std::vector<POINT2> clipPolygonToRect(const std::vector<POINT2>& poly, double xmin, double xmax, double ymin, double ymax) {
	auto clipEdge = [](
		const std::vector<POINT2>& points,
		const std::function<bool(const POINT2&)>& inside,
		const std::function<POINT2(const POINT2&, const POINT2&)>& intersect
	) {
		std::vector<POINT2> out;
		for (size_t i = 0; i < points.size(); i++) {
			const auto& curr = points[i];
			const auto& prev = points[(i + points.size() - 1) % points.size()];
			const auto currIn = inside(curr);
			const auto prevIn = inside(prev);
			if (currIn) {
				if (!prevIn) {
					out.push_back(intersect(prev, curr));
				}
				out.push_back(curr);
			} else if (prevIn) {
				out.push_back(intersect(prev, curr));
			}
		}
		return out;
	};
	auto pts = poly;
	pts = clipEdge(pts,
		[xmin](const POINT2& p) { return p.x >= xmin; },
		[xmin](const POINT2& a, const POINT2& b) {
			const auto t = (xmin - a.x) / (b.x - a.x);
			return POINT2{ xmin, a.y + t * (b.y - a.y) };
		});
	pts = clipEdge(pts,
		[xmax](const POINT2& p) { return p.x <= xmax; },
		[xmax](const POINT2& a, const POINT2& b) {
			const auto t = (xmax - a.x) / (b.x - a.x);
			return POINT2{ xmax, a.y + t * (b.y - a.y) };
		});
	pts = clipEdge(pts,
		[ymin](const POINT2& p) { return p.y >= ymin; },
		[ymin](const POINT2& a, const POINT2& b) {
			const auto t = (ymin - a.y) / (b.y - a.y);
			return POINT2{ a.x + t * (b.x - a.x), ymin };
		});
	pts = clipEdge(pts,
		[ymax](const POINT2& p) { return p.y <= ymax; },
		[ymax](const POINT2& a, const POINT2& b) {
			const auto t = (ymax - a.y) / (b.y - a.y);
			return POINT2{ a.x + t * (b.x - a.x), ymax };
		});
	return pts;
}

// Walks `poly`'s perimeter (closed implicitly) at even arc-length spacing and returns
// `count` points - the dense candidate pool additionalBoundaryXYPoints() runs its
// farthest-point selection over.
std::vector<POINT2> polygonPerimeterPoints(const std::vector<POINT2>& poly, int count) {
	std::vector<POINT2> pts;
	if (count <= 0 || poly.size() < 2) {
		return pts;
	}
	const auto n = poly.size();
	std::vector<double> edgeLen(n);
	double total = 0.0;
	for (size_t i = 0; i < n; i++) {
		const auto& a = poly[i];
		const auto& b = poly[(i + 1) % n];
		edgeLen[i] = std::hypot(b.x - a.x, b.y - a.y);
		total += edgeLen[i];
	}
	if (total <= 0.0) {
		return pts;
	}
	pts.reserve(count);
	for (int k = 0; k < count; k++) {
		const auto target = total * k / count;
		double acc = 0.0;
		for (size_t i = 0; i < n; i++) {
			const auto reach = acc + edgeLen[i];
			if (reach >= target || i == n - 1) {
				const auto& a = poly[i];
				const auto& b = poly[(i + 1) % n];
				const auto frac = edgeLen[i] > 0.0 ? std::clamp((target - acc) / edgeLen[i], 0.0, 1.0) : 0.0;
				pts.push_back(POINT2{ a.x + frac * (b.x - a.x), a.y + frac * (b.y - a.y) });
				break;
			}
			acc = reach;
		}
	}
	return pts;
}

// Greedy farthest-point / max-min-distance selection: each pick maximizes the minimum
// distance to every point already in `existingPoints` or already selected this call -
// incremental by construction (the k-th pick never moves any earlier one, so raising
// `maxCount` by one just appends a point rather than recomputing the whole set).
std::vector<POINT2> farthestPointSequence(const std::vector<POINT2>& candidates, const std::vector<POINT2>& existingPoints, int maxCount) {
	std::vector<POINT2> selected;
	auto pool = candidates;
	auto reference = existingPoints;
	for (int k = 0; k < maxCount && !pool.empty(); k++) {
		auto bestIdx = -1;
		auto bestDist = -std::numeric_limits<double>::infinity();
		for (size_t i = 0; i < pool.size(); i++) {
			auto minD = std::numeric_limits<double>::infinity();
			for (const auto& ref : reference) {
				const auto d = std::hypot(pool[i].x - ref.x, pool[i].y - ref.y);
				if (d < minD) {
					minD = d;
				}
			}
			if (minD > bestDist) {
				bestDist = minD;
				bestIdx = (int)i;
			}
		}
		selected.push_back(pool[bestIdx]);
		reference.push_back(pool[bestIdx]);
		pool.erase(pool.begin() + bestIdx);
	}
	return selected;
}
}

/*
 * Public definitions
 */

Brillouin::Brillouin(QObject* parent, Acquisition* acquisition, Camera*& andor, Camera*& brightfieldCamera, ScanControl*& scanControl)
	: AcquisitionMode(parent, acquisition, scanControl), m_andor(andor), m_brightfieldCamera(brightfieldCamera) {
	static QMetaObject::Connection connection = QWidget::connect(
		this,
		&Brillouin::s_scanOrderChanged,
		this,
		[this](SCAN_ORDER scanOrder) { updatePositions(); }
	);
	// Emit the initial positions
	updatePositions();
}

Brillouin::~Brillouin() {
	if (m_repetitionTimer) {
		m_repetitionTimer->stop();
		m_repetitionTimer->deleteLater();
	}
}

/*
 * Public slots
 */

void Brillouin::startRepetitions() {
	bool allowed = m_acquisition->enableMode(ACQUISITION_MODE::BRILLOUIN);
	if (!allowed) {
		return;
	}

	// Absolute-mode positions silently target the wrong physical location by exactly the
	// active objective's FOV-center offset if that offset was never calibrated (see
	// resolvedGridOriginUm()) - unlike relative mode, there is no live re-anchoring to save
	// it. Refuse to start rather than measure at an unverified location; the operator already
	// saw this exact condition as a blocking warning at objective-switch time (see
	// ScanControl::s_objectiveSwitched()) and either accepted it (isMissingObjectiveOffsetAccepted())
	// or should fix it before measuring, not have it silently ignored here.
	if (m_settings.gridCoordinatesAbsolute && m_scanControl) {
		const auto activeCalibration = m_scanControl->getActiveObjectiveCalibration();
		if (!activeCalibration.hasFovOffset && !m_scanControl->isMissingObjectiveOffsetAccepted()) {
			qWarning(logWarning()) << "Brillouin::startRepetitions: refusing to start - absolute-mode grid "
				"has no calibrated FOV-center offset for the active objective, and the missing-offset "
				"warning was not accepted.";
			m_acquisition->disableMode(ACQUISITION_MODE::BRILLOUIN);
			setAcquisitionStatus(ACQUISITION_STATUS::ABORTED);
			return;
		}
	}

	// If the repetition timer is running already, we stop the next repetition
	if (m_repetitionTimer != nullptr && m_repetitionTimer->isActive()) {
		m_repetitionTimer->stop();
		m_startOfLastRepetition.invalidate();
		finaliseRepetitions(m_currentRepetition, -2);
		setAcquisitionStatus(ACQUISITION_STATUS::STOPPED);
		return;
	}

	m_abort = false;

	auto info = std::string{ "Acquisition started." };
	qInfo(logInfo()) << info.c_str();

	m_currentRepetition = 0;
	m_startOfLastRepetition.start();

	m_repetitionTimer = new QTimer();
	QMetaObject::Connection connection = QWidget::connect(
		m_repetitionTimer,
		&QTimer::timeout,
		this,
		&Brillouin::waitForNextRepetition
	);
	m_repetitionTimer->start(100);
}

void Brillouin::waitForNextRepetition() {

	if (m_abort) {
		this->abortMode(m_acquisition->m_storage);
		return;
	}
	
	// Save the filename of the first repetition
	if (m_currentRepetition == 0) {
		m_baseFilename = m_acquisition->getCurrentFilename();
	}

	// Check if we have to start a new repetition or wait more
	auto timeSinceLast = int{ (int)(1e-3 * m_startOfLastRepetition.elapsed()) };
	if (m_currentRepetition == 0 || timeSinceLast >= m_settings.repetitions.interval * 60) {
		m_startOfLastRepetition.restart();
		m_repetitionTimer->stop();
		emit(s_totalProgress(m_currentRepetition, -1));

		if (m_settings.repetitions.filePerRepetition && m_currentRepetition != 0) {
			auto repetitionFilename = getRepetitionFilename();
			m_acquisition->openFile(repetitionFilename, true);
		}

		m_acquisition->newRepetition(ACQUISITION_MODE::BRILLOUIN);
		if (m_settings.saveOverviewBrightfieldPerZ) {
			m_acquisition->newRepetition(ACQUISITION_MODE::FLUORESCENCE);
		}

		setAcquisitionStatus(ACQUISITION_STATUS::STARTED);
		acquire(m_acquisition->m_storage);

		if (m_abort) {
			this->abortMode(m_acquisition->m_storage);
			return;
		}
		if (getStatus() == ACQUISITION_STATUS::WAITFORSURFACEREVIEW) {
			// Paused so the user can review the surface scan - continueAfterSurfaceReview()
			// (Continue/Full grid buttons) runs the actual measurement and then calls
			// finishRepetition() itself once that completes.
			return;
		}
		finishRepetition();
	} else {
		timeSinceLast = 1e-3 * m_startOfLastRepetition.elapsed();
		emit(s_totalProgress(m_currentRepetition, m_settings.repetitions.interval * 60 - timeSinceLast));
	}
}

void Brillouin::finishRepetition() {
	m_currentRepetition++;
	// Check if this was the last repetition
	if (m_currentRepetition < m_settings.repetitions.count) {
		m_repetitionTimer->start(100);
		setAcquisitionStatus(ACQUISITION_STATUS::WAITFORREPETITION);
	} else {
		m_startOfLastRepetition.invalidate();
		// Cleanup after last repetition
		finaliseRepetitions();
		setAcquisitionStatus(ACQUISITION_STATUS::FINISHED);
	}
}

void Brillouin::finaliseRepetitions() {
	finaliseRepetitions(m_settings.repetitions.count, -1);
}

void Brillouin::finaliseRepetitions(int nrFinishedRepetitions, int status) {
	emit(s_totalProgress(nrFinishedRepetitions, status));
	m_acquisition->disableMode(ACQUISITION_MODE::BRILLOUIN);
}

void Brillouin::setStepNumberX(int steps) {
	m_settings.setXSteps(steps);
	determineScanOrder();
}

void Brillouin::setStepNumberY(int steps) {
	m_settings.setYSteps(steps);
	determineScanOrder();
}

void Brillouin::setStepNumberZ(int steps) {
	m_settings.setZSteps(steps);
	determineScanOrder();
}

void Brillouin::setXMin(double xMin) {
	m_settings.setXMin(xMin);
	updatePositions();
}

void Brillouin::setXMax(double xMax) {
	m_settings.setXMax(xMax);
	updatePositions();
}

void Brillouin::setYMin(double yMin) {
	m_settings.setYMin(yMin);
	updatePositions();
}

void Brillouin::setYMax(double yMax) {
	m_settings.setYMax(yMax);
	updatePositions();
}

void Brillouin::setZMin(double zMin) {
	m_settings.setZMin(zMin);
	updatePositions();
}

void Brillouin::setZMax(double zMax) {
	m_settings.setZMax(zMax);
	updatePositions();
}

void Brillouin::setSettings(const BRILLOUIN_SETTINGS& settings) {
	m_settings = settings;
}

/*
 *	Scan direction order related variables and functions
 */

void Brillouin::setScanOrderX(int x) {
	if (m_scanOrder.automatical) {
		emit(s_scanOrderChanged(m_scanOrder));
		return;
	}
	// switch values
	if (m_scanOrder.y == x) {
		m_scanOrder.y = m_scanOrder.x;
	}
	if (m_scanOrder.z == x) {
		m_scanOrder.z = m_scanOrder.x;
	}
	m_scanOrder.x = x;
	emit(s_scanOrderChanged(m_scanOrder));
}

void Brillouin::setScanOrderY(int y) {
	if (m_scanOrder.automatical) {
		emit(s_scanOrderChanged(m_scanOrder));
		return;
	}
	// switch values
	if (m_scanOrder.x == y) {
		m_scanOrder.x = m_scanOrder.y;
	}
	if (m_scanOrder.z == y) {
		m_scanOrder.z = m_scanOrder.y;
	}
	m_scanOrder.y = y;
	emit(s_scanOrderChanged(m_scanOrder));
}

void Brillouin::setScanOrderZ(int z) {
	if (m_scanOrder.automatical) {
		emit(s_scanOrderChanged(m_scanOrder));
		return;
	}
	// switch values
	if (m_scanOrder.x == z) {
		m_scanOrder.x = m_scanOrder.z;
	}
	if (m_scanOrder.y == z) {
		m_scanOrder.y = m_scanOrder.z;
	}
	m_scanOrder.z = z;
	emit(s_scanOrderChanged(m_scanOrder));
}

void Brillouin::setScanOrderAuto(bool automatical) {
	m_scanOrder.automatical = automatical;
	determineScanOrder();
}

void Brillouin::determineScanOrder() {
	if (m_scanOrder.automatical) {
		// determine scan order based on step numbers
		// highest step number first, then descending
		auto stepNumbers = std::vector<int>{ m_settings.xSteps, m_settings.ySteps, m_settings.zSteps };
		auto indices = simplemath::tag_sort_inverse(stepNumbers);
		auto order = std::vector<int>(stepNumbers.size());
		for (gsl::index jj{ 0 }; jj < order.size(); jj++) {
			order[indices[jj]] = jj;
		}

		m_scanOrder.x = order[0];
		m_scanOrder.y = order[1];
		m_scanOrder.z = order[2];

	}
	emit(s_scanOrderChanged(m_scanOrder));
}

std::vector<POINT3> Brillouin::getOrderedPositions() {
	return m_orderedPositionsRelative;
}

std::vector<INDEX3> Brillouin::getOrderedIndices() const {
	return m_orderedIndices;
}

std::set<std::pair<int, int>> Brillouin::getSurfaceFoundXYIndices() const {
	return m_surfaceFoundXYIndices;
}

std::set<std::pair<int, int>> Brillouin::getSurfaceInterpolatedXYIndices() const {
	return m_surfaceInterpolatedXYIndices;
}

/*
 * Private definitions
 */

void Brillouin::abortMode(std::unique_ptr <StorageWrapper>& storage) {
	m_repetitionTimer->stop();
	m_startOfLastRepetition.invalidate();
	if (m_andor) {
		m_andor->stopAcquisition();
	}

	if (m_scanControl) {
		m_scanControl->setPreset(ScanPreset::SCAN_LASEROFF);
		// Acquisition is aborting - don't leave the RL shutter forced open.
		m_scanControl->setRLShutterOpen(false);
		m_scanControl->setPositionCompensated(m_startPosition);
		m_scanControl->enableMeasurementMode(false);
		QMetaObject::invokeMethod(
			m_scanControl,
			[scanControl = m_scanControl]() { scanControl->startAnnouncing(); },
			Qt::AutoConnection
		);
	}

	m_acquisition->disableMode(ACQUISITION_MODE::BRILLOUIN);

	// Here we wait until the storage object indicate it finished to write to the file.
	QEventLoop loop;
	auto connection = QWidget::connect(
		storage.get(),
		&StorageWrapper::finished,
		&loop,
		&QEventLoop::quit
	);
	QMetaObject::invokeMethod(
		storage.get(),
		[&storage = storage]() { storage.get()->s_finishedQueueing(); },
		Qt::AutoConnection
	);
	loop.exec();

	setAcquisitionStatus(ACQUISITION_STATUS::ABORTED);
	emit(s_positionChanged({ 0 , 0, 0 }, 0));
	emit(s_timeToCalibration(0));
}

void Brillouin::calibrate(std::unique_ptr <StorageWrapper>& storage) {
	// announce calibration start
	emit(s_calibrationRunning(true));

	// Capture the exposure time to restore afterward *before* touching anything, and use
	// this local copy for every revert below instead of re-reading m_settings.camera.exposureTime
	// at the end. That field is shared (by reference) with the GUI's exposure spinbox setting,
	// so anything that ever pushes a stale/default value into it while this calibration is
	// mid-flight (e.g. a camera settings reload) would otherwise make the "revert" below
	// restore the wrong value instead of what was actually active before this calibration.
	const auto originalExposureTime = m_settings.camera.exposureTime;

	// set exposure time for calibration
	if (m_andor) {
		m_andor->setCalibrationExposureTime(m_settings.calibrationExposureTime);
	}

	// move optical elements to position for calibration
	if (m_scanControl) {
		m_scanControl->setPreset(ScanPreset::SCAN_CALIBRATION);
		// Calibration is part of the acquisition, same as the actual measurement - keep
		// the RL shutter open for it too.
		m_scanControl->setRLShutterOpen(true);
	}
	std::this_thread::sleep_for(std::chrono::milliseconds(500));

	auto shift = 5.088; // this is the shift for water

	// acquire images
	auto rank_cal = 3;
	hsize_t dims_cal[3] = {
		(hsize_t)m_settings.nrCalibrationImages,
		(hsize_t)m_settings.camera.roi.height_binned,
		(hsize_t)m_settings.camera.roi.width_binned
	};

	auto images = std::vector<std::byte>((int64_t)m_settings.camera.roi.bytesPerFrame * m_settings.nrCalibrationImages);
	for (gsl::index mm{ 0 }; mm < m_settings.nrCalibrationImages; mm++) {
		if (m_abort) {
			// Undo the calibration-only exposure time/optics preset set above before
			// bailing out - otherwise an abort mid-calibration leaves the camera stuck at
			// the calibration exposure time (and the calibration optics preset) forever,
			// silently affecting every measurement after this one.
			if (m_scanControl) {
				m_scanControl->setPreset(ScanPreset::SCAN_BRILLOUIN);
				m_scanControl->setRLShutterOpen(true);
			}
			if (m_andor) {
				m_andor->setCalibrationExposureTime(originalExposureTime);
			}
			this->abortMode(storage);
			return;
		}
		// acquire images
		auto pointerPos = (int64_t)m_settings.camera.roi.bytesPerFrame * mm;

		if (m_andor) {
			m_andor->getImageForAcquisition(&images[pointerPos]);
		}
	}

	// the datetime has to be set here, otherwise it would be determined by the time the queue is processed
	auto date = QDateTime::currentDateTime().toOffsetFromUtc(QDateTime::currentDateTime().offsetFromUtc())
		.toString(Qt::ISODateWithMs).toStdString();

	if (m_settings.camera.readout.dataType == "unsigned short") {
		// cast the image to unsigned short
		auto images_ = (std::vector<unsigned short> *) & images;
		auto cal = new CALIBRATION<unsigned short>(
			nrCalibrations,			// index
			*images_,				// data
			rank_cal,				// the rank of the calibration data
			dims_cal,				// the dimension of the calibration data
			m_settings.sample,		// the samplename
			shift,					// the Brillouin shift of the sample
			date,					// the datetime
			m_settings.calibrationExposureTime, // the exposure time of the calibration
			m_settings.camera.gain,
			m_settings.camera.roi
			);

		QMetaObject::invokeMethod(
			storage.get(),
			[&storage = storage, cal]() { storage.get()->s_enqueueCalibration(cal); },
			Qt::AutoConnection
		);
	} else if (m_settings.camera.readout.dataType == "unsigned char") {
		// cast the image to unsigned char
		auto images_ = (std::vector<unsigned char> *) & images;
		auto cal = new CALIBRATION<unsigned char>(
			nrCalibrations,			// index
			*images_,				// data
			rank_cal,				// the rank of the calibration data
			dims_cal,				// the dimension of the calibration data
			m_settings.sample,		// the samplename
			shift,					// the Brillouin shift of the sample
			date,					// the datetime
			m_settings.calibrationExposureTime, // the exposure time of the calibration
			m_settings.camera.gain,
			m_settings.camera.roi
			);

		QMetaObject::invokeMethod(
			storage.get(),
			[&storage = storage, cal]() { storage.get()->s_enqueueCalibration(cal); },
			Qt::AutoConnection
		);
	} else if (m_settings.camera.readout.dataType == "unsigned int") {
		// cast the image to unsigned char
		auto images_ = (std::vector<unsigned int> *) & images;
		auto cal = new CALIBRATION<unsigned int>(
			nrCalibrations,			// index
			*images_,				// data
			rank_cal,				// the rank of the calibration data
			dims_cal,				// the dimension of the calibration data
			m_settings.sample,		// the samplename
			shift,					// the Brillouin shift of the sample
			date,					// the datetime
			m_settings.calibrationExposureTime, // the exposure time of the calibration
			m_settings.camera.gain,
			m_settings.camera.roi
			);

		QMetaObject::invokeMethod(
			storage.get(),
			[&storage = storage, cal]() { storage.get()->s_enqueueCalibration(cal); },
			Qt::AutoConnection
		);
	}

	nrCalibrations++;

	// revert optical elements to position for brightfield/Brillouin imaging
	if (m_scanControl) {
		m_scanControl->setPreset(ScanPreset::SCAN_BRILLOUIN);
		m_scanControl->setRLShutterOpen(true);
	}

	// reset exposure time
	if (m_andor) {
		m_andor->setCalibrationExposureTime(originalExposureTime);
	}
	std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

/*
 * Construct positions vector with correct order of scan directions
 */
void Brillouin::updatePositions() {
	// Create a local copy of the settings object to prevent subscript-out-of-range error
	// due to race-condition.
	auto settings = m_settings;

	ScanPlannerInput plannerInput{};
	plannerInput.startPosition = m_startPosition;
	plannerInput.xMin = settings.xMin;
	plannerInput.xMax = settings.xMax;
	plannerInput.xSteps = settings.xSteps;
	plannerInput.yMin = settings.yMin;
	plannerInput.yMax = settings.yMax;
	plannerInput.ySteps = settings.ySteps;
	plannerInput.zMin = settings.zMin;
	plannerInput.zMax = settings.zMax;
	plannerInput.zSteps = settings.zSteps;
	plannerInput.scanOrderX = m_scanOrder.x;
	plannerInput.scanOrderY = m_scanOrder.y;
	plannerInput.scanOrderZ = m_scanOrder.z;
	plannerInput.useRoiMask = settings.useRoiMask;
	plannerInput.roiPolygonUm = settings.roiPolygonUm;
	plannerInput.gridCoordinatesAbsolute = settings.gridCoordinatesAbsolute;
	plannerInput.absoluteGridOriginUm = resolvedGridOriginUm();

	auto plan = ScanPlanner::buildLegacyCartesianPlan(plannerInput);
	m_orderedPositions = std::move(plan.orderedPositionsAbsolute);
	m_orderedPositionsRelative = std::move(plan.orderedPositionsRelative);
	m_orderedIndices = std::move(plan.orderedIndices);
	m_calibrationAllowed = std::move(plan.calibrationAllowed);
	m_excludedPositions = std::move(plan.excludedPositionsAbsolute);
	m_excludedPositionsRelative = std::move(plan.excludedPositionsRelative);

	if (m_settings.gridCoordinatesAbsolute) {
		emit(s_orderedPositionsChanged(m_orderedPositions));
		emit(s_excludedPositionsChanged(m_excludedPositions));
	} else {
		emit(s_orderedPositionsChanged(m_orderedPositionsRelative));
		emit(s_excludedPositionsChanged(m_excludedPositionsRelative));
	}
}

bool Brillouin::remapProxyRoi(
	int roiLeft, int roiTop, int roiWidth, int roiHeight,
	const PROXY_ROI_FRAME& from, const PROXY_ROI_FRAME& to,
	int& outLeft, int& outTop, int& outWidth, int& outHeight
) {
	if (from.width <= 0 || from.height <= 0 || to.width <= 0 || to.height <= 0) {
		return false;
	}
	if (from.width == to.width && from.height == to.height
			&& from.originLeft == to.originLeft && from.originBottom == to.originBottom
			&& from.widthPhysical == to.widthPhysical && from.heightPhysical == to.heightPhysical) {
		// Identical frame - use the coordinates as-is.
		outLeft = roiLeft;
		outTop = roiTop;
		outWidth = roiWidth;
		outHeight = roiHeight;
		return true;
	}
	if (from.widthPhysical <= 0 || from.heightPhysical <= 0 || to.widthPhysical <= 0 || to.heightPhysical <= 0) {
		// Missing physical geometry (e.g. a settings file saved before this existed) -
		// fall back to the old, origin-blind proportional rescale. Still wrong whenever
		// the two frames don't share a sensor origin, same as before this function existed.
		const auto scaleX = (double)to.width / from.width;
		const auto scaleY = (double)to.height / from.height;
		outLeft = (int)std::lround(roiLeft * scaleX);
		outTop = (int)std::lround(roiTop * scaleY);
		outWidth = (int)std::lround(roiWidth * scaleX);
		outHeight = (int)std::lround(roiHeight * scaleY);
		return true;
	}

	// Physical (pre-binning) sensor pixels spanned by one binned cell, in `from` and `to`.
	const auto fromPixelsPerCellX = (double)from.widthPhysical / from.width;
	const auto fromPixelsPerCellY = (double)from.heightPhysical / from.height;
	const auto toPixelsPerCellX = (double)to.widthPhysical / to.width;
	const auto toPixelsPerCellY = (double)to.heightPhysical / to.height;

	// Local (binned, `from`) -> absolute physical sensor position.
	const auto absLeft = from.originLeft + roiLeft * fromPixelsPerCellX;
	const auto absRight = from.originLeft + (roiLeft + roiWidth) * fromPixelsPerCellX;
	const auto absBottom = from.originBottom + roiTop * fromPixelsPerCellY;
	const auto absTop = from.originBottom + (roiTop + roiHeight) * fromPixelsPerCellY;

	// Absolute physical sensor position -> local (binned, `to`).
	const auto toLeft = (absLeft - to.originLeft) / toPixelsPerCellX;
	const auto toRight = (absRight - to.originLeft) / toPixelsPerCellX;
	const auto toBottom = (absBottom - to.originBottom) / toPixelsPerCellY;
	const auto toTop = (absTop - to.originBottom) / toPixelsPerCellY;

	outLeft = (int)std::lround(toLeft);
	outTop = (int)std::lround(toBottom);
	outWidth = (int)std::lround(toRight - toLeft);
	outHeight = (int)std::lround(toTop - toBottom);
	return true;
}

double Brillouin::estimateFrameMetric(const std::vector<std::byte>& image) const {
	if (image.empty()) {
		return 0.0;
	}

	const int width = (int)m_settings.camera.roi.width_binned;
	const int height = (int)m_settings.camera.roi.height_binned;
	if (width <= 0 || height <= 0) {
		return 0.0;
	}

	auto getDisplayValue = [&](int x, int displayY) -> double {
		const auto rawY = height - 1 - displayY;
		const auto idx = (size_t)rawY * width + x;
		if (m_settings.camera.readout.dataType == "unsigned short") {
			const auto* data = reinterpret_cast<const unsigned short*>(image.data());
			return data[idx];
		}
		if (m_settings.camera.readout.dataType == "unsigned int") {
			const auto* data = reinterpret_cast<const unsigned int*>(image.data());
			return data[idx];
		}
		const auto* data = reinterpret_cast<const unsigned char*>(image.data());
		return data[idx];
	};

	std::vector<double> metrics;
	auto appendMetric = [&](int roiLeft, int roiDisplayBottom, int roiWidth, int roiHeight) {
		if (roiWidth <= 0 || roiHeight <= 0) {
			return;
		}

		roiLeft = std::max(0, roiLeft);
		roiDisplayBottom = std::max(0, roiDisplayBottom);
		roiWidth = std::min(roiWidth, width - roiLeft);
		roiHeight = std::min(roiHeight, height - roiDisplayBottom);
		if (roiWidth <= 0 || roiHeight <= 0) {
			return;
		}

		auto maxSignal = -std::numeric_limits<double>::infinity();
		for (int displayY = roiDisplayBottom; displayY < roiDisplayBottom + roiHeight; displayY++) {
			for (int x = roiLeft; x < roiLeft + roiWidth; x++) {
				maxSignal = std::max(maxSignal, getDisplayValue(x, displayY));
			}
		}
		if (!std::isfinite(maxSignal)) {
			return;
		}
		metrics.push_back(maxSignal);
	};

	// A spectral ROI is recorded against whatever frame (crop + binning) was active when it
	// was drawn. Reapplying its local pixel coordinates directly - or merely rescaling them
	// by frame size - is only correct if the current frame shares the same sensor origin as
	// when it was drawn. If the camera ROI was zoomed/cropped to a different region since
	// (e.g. drawn while zoomed in, then measured against the full sensor, or vice versa),
	// a size-only rescale silently lands the rectangle on the wrong physical location
	// instead of raising an error - which is what "surface never found despite a visible
	// drop", or a spectral ROI measuring somewhere other than where it was drawn, looks
	// like. remapProxyRoi() remaps it through its absolute sensor position instead.
	const PROXY_ROI_FRAME currentFrame{
		width, height,
		m_settings.camera.roi.left, m_settings.camera.roi.bottom,
		m_settings.camera.roi.width_physical, m_settings.camera.roi.height_physical
	};
	auto appendProxyRoi = [&](
		int roiLeft, int roiTop, int roiWidth, int roiHeight,
		const PROXY_ROI_FRAME& drawnFrame
	) {
		int outLeft{ 0 }, outTop{ 0 }, outWidth{ 0 }, outHeight{ 0 };
		if (remapProxyRoi(roiLeft, roiTop, roiWidth, roiHeight, drawnFrame, currentFrame,
				outLeft, outTop, outWidth, outHeight)) {
			appendMetric(outLeft, outTop, outWidth, outHeight);
		}
	};

	appendProxyRoi(
		m_settings.surfaceProxyRoiLeft,
		m_settings.surfaceProxyRoiTop,
		m_settings.surfaceProxyRoiWidth,
		m_settings.surfaceProxyRoiHeight,
		PROXY_ROI_FRAME{
			m_settings.surfaceProxyRoiFrameWidth, m_settings.surfaceProxyRoiFrameHeight,
			m_settings.surfaceProxyRoiFrameOriginLeft, m_settings.surfaceProxyRoiFrameOriginBottom,
			m_settings.surfaceProxyRoiFrameWidthPhysical, m_settings.surfaceProxyRoiFrameHeightPhysical
		}
	);
	appendProxyRoi(
		m_settings.surfaceProxyRoi2Left,
		m_settings.surfaceProxyRoi2Top,
		m_settings.surfaceProxyRoi2Width,
		m_settings.surfaceProxyRoi2Height,
		PROXY_ROI_FRAME{
			m_settings.surfaceProxyRoi2FrameWidth, m_settings.surfaceProxyRoi2FrameHeight,
			m_settings.surfaceProxyRoi2FrameOriginLeft, m_settings.surfaceProxyRoi2FrameOriginBottom,
			m_settings.surfaceProxyRoi2FrameWidthPhysical, m_settings.surfaceProxyRoi2FrameHeightPhysical
		}
	);
	if (metrics.empty()) {
		appendMetric(0, 0, width, height);
	}

	double metricSum{ 0.0 };
	for (const auto metric : metrics) {
		metricSum += metric;
	}
	return metrics.empty() ? 0.0 : metricSum / metrics.size();
}

std::pair<std::vector<double>, std::vector<double>> Brillouin::coarseXYSamples(int bin) const {
	const auto xyBin = std::max(1, bin);
	// Every `bin`-th index of the real, dense grid - not an independent re-interpolation
	// (see the header comment on this function for why that used to be able to place a
	// coarse point where no real measurement point would be).
	auto pickEveryNth = [xyBin](double lo, double hi, int steps) {
		auto dense = simplemath::linspace(lo, hi, std::max(1, steps));
		std::vector<double> coarse;
		for (size_t i = 0; i < dense.size(); i += xyBin) {
			coarse.push_back(dense[i]);
		}
		// Always include the last real point, even when it doesn't fall on a bin-multiple
		// index, so the far edge of the grid is never silently dropped from the coarse set.
		if (!dense.empty() && (coarse.empty() || coarse.back() != dense.back())) {
			coarse.push_back(dense.back());
		}
		return coarse;
	};
	auto xSamples = pickEveryNth(m_settings.xMin, m_settings.xMax, m_settings.xSteps);
	auto ySamples = pickEveryNth(m_settings.yMin, m_settings.yMax, m_settings.ySteps);
	return { std::move(xSamples), std::move(ySamples) };
}

std::vector<POINT2> Brillouin::additionalBoundaryXYPoints(int count) const {
	if (count <= 0) {
		return {};
	}

	// The curve actually sampled: the ROI clipped to the grid rectangle, or the bare
	// rectangle when no ROI is set - see clipPolygonToRect()'s comment for why this, rather
	// than the ROI polygon itself, is what has to be walked.
	const std::vector<POINT2> rect = {
		{ m_settings.xMin, m_settings.yMin }, { m_settings.xMax, m_settings.yMin },
		{ m_settings.xMax, m_settings.yMax }, { m_settings.xMin, m_settings.yMax }
	};
	auto boundary = rect;
	if (m_settings.useRoiMask && m_settings.roiPolygonUm.size() >= 3) {
		auto clipped = clipPolygonToRect(m_settings.roiPolygonUm, m_settings.xMin, m_settings.xMax, m_settings.yMin, m_settings.yMax);
		if (clipped.size() >= 3) {
			boundary = std::move(clipped);
		}
	}

	// The uniform coarse grid's own kept anchors (plan frame, same as xSamples/ySamples) -
	// farthest-point selection is evaluated against these, so a boundary point never lands
	// right next to interior coverage that's already there.
	const auto [xSamples, ySamples] = coarseXYSamples(m_settings.preScanXYBin);
	std::vector<POINT2> uniformAnchors;
	uniformAnchors.reserve(xSamples.size() * ySamples.size());
	for (const auto y : ySamples) {
		for (const auto x : xSamples) {
			if (m_settings.useRoiMask && !isPointInPolygonUm(POINT2{ x, y }, m_settings.roiPolygonUm)) {
				continue;
			}
			uniformAnchors.push_back(POINT2{ x, y });
		}
	}

	// Fine candidate pool along the (clipped) outline - enough resolution for the
	// farthest-point walk to actually spread `count` points out, capped so a large request
	// doesn't blow up the (candidates x reference-points) search below.
	const auto candidateCount = std::clamp(count * 12, 60, 400);
	const auto candidates = polygonPerimeterPoints(boundary, candidateCount);
	const auto idealPoints = farthestPointSequence(candidates, uniformAnchors, count);

	// Snap each ideal point to its nearest real grid index, independently per axis - same
	// "always land on a real measurement point" rule coarseXYSamples() follows.
	const auto denseX = simplemath::linspace(m_settings.xMin, m_settings.xMax, std::max(1, m_settings.xSteps));
	const auto denseY = simplemath::linspace(m_settings.yMin, m_settings.yMax, std::max(1, m_settings.ySteps));
	auto nearestValue = [](double v, const std::vector<double>& values) {
		auto best = values.front();
		auto bestD = std::numeric_limits<double>::infinity();
		for (const auto value : values) {
			const auto d = std::abs(value - v);
			if (d < bestD) {
				bestD = d;
				best = value;
			}
		}
		return best;
	};

	// Candidates that snap onto an index already used - by the uniform grid, or by an
	// earlier boundary pick this same call - are skipped rather than duplicated, which is
	// why the result can have fewer than `count` entries.
	std::set<std::pair<double, double>> usedIndices;
	for (const auto& anchor : uniformAnchors) {
		usedIndices.insert({ anchor.x, anchor.y });
	}
	std::vector<POINT2> result;
	for (const auto& ideal : idealPoints) {
		const POINT2 snapped{ nearestValue(ideal.x, denseX), nearestValue(ideal.y, denseY) };
		const auto key = std::make_pair(snapped.x, snapped.y);
		if (usedIndices.count(key)) {
			continue;
		}
		usedIndices.insert(key);
		result.push_back(snapped);
	}
	return result;
}

std::optional<double> Brillouin::measureBoundarySurfaceZ(
	POINT2 xyPlan, double seedZRel, double zTravel, double zStep, double referenceThreshold
) {
	// Mirrors searchColumn()'s algorithm (local to runSurfacePreScan()) exactly - seed,
	// rewind if already past the interface, forward search, verification window with a
	// trend check - just parameterized by a raw (x, y) instead of a coarse-grid (xi, yi)
	// index, since boundary points don't have a slot in that grid to begin with. Kept as its
	// own copy rather than refactoring searchColumn() itself to share it, to avoid touching
	// that already-tuned logic for the sake of a point kind it wasn't written to support.
	auto frame = std::vector<std::byte>(m_settings.camera.roi.bytesPerFrame);
	auto measureHere = [&](double zRel, int frameAverage) -> std::optional<double> {
		if (m_abort) {
			return std::nullopt;
		}
		const auto gridOrigin = resolvedGridOriginUm();
		const auto xyPosition = m_settings.gridCoordinatesAbsolute
			? POINT2{ xyPlan.x + gridOrigin.x, xyPlan.y + gridOrigin.y }
			: POINT2{ m_startPosition.x + xyPlan.x, m_startPosition.y + xyPlan.y };
		const auto zOrigin = m_settings.gridCoordinatesAbsolute
			? gridOrigin.z
			: m_startPosition.z;
		const auto target = POINT3{ xyPosition.x, xyPosition.y, zOrigin + zRel };
		approachGridPosition(target);
		const auto frames = std::max(1, frameAverage);
		double sum = 0.0;
		for (int f = 0; f < frames; f++) {
			if (m_abort) {
				return std::nullopt;
			}
			m_andor->getImageForAcquisition(frame.data());
			sum += estimateFrameMetric(frame);
		}
		return sum / frames;
	};

	auto zRel = std::clamp(seedZRel, 0.0, zTravel);
	auto metric = measureHere(zRel, 1);
	if (!metric) {
		return std::nullopt;
	}

	auto rewound = 0.0;
	while (std::isfinite(referenceThreshold) && *metric <= referenceThreshold) {
		zRel -= zStep;
		rewound += zStep;
		if (rewound > m_settings.surfaceMaxRewindUm || zRel < 0.0) {
			return std::nullopt;
		}
		metric = measureHere(zRel, 1);
		if (!metric) {
			return std::nullopt;
		}
	}

	while (zRel <= zTravel) {
		if (*metric <= referenceThreshold) {
			const auto candidateZ = zRel;
			std::vector<double> window;
			const auto candidateAvg = measureHere(candidateZ, m_settings.surfaceVerificationFrameAverage);
			if (!candidateAvg) {
				return std::nullopt;
			}
			window.push_back(*candidateAvg);

			auto verified = true;
			for (int k = 1; k <= std::max(0, m_settings.surfaceVerificationSteps); k++) {
				const auto zk = candidateZ + k * zStep;
				if (zk > zTravel) {
					verified = false;
					break;
				}
				const auto mk = measureHere(zk, m_settings.surfaceVerificationFrameAverage);
				if (!mk) {
					return std::nullopt;
				}
				window.push_back(*mk);
				if (*mk > referenceThreshold) {
					verified = false;
					break;
				}
			}

			if (verified && window.size() == (size_t)m_settings.surfaceVerificationSteps + 1) {
				const auto half = (window.size() + 1) / 2;
				const auto firstMean = std::accumulate(window.begin(), window.begin() + half, 0.0) / half;
				const auto secondCount = window.size() - half;
				const auto secondMean = secondCount > 0
					? std::accumulate(window.begin() + half, window.end(), 0.0) / secondCount
					: firstMean;
				if (secondMean <= firstMean * (1.0 + std::max(0.0, m_settings.surfaceVerificationToleranceFraction))) {
					return candidateZ;
				}
			}
			return std::nullopt;
		}
		zRel += zStep;
		if (zRel > zTravel) {
			break;
		}
		metric = measureHere(zRel, 1);
		if (!metric) {
			return std::nullopt;
		}
	}

	return std::nullopt;
}

POINT3 Brillouin::planPositionToGridFrame(const POINT3& planPosition) const {
	if (m_settings.gridCoordinatesAbsolute) {
		return planPosition;
	}
	return POINT3{
		planPosition.x + m_startPosition.x,
		planPosition.y + m_startPosition.y,
		planPosition.z + m_startPosition.z
	};
}

POINT3 Brillouin::rawPositionToGridFrame(const POINT3& rawPosition) const {
	if (m_settings.gridCoordinatesAbsolute) {
		const auto origin = resolvedGridOriginUm();
		return POINT3{
			rawPosition.x - origin.x,
			rawPosition.y - origin.y,
			rawPosition.z - origin.z
		};
	}
	return rawPosition;
}

POINT3 Brillouin::resolvedGridOriginUm() const {
	const auto offsetUm = m_scanControl ? m_scanControl->getActiveObjectiveFovOffsetUm() : POINT2{ 0, 0 };
	return POINT3{
		m_settings.absoluteGridOriginUm.x + offsetUm.x,
		m_settings.absoluteGridOriginUm.y + offsetUm.y,
		m_settings.absoluteGridOriginUm.z
	};
}

Brillouin::SurfaceScanResult Brillouin::runSurfacePreScan() {
	// Cleared up front so every exit path - including the early-out ones below and an
	// abort partway through - leaves these reflecting only a scan that actually completed,
	// never stale found/interpolated data from an earlier, unrelated run.
	m_surfaceFoundXYIndices.clear();
	m_surfaceInterpolatedXYIndices.clear();
	m_surfaceZRangeValid = false;
	m_surfacePreScanXUm.clear();
	m_surfacePreScanYUm.clear();
	m_surfacePreScanFoundMask.clear();
	m_surfacePreScanZUm.clear();
	m_surfacePreScanMetric.clear();
	m_surfaceBoundaryPointsUm.clear();
	m_surfaceReferenceThreshold = std::numeric_limits<double>::quiet_NaN();

	if (!m_scanControl || !m_andor) {
		return {};
	}

	const auto zTravel = std::max(0.0, m_settings.preScanZTravelRangeUm);
	const auto zStep = std::max(1e-6, m_settings.preScanZStepUm);
	const auto zStepsCoarse = std::max(2, (int)std::floor(zTravel / zStep) + 1);

	const auto [xSamples, ySamples] = coarseXYSamples(m_settings.preScanXYBin);

	if (xSamples.empty() || ySamples.empty() || zStepsCoarse < 2) {
		return {};
	}

	// Recorded now (rather than only after a successful scan) so the raw sample grid is
	// still available for diagnosing an aborted/failed pre-scan too.
	m_surfacePreScanXUm = xSamples;
	m_surfacePreScanYUm = ySamples;

	std::vector<std::vector<double>> zMetric(ySamples.size(),
		std::vector<double>(xSamples.size(), std::numeric_limits<double>::quiet_NaN()));
	std::vector<std::vector<double>> zSurface(ySamples.size(), std::vector<double>(xSamples.size(), 0.0));
	std::vector<std::vector<bool>> zSurfaceValid(ySamples.size(), std::vector<bool>(xSamples.size(), false));
	// Order each column was found in (-1 = not found yet), used to break seed-distance ties
	// in favor of whichever neighbor was measured most recently.
	std::vector<std::vector<int>> processOrder(ySamples.size(), std::vector<int>(xSamples.size(), -1));
	auto frame = std::vector<std::byte>(m_settings.camera.roi.bytesPerFrame);
	// Progress is based on columns (coarse xy points), not z-measurement steps: the
	// neighbor-seeded search below takes a variable, usually small, number of z-steps per
	// column once a few neighbors are found, nowhere near the worst-case full zStepsCoarse
	// sweep a column could take. Estimating progress from steps-so-far against that
	// worst-case denominator badly underestimates how far along the scan really is (e.g.
	// showing ~10% when nearly every column is already done). The column count, in
	// contrast, is known exactly up front and every column is visited exactly once.
	const auto totalColumnsExpected = std::max(1, (int)(xSamples.size() * ySamples.size()));
	int totalColumns = 0;
	int processCounter = 0;

	// Measure medium reference before scanning - this is the only threshold source for
	// surface detection, so it always runs (not user-optional).
	bool referencePositionFound = false;
	POINT3 referencePosition{ 0.0, 0.0, 0.0 };
	for (gsl::index yi{ 0 }; yi < (gsl::index)ySamples.size() && !referencePositionFound; yi++) {
		for (gsl::index xi{ 0 }; xi < (gsl::index)xSamples.size(); xi++) {
			const POINT2 coarsePoint{ xSamples[xi], ySamples[yi] };
			if (m_settings.useRoiMask && !isPointInPolygonUm(coarsePoint, m_settings.roiPolygonUm)) {
				continue;
			}
			const auto gridOrigin = resolvedGridOriginUm();
			const auto zOrigin = m_settings.gridCoordinatesAbsolute
				? gridOrigin.z
				: m_startPosition.z;
			referencePosition = m_settings.gridCoordinatesAbsolute
				? POINT3{ xSamples[xi] + gridOrigin.x, ySamples[yi] + gridOrigin.y, zOrigin }
				: POINT3{ m_startPosition.x + xSamples[xi], m_startPosition.y + ySamples[yi], zOrigin };
			referencePositionFound = true;
			break;
		}
	}
	if (referencePositionFound) {
		// Mirrors runMeasurementPhase()'s approach to its own first position exactly
		// (Brillouin.cpp: "move stage to first position, wait 50 ms for it to finish") -
		// this is the equivalent one-time first move here, before the per-column loop
		// below settles into the same move-with-no-extra-wait pattern that loop uses too.
		approachGridPosition(referencePosition);
		std::this_thread::sleep_for(std::chrono::milliseconds(50));
	}
	const int refFrames = std::max(1, m_settings.mediumReferenceFrameCount);
	double refSum = 0.0;
	for (int i = 0; i < refFrames; i++) {
		if (m_abort) {
			return {};
		}
		m_andor->getImageForAcquisition(frame.data());
		const auto refMetric = estimateFrameMetric(frame);
		refSum += refMetric;
		const auto refProgress = 5.0 * (double)(i + 1) / refFrames;
		emit(s_surfaceScanProgress(
			refProgress,
			QString("Surface reference %1/%2: metric %3")
				.arg(i + 1)
				.arg(refFrames)
				.arg(refMetric, 0, 'f', 3)
		));
	}
	m_settings.mediumReferenceValue = refSum / refFrames;
	emit(s_surfaceScanProgress(
		5.0,
		QString("Surface reference measured: %1")
			.arg(m_settings.mediumReferenceValue, 0, 'f', 3)
	));

	const auto dropFraction = std::clamp(m_settings.surfaceDropFraction, 0.0, 0.99);
	const auto referenceThreshold = (m_settings.mediumReferenceValue > 1e-12)
		? (1.0 - dropFraction) * m_settings.mediumReferenceValue
		: std::numeric_limits<double>::quiet_NaN();
	m_surfaceReferenceThreshold = referenceThreshold;

	// Measures the surface-drop metric at coarse column (xi, yi) and relative z `zRel`,
	// averaging `frameAverage` frames at that position to reduce noise (1 = single frame,
	// used for the fast initial walk; > 1 used during verification). Moves the stage there
	// first, via the exact same approachGridPosition() runMeasurementPhase()'s own
	// point-to-point moves use (Brillouin.cpp's "move stage to next position", which adds
	// no extra wait after the move either - see its comments) - setPositionCompensated()
	// only pre-approaches x/y when they actually change, so repeated calls within a column
	// (z-only steps) add no extra motion. Returns std::nullopt if the acquisition was
	// aborted mid-measurement.
	auto measureAt = [&](gsl::index xi, gsl::index yi, double zRel, int frameAverage) -> std::optional<double> {
		if (m_abort) {
			return std::nullopt;
		}
		const auto gridOrigin = resolvedGridOriginUm();
		const auto xyPosition = m_settings.gridCoordinatesAbsolute
			? POINT2{ xSamples[xi] + gridOrigin.x, ySamples[yi] + gridOrigin.y }
			: POINT2{ m_startPosition.x + xSamples[xi], m_startPosition.y + ySamples[yi] };
		const auto zOrigin = m_settings.gridCoordinatesAbsolute
			? gridOrigin.z
			: m_startPosition.z;
		const auto target = POINT3{ xyPosition.x, xyPosition.y, zOrigin + zRel };
		approachGridPosition(target);
		const auto frames = std::max(1, frameAverage);
		double sum = 0.0;
		for (int f = 0; f < frames; f++) {
			if (m_abort) {
				return std::nullopt;
			}
			m_andor->getImageForAcquisition(frame.data());
			sum += estimateFrameMetric(frame);
		}
		return sum / frames;
	};

	auto emitSurfaceProgress = [&](const QString& message) {
		const auto progress = std::clamp(5.0 + 95.0 * (double)totalColumns / totalColumnsExpected, 5.0, 99.9);
		emit(s_surfaceScanProgress(progress, message));
	};

	// Nearest already-found coarse neighbor to seed the search from, ties broken in favor
	// of whichever was found most recently (largest processOrder) - the "locally coplanar
	// tissue" assumption is only as good as the neighbor it's based on, and a just-measured
	// neighbor is the best available evidence. std::nullopt if nothing has been found yet
	// (first column), in which case the caller starts from z = 0 like the legacy sweep did.
	auto findSeedZRel = [&](gsl::index xi, gsl::index yi) -> std::optional<double> {
		auto bestDist2 = std::numeric_limits<double>::infinity();
		auto bestOrder = -1;
		auto bestZ = 0.0;
		auto found = false;
		for (gsl::index yc{ 0 }; yc < (gsl::index)ySamples.size(); yc++) {
			for (gsl::index xc{ 0 }; xc < (gsl::index)xSamples.size(); xc++) {
				if (!zSurfaceValid[yc][xc]) {
					continue;
				}
				const auto dx = xSamples[xc] - xSamples[xi];
				const auto dy = ySamples[yc] - ySamples[yi];
				const auto d2 = dx * dx + dy * dy;
				const auto tol = 1e-6 * std::max(1.0, bestDist2);
				const auto isCloser = d2 < bestDist2 - tol;
				const auto isTie = std::abs(d2 - bestDist2) <= tol;
				if (isCloser || (isTie && processOrder[yc][xc] > bestOrder)) {
					bestDist2 = d2;
					bestOrder = processOrder[yc][xc];
					bestZ = zSurface[yc][xc];
					found = true;
				}
			}
		}
		return found ? std::optional<double>(bestZ) : std::nullopt;
	};

	// Searches one coarse column for the surface, seeded from the nearest already-found
	// neighbor instead of always starting at z = 0. If the seed is already past the
	// interface (metric at/below threshold), first "rewinds" back towards water until
	// above threshold again (bounded by surfaceMaxRewindUm - if exceeded, or z would go
	// negative, the column is marked not found, same as the legacy "no drop found" case).
	// From there it runs the same forward search a column with no seed would. A candidate
	// crossing is only accepted once surfaceVerificationSteps further points (each
	// optionally averaged over surfaceVerificationFrameAverage frames to cut noise) confirm
	// the signal stays down and doesn't trend back up, matching the expected monotonic drop
	// from water into and through tissue - otherwise the whole column is treated as not
	// found, exactly like one that never crosses at all (deliberately not retried deeper,
	// to keep this simple). Returns false if aborted (caller bails out immediately).
	auto searchColumn = [&](gsl::index xi, gsl::index yi) -> bool {
		totalColumns++;
		const auto seed = findSeedZRel(xi, yi).value_or(0.0);
		auto zRel = std::clamp(seed, 0.0, zTravel);

		auto metric = measureAt(xi, yi, zRel, 1);
		if (!metric) {
			return false;
		}
		zMetric[yi][xi] = *metric;
		emitSurfaceProgress(QString("Surface scan: x %1/%2, y %3/%4, seeded z %5 um, metric %6, threshold %7")
			.arg((int)xi + 1).arg((int)xSamples.size())
			.arg((int)yi + 1).arg((int)ySamples.size())
			.arg(zRel, 0, 'f', 1).arg(*metric, 0, 'f', 3).arg(referenceThreshold, 0, 'f', 3));

		// Rewind phase.
		auto rewound = 0.0;
		while (std::isfinite(referenceThreshold) && *metric <= referenceThreshold) {
			zRel -= zStep;
			rewound += zStep;
			if (rewound > m_settings.surfaceMaxRewindUm || zRel < 0.0) {
				emitSurfaceProgress(QString("No surface found at x %1/%2, y %3/%4 (rewind limit reached)")
					.arg((int)xi + 1).arg((int)xSamples.size())
					.arg((int)yi + 1).arg((int)ySamples.size()));
				return true;
			}
			metric = measureAt(xi, yi, zRel, 1);
			if (!metric) {
				return false;
			}
			zMetric[yi][xi] = *metric;
			emitSurfaceProgress(QString("Surface scan (rewind): x %1/%2, y %3/%4, z %5 um, metric %6, threshold %7")
				.arg((int)xi + 1).arg((int)xSamples.size())
				.arg((int)yi + 1).arg((int)ySamples.size())
				.arg(zRel, 0, 'f', 1).arg(*metric, 0, 'f', 3).arg(referenceThreshold, 0, 'f', 3));
		}

		// Forward search - metric is guaranteed above threshold here (or undefined, which
		// can't happen since the medium reference measurement above always runs).
		while (zRel <= zTravel) {
			if (*metric <= referenceThreshold) {
				const auto candidateZ = zRel;
				std::vector<double> window;
				const auto candidateAvg = measureAt(xi, yi, candidateZ, m_settings.surfaceVerificationFrameAverage);
				if (!candidateAvg) {
					return false;
				}
				zMetric[yi][xi] = *candidateAvg;
				window.push_back(*candidateAvg);

				auto verified = true;
				for (int k = 1; k <= std::max(0, m_settings.surfaceVerificationSteps); k++) {
					const auto zk = candidateZ + k * zStep;
					if (zk > zTravel) {
						verified = false;
						break;
					}
					const auto mk = measureAt(xi, yi, zk, m_settings.surfaceVerificationFrameAverage);
					if (!mk) {
						return false;
					}
					zMetric[yi][xi] = *mk;
					emitSurfaceProgress(QString("Surface verification %1/%2 at x %3/%4, y %5/%6: metric %7, threshold %8")
						.arg(k).arg(m_settings.surfaceVerificationSteps)
						.arg((int)xi + 1).arg((int)xSamples.size())
						.arg((int)yi + 1).arg((int)ySamples.size())
						.arg(*mk, 0, 'f', 3).arg(referenceThreshold, 0, 'f', 3));
					window.push_back(*mk);
					if (*mk > referenceThreshold) {
						verified = false;
						break;
					}
				}

				if (verified && window.size() == (size_t)m_settings.surfaceVerificationSteps + 1) {
					// Trend check on the (frame-averaged) window: the mean of the second
					// half must not rise measurably above the first half's, matching the
					// expected monotonic drop from water into and through tissue rather
					// than a transient noise dip that partially recovers.
					const auto half = (window.size() + 1) / 2;
					const auto firstMean = std::accumulate(window.begin(), window.begin() + half, 0.0) / half;
					const auto secondCount = window.size() - half;
					const auto secondMean = secondCount > 0
						? std::accumulate(window.begin() + half, window.end(), 0.0) / secondCount
						: firstMean;
					if (secondMean <= firstMean * (1.0 + std::max(0.0, m_settings.surfaceVerificationToleranceFraction))) {
						zSurface[yi][xi] = candidateZ;
						zSurfaceValid[yi][xi] = true;
						processOrder[yi][xi] = processCounter++;
						// The metric at the found surface itself, not whatever the
						// verification loop above last measured deeper into the tissue.
						zMetric[yi][xi] = *candidateAvg;
						emitSurfaceProgress(QString("Surface found at x %1/%2, y %3/%4, z %5 um: metric %6 <= threshold %7, verified")
							.arg((int)xi + 1).arg((int)xSamples.size())
							.arg((int)yi + 1).arg((int)ySamples.size())
							.arg(candidateZ, 0, 'f', 1).arg(*candidateAvg, 0, 'f', 3).arg(referenceThreshold, 0, 'f', 3));
						return true;
					}
				}

				emitSurfaceProgress(QString("No surface found at x %1/%2, y %3/%4 (candidate at z %5 um failed verification)")
					.arg((int)xi + 1).arg((int)xSamples.size())
					.arg((int)yi + 1).arg((int)ySamples.size())
					.arg(candidateZ, 0, 'f', 1));
				return true;
			}
			zRel += zStep;
			if (zRel > zTravel) {
				break;
			}
			metric = measureAt(xi, yi, zRel, 1);
			if (!metric) {
				return false;
			}
			zMetric[yi][xi] = *metric;
			emitSurfaceProgress(QString("Surface scan: x %1/%2, y %3/%4, z %5 um, metric %6, threshold %7")
				.arg((int)xi + 1).arg((int)xSamples.size())
				.arg((int)yi + 1).arg((int)ySamples.size())
				.arg(zRel, 0, 'f', 1).arg(*metric, 0, 'f', 3).arg(referenceThreshold, 0, 'f', 3));
		}

		emitSurfaceProgress(QString("No surface found at x %1/%2, y %3/%4 (no drop within range)")
			.arg((int)xi + 1).arg((int)xSamples.size())
			.arg((int)yi + 1).arg((int)ySamples.size()));
		return true;
	};

	// Serpentine (boustrophedon) traversal: alternate x-direction every row so consecutive
	// columns are always spatially adjacent - minimizes stage travel and keeps the
	// neighbor-seeded search's "locally coplanar" assumption meaningful (the previous
	// column really is next to the new one, not on the opposite side of the grid).
	for (gsl::index yi{ 0 }; yi < (gsl::index)ySamples.size(); yi++) {
		const auto reverseRow = (yi % 2) == 1;
		for (gsl::index xiRaw{ 0 }; xiRaw < (gsl::index)xSamples.size(); xiRaw++) {
			const gsl::index xi = reverseRow ? ((gsl::index)xSamples.size() - 1 - xiRaw) : xiRaw;
			const POINT2 coarsePoint{ xSamples[xi], ySamples[yi] };
			if (m_settings.useRoiMask && !isPointInPolygonUm(coarsePoint, m_settings.roiPolygonUm)) {
				continue;
			}
			if (!searchColumn(xi, yi)) {
				return {};
			}
		}
	}

	// Smooth coarse surface map with a small local mean filter. Only ever averages over
	// zSurfaceValid neighbors - an invalid (never-found) cell defaults to 0.0, which
	// otherwise silently pulled a genuine neighbor's smoothed value towards 0 rather than
	// its actual measured height, exactly at grid edges/gaps where smoothing matters most.
	// Invalid cells themselves are left untouched here (irrelevant until the gap-fill pass
	// below, which reads only zSurfaceGenuine neighbors, never an unfound cell's z).
	const auto smoothingPasses = std::max(0, (int)std::round(m_settings.surfaceSmoothSigmaUm / 5.0));
	for (int pass = 0; pass < smoothingPasses; pass++) {
		auto smoothed = zSurface;
		for (gsl::index yi{ 0 }; yi < (gsl::index)zSurface.size(); yi++) {
			for (gsl::index xi{ 0 }; xi < (gsl::index)zSurface[yi].size(); xi++) {
				if (!zSurfaceValid[yi][xi]) {
					continue;
				}
				double sum = 0.0;
				int count = 0;
				for (int dy = -1; dy <= 1; dy++) {
					for (int dx = -1; dx <= 1; dx++) {
						const auto ny = yi + dy;
						const auto nx = xi + dx;
						if (ny >= 0 && ny < (gsl::index)zSurface.size()
							&& nx >= 0 && nx < (gsl::index)zSurface[yi].size()
							&& zSurfaceValid[ny][nx]) {
							sum += zSurface[ny][nx];
							count++;
						}
					}
				}
				smoothed[yi][xi] = count > 0 ? sum / count : zSurface[yi][xi];
			}
		}
		zSurface = std::move(smoothed);
	}

	// Gap-fill, done only now (at the very end): a coarse column with no found surface
	// gets one, averaged from its direct (4-connected, not diagonal) neighbors, but only
	// if at least 2 of them have a GENUINE surface - i.e. this check only ever looks at
	// zSurfaceGenuine (a snapshot taken before this pass writes anything), never at another
	// cell this same pass just filled. That means a bad fill can never cascade into
	// filling further cells from it, and the order cells happen to be visited in here
	// can't change the result.
	const auto zSurfaceGenuine = zSurfaceValid;
	auto zSurfaceInterpolatedCoarse = std::vector<std::vector<bool>>(ySamples.size(), std::vector<bool>(xSamples.size(), false));
	for (gsl::index yi{ 0 }; yi < (gsl::index)ySamples.size(); yi++) {
		for (gsl::index xi{ 0 }; xi < (gsl::index)xSamples.size(); xi++) {
			if (zSurfaceGenuine[yi][xi]) {
				continue;
			}
			std::vector<double> neighborZ;
			if (yi > 0 && zSurfaceGenuine[yi - 1][xi]) {
				neighborZ.push_back(zSurface[yi - 1][xi]);
			}
			if (yi + 1 < (gsl::index)ySamples.size() && zSurfaceGenuine[yi + 1][xi]) {
				neighborZ.push_back(zSurface[yi + 1][xi]);
			}
			if (xi > 0 && zSurfaceGenuine[yi][xi - 1]) {
				neighborZ.push_back(zSurface[yi][xi - 1]);
			}
			if (xi + 1 < (gsl::index)xSamples.size() && zSurfaceGenuine[yi][xi + 1]) {
				neighborZ.push_back(zSurface[yi][xi + 1]);
			}
			if (neighborZ.size() >= 2) {
				zSurface[yi][xi] = std::accumulate(neighborZ.begin(), neighborZ.end(), 0.0) / neighborZ.size();
				zSurfaceInterpolatedCoarse[yi][xi] = true;
			}
		}
	}

	// Raw coarse pre-scan results, flat-indexed xi * ySamples.size() + yi (mirroring
	// surface-found-mask's own x-major flat indexing) - see the m_surfacePreScanFoundMask
	// et al. declarations for what each array means. Recorded for every coarse cell, not
	// just ones inside the ROI mask, so a reader can tell "outside the ROI" (found mask
	// would read 0 here too, same as a genuine failure) apart by checking their own
	// (x, y) against the ROI polygon/roi-scan-plan-mask - consistent with how the dense
	// surface-found-mask already works.
	m_surfacePreScanFoundMask.assign(xSamples.size() * ySamples.size(), 0.0);
	m_surfacePreScanZUm.assign(xSamples.size() * ySamples.size(), 0.0);
	m_surfacePreScanMetric.assign(
		xSamples.size() * ySamples.size(), std::numeric_limits<double>::quiet_NaN());
	for (gsl::index yi{ 0 }; yi < (gsl::index)ySamples.size(); yi++) {
		for (gsl::index xi{ 0 }; xi < (gsl::index)xSamples.size(); xi++) {
			const auto flat = xi * (gsl::index)ySamples.size() + yi;
			m_surfacePreScanFoundMask[flat] = zSurfaceGenuine[yi][xi]
				? 1.0 : (zSurfaceInterpolatedCoarse[yi][xi] ? 2.0 : 0.0);
			m_surfacePreScanZUm[flat] = zSurface[yi][xi];
			m_surfacePreScanMetric[flat] = zMetric[yi][xi];
		}
	}

	// Final, authoritative "no surface" count - after gap-fill, not the running total shown
	// live during the scan above (which counts columns that were rescued by gap-fill too).
	auto failedColumns = 0;
	for (gsl::index yi{ 0 }; yi < (gsl::index)ySamples.size(); yi++) {
		for (gsl::index xi{ 0 }; xi < (gsl::index)xSamples.size(); xi++) {
			const POINT2 coarsePoint{ xSamples[xi], ySamples[yi] };
			if (m_settings.useRoiMask && !isPointInPolygonUm(coarsePoint, m_settings.roiPolygonUm)) {
				continue;
			}
			if (!zSurfaceGenuine[yi][xi] && !zSurfaceInterpolatedCoarse[yi][xi]) {
				failedColumns++;
			}
		}
	}

	// Additional boundary anchor points (opt-in, m_settings.additionalBoundaryPoints) - not
	// part of the rectangular coarse grid at all, so they get their own small measurement
	// pass here rather than a slot in zSurface[][], seeded from the rectangular grid's best
	// available data (genuine or gap-filled) and from each other, using the exact same
	// seed/rewind/verify algorithm searchColumn() uses (see measureBoundarySurfaceZ()).
	// Folded into the dense-grid interpolation below as extra weighted neighbors, alongside -
	// not instead of - the untouched rectangular grid above.
	const auto boundaryXY = additionalBoundaryXYPoints(m_settings.additionalBoundaryPoints);
	if (!boundaryXY.empty()) {
		std::vector<POINT3> referencePoints;
		for (gsl::index yc{ 0 }; yc < (gsl::index)ySamples.size(); yc++) {
			for (gsl::index xc{ 0 }; xc < (gsl::index)xSamples.size(); xc++) {
				if (zSurfaceGenuine[yc][xc] || zSurfaceInterpolatedCoarse[yc][xc]) {
					referencePoints.push_back(POINT3{ xSamples[xc], ySamples[yc], zSurface[yc][xc] });
				}
			}
		}
		int boundaryDone = 0;
		for (const auto& xy : boundaryXY) {
			if (m_abort) {
				return {};
			}
			// Nearest already-known point (rectangular grid, or an earlier boundary point
			// this same loop already found) seeds the search - same "locally coplanar"
			// reasoning findSeedZRel() uses for the rectangular grid.
			auto seedZ = 0.0;
			auto bestDist2 = std::numeric_limits<double>::infinity();
			for (const auto& ref : referencePoints) {
				const auto dx = ref.x - xy.x;
				const auto dy = ref.y - xy.y;
				const auto d2 = dx * dx + dy * dy;
				if (d2 < bestDist2) {
					bestDist2 = d2;
					seedZ = ref.z;
				}
			}
			boundaryDone++;
			emitSurfaceProgress(QString("Boundary point %1/%2").arg(boundaryDone).arg((int)boundaryXY.size()));
			const auto found = measureBoundarySurfaceZ(xy, seedZ, zTravel, zStep, referenceThreshold);
			if (found) {
				m_surfaceBoundaryPointsUm.push_back(POINT3{ xy.x, xy.y, *found });
				referencePoints.push_back(POINT3{ xy.x, xy.y, *found });
			}
		}
	}

	// Bilinear interpolation on coarse XY map; nearest sample on boundaries. Both
	// genuinely-found and gap-filled coarse cells count as data here.
	const auto xDense = simplemath::linspace(m_settings.xMin, m_settings.xMax, m_settings.xSteps);
	const auto yDense = simplemath::linspace(m_settings.yMin, m_settings.yMax, m_settings.ySteps);
	std::map<std::pair<int, int>, double> zCenterByXYIndex;
	std::set<std::pair<int, int>> interpolatedXYIndices;

	// This is an O(dense grid x coarse grid) pass - for a fine dense grid it can take a
	// while, and previously ran with no progress feedback and no abort check at all, which
	// made the UI look hung right after the coarse scan finished (status bar just stopped
	// updating) with no way to cancel out of it. Reported on the same channel as the coarse
	// scan above, so it shows up in the same place instead of looking like a stall.
	for (gsl::index yi{ 0 }; yi < (gsl::index)yDense.size(); yi++) {
		if (m_abort) {
			return {};
		}
		const auto interpolationProgress = std::clamp(
			99.9 * (double)(yi + 1) / std::max((gsl::index)1, (gsl::index)yDense.size()),
			0.0, 99.9
		);
		emit(s_surfaceScanProgress(
			interpolationProgress,
			QString("Interpolating surface: row %1/%2").arg(yi + 1).arg(yDense.size())
		));
		for (gsl::index xi{ 0 }; xi < (gsl::index)xDense.size(); xi++) {
			auto x = xDense[xi];
			auto y = yDense[yi];

			if (m_settings.useRoiMask && !isPointInPolygonUm(POINT2{ x, y }, m_settings.roiPolygonUm)) {
				continue;
			}

			// Robust interpolation for ROI-masked coarse maps:
			// use inverse-distance weighting over valid coarse samples.
			double weightedSum = 0.0;
			double weightNorm = 0.0;
			bool exactMatch = false;
			double exactZ = 0.0;
			// Whether any coarse cell contributing to this dense point came from the
			// gap-fill pass rather than a genuine measurement - tracked so
			// getSurfaceInterpolatedXYIndices() can tell a purely-measured dense point
			// apart from one that leans on an interpolated neighbor.
			bool usedGapFilledContribution = false;
			for (gsl::index yc{ 0 }; yc < (gsl::index)ySamples.size(); yc++) {
				for (gsl::index xc{ 0 }; xc < (gsl::index)xSamples.size(); xc++) {
					if (!zSurfaceGenuine[yc][xc] && !zSurfaceInterpolatedCoarse[yc][xc]) {
						continue;
					}
					const auto dx = x - xSamples[xc];
					const auto dy = y - ySamples[yc];
					const auto d2 = dx * dx + dy * dy;
					if (d2 <= 1e-12) {
						exactMatch = true;
						exactZ = zSurface[yc][xc];
						usedGapFilledContribution = zSurfaceInterpolatedCoarse[yc][xc];
						break;
					}
					if (zSurfaceInterpolatedCoarse[yc][xc]) {
						usedGapFilledContribution = true;
					}
					const auto w = 1.0 / d2;
					weightedSum += w * zSurface[yc][xc];
					weightNorm += w;
				}
				if (exactMatch) {
					break;
				}
			}
			// Additional boundary points (if any - see measureBoundarySurfaceZ() above)
			// contribute the same way, as flat extra neighbors rather than (xc, yc) grid
			// cells - always treated as "genuine" since they have no gap-fill concept of
			// their own.
			if (!exactMatch) {
				for (const auto& boundaryPoint : m_surfaceBoundaryPointsUm) {
					const auto dx = x - boundaryPoint.x;
					const auto dy = y - boundaryPoint.y;
					const auto d2 = dx * dx + dy * dy;
					if (d2 <= 1e-12) {
						exactMatch = true;
						exactZ = boundaryPoint.z;
						break;
					}
					const auto w = 1.0 / d2;
					weightedSum += w * boundaryPoint.z;
					weightNorm += w;
				}
			}
			if (!exactMatch && weightNorm <= 0.0) {
				continue;
			}
			const auto zInterp = exactMatch ? exactZ : (weightedSum / weightNorm);

			const auto zOrigin = m_settings.gridCoordinatesAbsolute
				? resolvedGridOriginUm().z
				: m_startPosition.z;
			const auto centerZAbs = zOrigin + zInterp + m_settings.surfaceZOffsetUm;
			zCenterByXYIndex[{ (int)xi, (int)yi }] = centerZAbs;
			if (usedGapFilledContribution) {
				interpolatedXYIndices.insert({ (int)xi, (int)yi });
			}
		}
	}

	// Full set of (x, y) scan-plan index pairs that ended up with a surface z value here
	// (found directly or via coarse/dense interpolation) - exposed via
	// getSurfaceFoundXYIndices() for the GUI to distinguish these from points with none.
	m_surfaceFoundXYIndices.clear();
	for (const auto& entry : zCenterByXYIndex) {
		m_surfaceFoundXYIndices.insert(entry.first);
	}
	// Subset of the above that leans on at least one gap-filled coarse cell, as opposed to
	// being interpolated purely from genuinely-measured ones - see
	// getSurfaceInterpolatedXYIndices().
	m_surfaceInterpolatedXYIndices = std::move(interpolatedXYIndices);

	// Global min/max of the found surface (same absolute convention as centerZAbs above) -
	// used by overviewStackZAbs() to build a full-stack z range that covers every xy
	// tile's surface, not just whichever neighbor happens to be closest.
	m_surfaceZRangeValid = !zCenterByXYIndex.empty();
	if (m_surfaceZRangeValid) {
		auto minMax = std::minmax_element(
			zCenterByXYIndex.begin(), zCenterByXYIndex.end(),
			[](const auto& a, const auto& b) { return a.second < b.second; }
		);
		m_surfaceZMinAbs = minMax.first->second;
		m_surfaceZMaxAbs = minMax.second->second;
	}

	// zOrigin here must match the one used above to compute centerZAbs (zOrigin + zInterp +
	// surfaceZOffsetUm) - it's also exactly what m_orderedPositions[ll].z was built from
	// before this loop touches it (zOrigin + the grid's own zMin..zMax offset for this
	// z-index), so subtracting it back out recovers that same, still user-configured,
	// zMin..zMax-relative offset. Re-adding it onto the found surface below is what makes
	// "surface found at 100, zMin/zMax -10/20" actually scan 90..120: the surface takes
	// over the role zOrigin used to play, with the offset itself left untouched. Note this
	// intentionally no longer derives the range from the separate, UI-inaccessible
	// surfaceFollowHalfRangeUm field (always-symmetric around the surface and defaulted to
	// +/-10 um regardless of the grid's own zMin/zMax) - that silently ignored zMin/zMax
	// whenever surface follow was on.
	const auto zOrigin = m_settings.gridCoordinatesAbsolute
		? resolvedGridOriginUm().z
		: m_startPosition.z;

	for (gsl::index ll{ 0 }; ll < (gsl::index)m_orderedPositions.size(); ll++) {
		const auto ix = m_orderedIndices[ll].x;
		const auto iy = m_orderedIndices[ll].y;
		const auto it = zCenterByXYIndex.find({ ix, iy });
		if (it == zCenterByXYIndex.end()) {
			continue;
		}
		const auto zLocal = m_orderedPositions[ll].z - zOrigin;
		const auto zAbs = it->second + zLocal;
		m_orderedPositions[ll].z = zAbs;
		m_orderedPositionsRelative[ll].z = zAbs - m_startPosition.z;
	}

	return SurfaceScanResult{ true, totalColumns, failedColumns };
}

void Brillouin::applySurfaceFollowPlan() {
	if (!m_settings.useSurfaceFollow) {
		// Otherwise these could still hold results from an earlier repetition/session
		// where surface follow was on, misleadingly surviving into a run where it's off.
		m_surfaceFoundXYIndices.clear();
		m_surfaceInterpolatedXYIndices.clear();
		m_surfaceZRangeValid = false;
		return;
	}
	const auto result = runSurfacePreScan();
	if (result.success) {
		if (m_settings.gridCoordinatesAbsolute) {
			emit(s_orderedPositionsChanged(m_orderedPositions));
		} else {
			emit(s_orderedPositionsChanged(m_orderedPositionsRelative));
		}
		// Left at 100% (no further scan-progress emits follow) so this stays visible in
		// the status bar rather than being immediately overwritten by the next column's
		// transient progress message, unlike the per-column "No surface found" emits above.
		const auto message = result.failedColumns > 0
			? QString("Surface scan finished: no surface found for %1 of %2 columns.")
				.arg(result.failedColumns)
				.arg(result.totalColumns)
			: QString("Surface scan finished: surface found for all %1 columns.")
				.arg(result.totalColumns);
		emit(s_surfaceScanProgress(100.0, message));
	}
}

/*
 * Flat plan z for this z-index - origin.z + directionsZ[zIndex], always, regardless of
 * surface-follow state or grid coordinate mode. This used to be overridden by whichever
 * already-measured neighbor was closest in xy when surface-follow was on, using that
 * neighbor's surface-corrected z - which is exactly why the overview z looked "random":
 * it tracked one arbitrary neighbor instead of the plane. Surface-tracking for the
 * overview image is now expressed exclusively through overviewStackZAbs()'s full-stack
 * mode; "sampled grid points" always uses this flat value directly, never a stack.
 */
double Brillouin::overviewFlatZAbs(int zIndex, const std::vector<double>& directionsZ) const {
	const auto origin = m_settings.gridCoordinatesAbsolute ? resolvedGridOriginUm() : m_startPosition;
	const auto clampedZIndex = std::clamp(zIndex, 0, (int)directionsZ.size() - 1);
	return origin.z + directionsZ[clampedZIndex];
}

/*
 * xy point(s) for the overview image itself: the true grid center (single image), or one
 * per mosaic tile when overviewBrightfieldFullGrid is enabled (covering the whole grid
 * extent) - NOT "sampled grid points", which is an independent, additive option handled by
 * overviewCapturePoints(). Works in both absolute and relative grid mode, since
 * overviewTileCentersXY()/overviewGridCenterXY() both return xy in the same frame: origin-
 * inclusive (absolute stage um) for absolute mode, a pure grid offset with no origin baked
 * in for relative mode (see overviewTileCentersXY()'s own comments).
 */
std::vector<POINT2> Brillouin::overviewImageXY() const {
	if (m_settings.overviewBrightfieldFullGrid) {
		return overviewTileCentersXY();
	}
	return { overviewGridCenterXY() };
}

/*
 * z-targets to capture at the overview image's xy point(s) for this z-index. With
 * overviewBrightfieldFullStack off, this is just the single flat plan z (legacy behaviour,
 * 1 image per z-plane). With it on, this is zSteps values spanning either the grid's own
 * zMin..zMax (surface-follow off), or the global lowest-to-highest found surface offset by
 * zMin/zMax (surface-follow on) - e.g. lowest surface 100, highest 120, zMin/zMax -10/20 ->
 * stack from 90 to 140 at zSteps points. Falls back to the non-surface-follow range if
 * surface-follow is on but nothing was ever found, so this degrades gracefully instead of
 * using stale/default zero bounds.
 *
 * Only ever applies to the overview image (see overviewImageXY()) - "sampled grid points"
 * always captures a single flat image per point instead (see overviewCapturePoints()),
 * independent of this setting.
 */
std::vector<double> Brillouin::overviewStackZAbs(int zIndex, const std::vector<double>& directionsZ) const {
	if (!m_settings.overviewBrightfieldFullStack) {
		return { overviewFlatZAbs(zIndex, directionsZ) };
	}

	const auto origin = m_settings.gridCoordinatesAbsolute ? resolvedGridOriginUm() : m_startPosition;
	if (m_settings.useSurfaceFollow && m_surfaceZRangeValid) {
		return simplemath::linspace(
			m_surfaceZMinAbs + m_settings.zMin,
			m_surfaceZMaxAbs + m_settings.zMax,
			m_settings.zSteps
		);
	}

	return simplemath::linspace(origin.z + m_settings.zMin, origin.z + m_settings.zMax, m_settings.zSteps);
}

/*
 * Every image actually captured for this z-index: the overview image's xy point(s)
 * (overviewImageXY()) each paired with overviewStackZAbs() (a full stack, if enabled, only
 * ever applies here), followed by - additionally, independent of the overview image's own
 * settings - "sampled grid points" (overviewSampledGridXY()) if overviewBrightfieldSampledGrid
 * is on, each paired with a single flat overviewFlatZAbs() (never a stack, no matter how
 * many sampled points there are). The two groups are simply concatenated, not merged or
 * deduplicated, even if they happen to coincide in xy.
 */
std::vector<Brillouin::OverviewCapturePoint> Brillouin::overviewCapturePoints(int zIndex, const std::vector<double>& directionsZ) const {
	const auto xyOrigin = m_settings.gridCoordinatesAbsolute
		? POINT2{ 0, 0 }
		: POINT2{ m_startPosition.x, m_startPosition.y };

	std::vector<OverviewCapturePoint> points;

	const auto stackZ = overviewStackZAbs(zIndex, directionsZ);
	for (const auto& xy : overviewImageXY()) {
		points.push_back(OverviewCapturePoint{ POINT2{ xy.x + xyOrigin.x, xy.y + xyOrigin.y }, stackZ });
	}

	if (m_settings.overviewBrightfieldSampledGrid) {
		const std::vector<double> flatZ{ overviewFlatZAbs(zIndex, directionsZ) };
		for (const auto& xy : overviewSampledGridXY()) {
			points.push_back(OverviewCapturePoint{ POINT2{ xy.x + xyOrigin.x, xy.y + xyOrigin.y }, flatZ });
		}
	}

	return points;
}

int Brillouin::overviewImageCountTotal() const {
	if (!m_settings.saveOverviewBrightfieldPerZ || m_settings.zSteps <= 0) {
		return 0;
	}
	// Point count/stack depths are constant across z (see overviewCapturePoints()'s own
	// comment), so a single z-index is enough to get the per-z-plane count.
	const auto directionsZ = simplemath::linspace(m_settings.zMin, m_settings.zMax, m_settings.zSteps);
	const auto points = overviewCapturePoints(0, directionsZ);
	size_t perZ = 0;
	for (const auto& point : points) {
		perZ += point.zAbs.size();
	}
	return (int)(perZ * (size_t)m_settings.zSteps);
}

/*
 * Tile centers (x/y, absolute µm) needed to cover the active measurement points with
 * camera-FOV-sized images at exactly kOverviewTileOverlapFraction overlap between
 * neighboring tiles. Tiles are spaced at a fixed pitch and centered on each cluster's
 * extent, so the tiled area overshoots the outermost points symmetrically on both ends
 * (rather than being squeezed to land exactly on them) - "extra" tiles beyond the minimum
 * required always land as overshoot at the edges, never as more-than-requested overlap.
 *
 * "Active" means the actual sampled grid points (m_orderedPositions/m_orderedIndices,
 * already ROI-mask filtered by ScanPlanner) rather than the full rectangular
 * [xMin,xMax]x[yMin,yMax] extent - an ROI mask or a grid whose points are spaced further
 * apart than one FOV can otherwise leave large stretches with nothing to measure, which
 * would be wasteful to tile.
 *
 * Points are first grouped into clusters using proximity (within one FOV size in each
 * axis counts as "connected"), and each cluster is tiled independently - this avoids
 * spending tiles bridging gaps between widely separated groups of points, while still
 * fully covering everything and keeping the fixed overlap within each group.
 *
 * Falls back to a single tile at the grid center if the brightfield camera or scale
 * calibration isn't available, or if no plan has been built yet.
 */
POINT2 Brillouin::overviewTileFootprintUm() const {
	if (!m_scanControl || !m_brightfieldCamera) {
		return POINT2{ 0.0, 0.0 };
	}
	const auto scaleCalibration = m_scanControl->getScaleCalibration();
	const auto brightfieldSettings = m_brightfieldCamera->getSettings();
	const auto width = (double)brightfieldSettings.roi.width_binned;
	const auto height = (double)brightfieldSettings.roi.height_binned;
	// Axis-aligned bounding box, in stage um, of the camera frame's footprint. Using
	// |pixToMicrometerX| * width for the stage-x extent (and the Y equivalent for
	// stage-y) is only correct if the camera's raw pixel axes line up with the stage's
	// x/y motion axes. Some hardware calibrations are rotated relative to the stage
	// (e.g. ZeissMTB_Erlangen's pixToMicrometerX/Y is a 90 degree rotation - a raw
	// pixel-x step moves the stage-y coordinate, not stage-x), in which case that
	// shortcut silently swaps which axis gets padded. This sums each pixel axis's
	// contribution to each stage axis instead, which reduces to the same simple
	// formula for an axis-aligned calibration and is also correct for a rotated one.
	return POINT2{
		std::abs(width * scaleCalibration.pixToMicrometerX.x) + std::abs(height * scaleCalibration.pixToMicrometerY.x),
		std::abs(width * scaleCalibration.pixToMicrometerX.y) + std::abs(height * scaleCalibration.pixToMicrometerY.y)
	};
}

std::vector<POINT2> Brillouin::overviewTileCentersXY() const {
	// Everything here must live in the same frame the live preview actually draws crosses
	// in: absolute-mode positions have absoluteGridOriginUm baked in, but relative-mode
	// positions are pure grid offsets with NO origin added (m_orderedPositionsRelative
	// cancels the start position out entirely - see ScanPlanner::buildLegacyCartesianPlan).
	// Using m_startPosition here instead of {0,0,0} - or m_orderedPositions instead of
	// m_orderedPositionsRelative - used to bake in whatever m_startPosition was last left at
	// (stale, or {0,0,0} before any acquisition ever ran), which doesn't match how the
	// crosses are actually positioned in relative/live-preview mode at all.
	const auto& positions = m_settings.gridCoordinatesAbsolute ? m_orderedPositions : m_orderedPositionsRelative;
	const auto origin = m_settings.gridCoordinatesAbsolute ? resolvedGridOriginUm() : POINT3{};
	const auto gridXMin = m_settings.xMin + origin.x;
	const auto gridXMax = m_settings.xMax + origin.x;
	const auto gridYMin = m_settings.yMin + origin.y;
	const auto gridYMax = m_settings.yMax + origin.y;
	const auto gridCenter = POINT2{ 0.5 * (gridXMin + gridXMax), 0.5 * (gridYMin + gridYMax) };

	if (!m_scanControl || !m_brightfieldCamera) {
		return { gridCenter };
	}

	const auto fov = overviewTileFootprintUm();
	const auto fovWidthUm = fov.x;
	const auto fovHeightUm = fov.y;
	if (fovWidthUm <= 0.0 || fovHeightUm <= 0.0) {
		return { gridCenter };
	}

	// Unique active (x, y) grid positions. The ROI mask is applied per (x, y) regardless
	// of z (see ScanPlanner::buildLegacyCartesianPlan), so it's enough to look at the
	// (x, y) index pairs once rather than per z slice.
	std::set<std::pair<int, int>> seenIndexPairs;
	std::vector<POINT2> activePoints;
	const auto pointCount = std::min(m_orderedIndices.size(), positions.size());
	for (size_t i = 0; i < pointCount; i++) {
		const auto key = std::make_pair(m_orderedIndices[i].x, m_orderedIndices[i].y);
		if (seenIndexPairs.insert(key).second) {
			activePoints.push_back(POINT2{ positions[i].x, positions[i].y });
		}
	}
	if (activePoints.empty()) {
		// No plan built yet - fall back to something sensible rather than tiling nothing.
		return { gridCenter };
	}

	// Union-find clustering: two active points are connected if they're within one FOV
	// size of each other in both axes, i.e. close enough that covering both without a
	// large empty gap between them is worthwhile.
	std::vector<size_t> parent(activePoints.size());
	for (size_t i = 0; i < parent.size(); i++) {
		parent[i] = i;
	}
	std::function<size_t(size_t)> find = [&](size_t i) {
		while (parent[i] != i) {
			parent[i] = parent[parent[i]];
			i = parent[i];
		}
		return i;
	};
	auto unite = [&](size_t a, size_t b) {
		a = find(a);
		b = find(b);
		if (a != b) {
			parent[a] = b;
		}
	};
	for (size_t i = 0; i < activePoints.size(); i++) {
		for (size_t j = i + 1; j < activePoints.size(); j++) {
			const auto dx = std::abs(activePoints[i].x - activePoints[j].x);
			const auto dy = std::abs(activePoints[i].y - activePoints[j].y);
			if (dx <= fovWidthUm && dy <= fovHeightUm) {
				unite(i, j);
			}
		}
	}

	std::map<size_t, std::vector<POINT2>> clusters;
	for (size_t i = 0; i < activePoints.size(); i++) {
		clusters[find(i)].push_back(activePoints[i]);
	}

	std::vector<POINT2> tiles;
	for (const auto& entry : clusters) {
		const auto& clusterPoints = entry.second;
		auto clusterXMin = clusterPoints.front().x;
		auto clusterXMax = clusterPoints.front().x;
		auto clusterYMin = clusterPoints.front().y;
		auto clusterYMax = clusterPoints.front().y;
		for (const auto& p : clusterPoints) {
			clusterXMin = std::min(clusterXMin, p.x);
			clusterXMax = std::max(clusterXMax, p.x);
			clusterYMin = std::min(clusterYMin, p.y);
			clusterYMax = std::max(clusterYMax, p.y);
		}
		const auto tileCountX = overviewRequiredTileCount(clusterXMax - clusterXMin, fovWidthUm, kOverviewTileOverlapFraction);
		const auto tileCountY = overviewRequiredTileCount(clusterYMax - clusterYMin, fovHeightUm, kOverviewTileOverlapFraction);
		const auto centersX = overviewTileCentersAlongAxis(clusterXMin, clusterXMax - clusterXMin, fovWidthUm, kOverviewTileOverlapFraction, tileCountX);
		const auto centersY = overviewTileCentersAlongAxis(clusterYMin, clusterYMax - clusterYMin, fovHeightUm, kOverviewTileOverlapFraction, tileCountY);
		for (const auto y : centersY) {
			for (const auto x : centersX) {
				tiles.push_back(POINT2{ x, y });
			}
		}
	}
	return tiles;
}

/*
 * The true center of the configured [xMin,xMax]x[yMin,yMax] grid, in the same frame
 * overviewTileCentersXY() uses (origin-inclusive absolute um for absolute mode, a pure
 * grid offset for relative mode) - used by the "single image" overview coverage mode.
 */
POINT2 Brillouin::overviewGridCenterXY() const {
	const auto origin = m_settings.gridCoordinatesAbsolute ? resolvedGridOriginUm() : POINT3{};
	const auto gridXMin = m_settings.xMin + origin.x;
	const auto gridXMax = m_settings.xMax + origin.x;
	const auto gridYMin = m_settings.yMin + origin.y;
	const auto gridYMax = m_settings.yMax + origin.y;
	return POINT2{ 0.5 * (gridXMin + gridXMax), 0.5 * (gridYMin + gridYMax) };
}

/*
 * Real measurement-grid (x, y) points, coarse-binned by overviewBrightfieldBin using the
 * exact same coarse-grid reduction coarseXYSamples() shares with the surface pre-scan's
 * "Grid bin" - i.e. these coordinates are literally "the position the scanner would
 * normally image at", not a synthesized tile center. ROI-mask filtered the same way
 * runSurfacePreScan()/the roi-scan-plan-mask metadata test it (raw xMin/xMax-based
 * coordinates, before the origin below is added). Returned in the same frame
 * overviewTileCentersXY() uses, so callers can treat all 3 coverage modes uniformly.
 */
std::vector<POINT2> Brillouin::coarseGridXYPoints(int bin) const {
	const auto [xSamples, ySamples] = coarseXYSamples(bin);
	const auto origin = m_settings.gridCoordinatesAbsolute ? resolvedGridOriginUm() : POINT3{};

	std::vector<POINT2> points;
	points.reserve(xSamples.size() * ySamples.size());
	for (const auto y : ySamples) {
		for (const auto x : xSamples) {
			if (m_settings.useRoiMask && !isPointInPolygonUm(POINT2{ x, y }, m_settings.roiPolygonUm)) {
				continue;
			}
			points.push_back(POINT2{ x + origin.x, y + origin.y });
		}
	}
	return points;
}

std::vector<POINT2> Brillouin::overviewSampledGridXY() const {
	auto points = coarseGridXYPoints(m_settings.overviewBrightfieldBin);
	if (points.empty()) {
		// No point survived the ROI mask (or the grid is degenerate) - fall back to
		// something sensible rather than capturing nothing at all.
		points.push_back(overviewGridCenterXY());
	}
	return points;
}

std::vector<POINT2> Brillouin::surfacePreScanGridXY() const {
	// Must match runSurfacePreScan()'s own point set exactly - this is what the GUI's live
	// "proposed" preview draws, and a mismatch here is exactly the kind of preview/reality
	// drift that once made the ROI polygon overlay land nowhere near its own
	// roi-scan-plan-mask (see planPositionToGridFrame()'s comment).
	auto points = coarseGridXYPoints(m_settings.preScanXYBin);
	const auto origin = m_settings.gridCoordinatesAbsolute ? resolvedGridOriginUm() : POINT3{};
	for (const auto& xy : additionalBoundaryXYPoints(m_settings.additionalBoundaryPoints)) {
		points.push_back(POINT2{ xy.x + origin.x, xy.y + origin.y });
	}
	return points;
}

std::vector<std::pair<POINT2, POINT2>> Brillouin::overviewTileOutlinesUm() const {
	// See overviewTileCentersXY() for why relative mode must use m_orderedPositionsRelative
	// and a zero origin here, not m_orderedPositions/m_startPosition.
	const auto& positions = m_settings.gridCoordinatesAbsolute ? m_orderedPositions : m_orderedPositionsRelative;
	const auto origin = m_settings.gridCoordinatesAbsolute ? resolvedGridOriginUm() : POINT3{};
	const auto gridXMin = m_settings.xMin + origin.x;
	const auto gridXMax = m_settings.xMax + origin.x;
	const auto gridYMin = m_settings.yMin + origin.y;
	const auto gridYMax = m_settings.yMax + origin.y;
	const auto gridCenter = POINT2{ 0.5 * (gridXMin + gridXMax), 0.5 * (gridYMin + gridYMax) };

	if (!m_scanControl || !m_brightfieldCamera) {
		return { { gridCenter, gridCenter } };
	}

	const auto fov = overviewTileFootprintUm();
	const auto fovWidthUm = fov.x;
	const auto fovHeightUm = fov.y;
	if (fovWidthUm <= 0.0 || fovHeightUm <= 0.0) {
		return { { gridCenter, gridCenter } };
	}

	// Unique active (x, y) grid positions - see overviewTileCentersXY() for why this is
	// enough without also looking at z.
	std::set<std::pair<int, int>> seenIndexPairs;
	std::vector<POINT2> activePoints;
	const auto pointCount = std::min(m_orderedIndices.size(), positions.size());
	for (size_t i = 0; i < pointCount; i++) {
		const auto key = std::make_pair(m_orderedIndices[i].x, m_orderedIndices[i].y);
		if (seenIndexPairs.insert(key).second) {
			activePoints.push_back(POINT2{ positions[i].x, positions[i].y });
		}
	}
	if (activePoints.empty()) {
		return { { gridCenter, gridCenter } };
	}

	// Union-find clustering: same connectivity rule as overviewTileCentersXY(), so the
	// outlines drawn here match the tile groups actually captured 1:1.
	std::vector<size_t> parent(activePoints.size());
	for (size_t i = 0; i < parent.size(); i++) {
		parent[i] = i;
	}
	std::function<size_t(size_t)> find = [&](size_t i) {
		while (parent[i] != i) {
			parent[i] = parent[parent[i]];
			i = parent[i];
		}
		return i;
	};
	auto unite = [&](size_t a, size_t b) {
		a = find(a);
		b = find(b);
		if (a != b) {
			parent[a] = b;
		}
	};
	for (size_t i = 0; i < activePoints.size(); i++) {
		for (size_t j = i + 1; j < activePoints.size(); j++) {
			const auto dx = std::abs(activePoints[i].x - activePoints[j].x);
			const auto dy = std::abs(activePoints[i].y - activePoints[j].y);
			if (dx <= fovWidthUm && dy <= fovHeightUm) {
				unite(i, j);
			}
		}
	}

	std::map<size_t, std::vector<POINT2>> clusters;
	for (size_t i = 0; i < activePoints.size(); i++) {
		clusters[find(i)].push_back(activePoints[i]);
	}

	std::vector<std::pair<POINT2, POINT2>> outlines;
	outlines.reserve(clusters.size());
	for (const auto& entry : clusters) {
		const auto& clusterPoints = entry.second;
		auto clusterXMin = clusterPoints.front().x;
		auto clusterXMax = clusterPoints.front().x;
		auto clusterYMin = clusterPoints.front().y;
		auto clusterYMax = clusterPoints.front().y;
		for (const auto& p : clusterPoints) {
			clusterXMin = std::min(clusterXMin, p.x);
			clusterXMax = std::max(clusterXMax, p.x);
			clusterYMin = std::min(clusterYMin, p.y);
			clusterYMax = std::max(clusterYMax, p.y);
		}
		// Same fixed-pitch tile centers overviewTileCentersXY() uses to actually capture this
		// cluster - the outline is just the union of those tiles' edges, so it always shows
		// the real (guaranteed exactly kOverviewTileOverlapFraction) overlap and overshoot,
		// rather than hugging the outermost points.
		const auto tileCountX = overviewRequiredTileCount(clusterXMax - clusterXMin, fovWidthUm, kOverviewTileOverlapFraction);
		const auto tileCountY = overviewRequiredTileCount(clusterYMax - clusterYMin, fovHeightUm, kOverviewTileOverlapFraction);
		const auto centersX = overviewTileCentersAlongAxis(clusterXMin, clusterXMax - clusterXMin, fovWidthUm, kOverviewTileOverlapFraction, tileCountX);
		const auto centersY = overviewTileCentersAlongAxis(clusterYMin, clusterYMax - clusterYMin, fovHeightUm, kOverviewTileOverlapFraction, tileCountY);
		const auto outlineXMin = centersX.front() - 0.5 * fovWidthUm;
		const auto outlineXMax = centersX.back() + 0.5 * fovWidthUm;
		const auto outlineYMin = centersY.front() - 0.5 * fovHeightUm;
		const auto outlineYMax = centersY.back() + 0.5 * fovHeightUm;
		outlines.push_back({ POINT2{ outlineXMin, outlineYMin }, POINT2{ outlineXMax, outlineYMax } });
	}
	return outlines;
}

template <typename T>
void enqueueOverviewBrightfieldImage(
	std::unique_ptr<StorageWrapper>& storage,
	int imageNumber,
	CAMERA_SETTINGS& cameraSettings,
	const std::vector<std::byte>& image,
	const POINT3& targetPosition,
	const POINT3& stagePosition
) {
	auto date = QDateTime::currentDateTime().toOffsetFromUtc(QDateTime::currentDateTime().offsetFromUtc())
		.toString(Qt::ISODateWithMs).toStdString();
	int rankData{ 3 };
	auto dimsData = new hsize_t[3]{ 1, (hsize_t)cameraSettings.roi.height_binned, (hsize_t)cameraSettings.roi.width_binned };
	const auto pixelCount = (size_t)cameraSettings.roi.height_binned * (size_t)cameraSettings.roi.width_binned;
	auto typedImage = std::vector<T>(pixelCount);
	const auto bytesToCopy = std::min(image.size(), typedImage.size() * sizeof(T));
	if (bytesToCopy > 0) {
		std::memcpy(typedImage.data(), image.data(), bytesToCopy);
	}
	auto img = new FLUOIMAGE<T>(
		imageNumber,
		rankData,
		dimsData,
		date,
		"Brightfield z overview",
		typedImage,
		cameraSettings.exposureTime,
		cameraSettings.gain,
		cameraSettings.roi,
		targetPosition,
		true,
		stagePosition
	);

	QMetaObject::invokeMethod(
		storage.get(),
		[&storage = storage, img]() { storage.get()->s_enqueuePayload(img); },
		Qt::AutoConnection
	);
}

void Brillouin::captureOverviewBrightfield(
	std::unique_ptr <StorageWrapper>& storage,
	int imageNumber,
	int zIndex,
	const POINT3& position
) {
	if (!m_settings.saveOverviewBrightfieldPerZ || !m_brightfieldCamera || !m_brightfieldCamera->getConnectionStatus()
		|| !m_scanControl || m_abort) {
		return;
	}

	m_scanControl->setPreset(ScanPreset::SCAN_BRIGHTFIELD);
	// Brightfield capture must never happen with the RL shutter open.
	m_scanControl->setRLShutterOpen(false);
	m_scanControl->setPositionCompensated(position);
	std::this_thread::sleep_for(std::chrono::milliseconds(100));

	// Actual stage position at capture time, read back after the compensated move and
	// settle delay - can differ slightly from the commanded `position` (hysteresis
	// compensation, backlash). Both are stored using the same grid frame sampled-x/y/z and
	// positions-x/y/z already use (see rawPositionToGridFrame()), so all of a file's
	// position metadata stays directly comparable.
	const auto targetPosition = rawPositionToGridFrame(position);
	const auto stagePosition = rawPositionToGridFrame(m_scanControl->getPosition());

	auto brightfieldSettings = m_brightfieldCamera->getSettings();
	// Deliberately no ROI override here - the overview capture always uses whatever ROI the
	// camera is currently at (full sensor unless something else cropped it), giving it the
	// widest field of view of any capture path on purpose.
	brightfieldSettings.exposureTime = 1e-3 * std::max(1, m_settings.overviewBrightfieldExposureMs);
	brightfieldSettings.gain = m_settings.overviewBrightfieldGain;
	brightfieldSettings.frameCount = 1;
	brightfieldSettings.readout.triggerMode = L"Software";
	brightfieldSettings.readout.cycleMode = L"Fixed";

	m_brightfieldCamera->startAcquisition(brightfieldSettings);
	brightfieldSettings = m_brightfieldCamera->getSettings();

	if (brightfieldSettings.roi.bytesPerFrame <= 0) {
		m_brightfieldCamera->stopAcquisition();
		m_scanControl->setPreset(ScanPreset::SCAN_BRILLOUIN);
		m_scanControl->setRLShutterOpen(true);
		emit(s_surfaceScanProgress(
			100.0 * (double)(zIndex + 1) / std::max(1, m_settings.zSteps),
			QString("Skipped brightfield overview for z slice %1/%2: invalid frame size")
				.arg(zIndex + 1)
				.arg(m_settings.zSteps)
		));
		return;
	}

	std::vector<std::byte> image(brightfieldSettings.roi.bytesPerFrame);
	m_brightfieldCamera->getImageForAcquisition(image.data(), false);
	m_brightfieldCamera->stopAcquisition();

	auto queuedImage = false;
	if (brightfieldSettings.readout.dataType == "unsigned short") {
		enqueueOverviewBrightfieldImage<unsigned short>(storage, imageNumber, brightfieldSettings, image, targetPosition, stagePosition);
		queuedImage = true;
	} else if (brightfieldSettings.readout.dataType == "unsigned char") {
		enqueueOverviewBrightfieldImage<unsigned char>(storage, imageNumber, brightfieldSettings, image, targetPosition, stagePosition);
		queuedImage = true;
	}

	if (queuedImage) {
		emit(s_surfaceScanProgress(
			100.0 * (double)(zIndex + 1) / std::max(1, m_settings.zSteps),
			QString("Saved brightfield overview for z slice %1/%2").arg(zIndex + 1).arg(m_settings.zSteps)
		));
	}

	m_scanControl->setPreset(ScanPreset::SCAN_BRILLOUIN);
	m_scanControl->setRLShutterOpen(true);
}

std::string Brillouin::getRepetitionFilename() {
	auto rawFilename = m_baseFilename.substr(0, m_baseFilename.find_last_of("."));
	auto fileEnding = m_baseFilename.substr(m_baseFilename.find_last_of("."), std::string::npos);

	// Get the number of digits necessary for the desired repetition count
	auto nrDigits = (int)floor(log10(m_settings.repetitions.count) + 1);

	auto formatString = std::string{ rawFilename + "_rep%0" + std::to_string(nrDigits) + "d" + fileEnding};

	auto string = QString{};
	string.sprintf(formatString.c_str(), m_currentRepetition);

	return string.toStdString();
}


/*
 * Private slots
 */

void Brillouin::acquire(std::unique_ptr <StorageWrapper>& storage) {
	setAcquisitionStatus(ACQUISITION_STATUS::STARTED);
	// prepare camera for image acquisition

	if (m_andor) {
		m_andor->startAcquisition(m_settings.camera);
		m_settings.camera = m_andor->getSettings();
	} else {
		m_abort = true;
		return;
	}

	if (m_scanControl) {
		QMetaObject::invokeMethod(
			m_scanControl,
			[scanControl = m_scanControl]() { scanControl->stopAnnouncing(); },
			Qt::AutoConnection
		);
		// set optical elements for brightfield/Brillouin imaging
		m_scanControl->setPreset(ScanPreset::SCAN_BRILLOUIN);
		// Force the RL shutter open for the whole acquisition, regardless of whatever
		// manual state the user left it in beforehand - see ScanControl::setPreset()'s
		// comment on why this isn't handled by the preset table itself.
		m_scanControl->setRLShutterOpen(true);
	} else {
		m_abort = true;
		return;
	}
	std::this_thread::sleep_for(std::chrono::milliseconds(500));

	// get current stage position
	if (m_scanControl) {
		m_startPosition = m_scanControl->getPosition();
		// Enable measurement mode (so the AOI display is correct).
		m_scanControl->enableMeasurementMode(true);
	} else {
		m_abort = true;
		return;
	}

	auto commentIn = std::string{ "Brillouin data" };
	storage->setComment(commentIn);

	storage->setResolution("x", m_settings.xSteps);
	storage->setResolution("y", m_settings.ySteps);
	storage->setResolution("z", m_settings.zSteps);

	auto resolutionXout = storage->getResolution("x");

	writeScaleCalibration(storage, ACQUISITION_MODE::BRILLOUIN);
	if (m_settings.saveOverviewBrightfieldPerZ) {
		writeScaleCalibration(storage, ACQUISITION_MODE::FLUORESCENCE);
	}

	/*
	 * Update the positions vector
	 */
	updatePositions();
	applySurfaceFollowPlan();

	// applySurfaceFollowPlan() can come back early (aborted mid-interpolation, see
	// runSurfacePreScan()) without that having been checked here before - which used to mean
	// an abort during that pass was silently ignored: this would still move the stage back,
	// switch to brightfield and enter WAITFORSURFACEREVIEW with an incomplete surface map,
	// exactly as if the scan had finished normally.
	if (m_abort) {
		abortMode(m_acquisition->m_storage);
		return;
	}

	if (m_settings.useSurfaceFollow) {
		// Pause here for the user to review which grid points actually got a surface z
		// value before committing to measuring them. Nothing has been written to `storage`
		// yet at this point - writeScaleCalibration() above is one-time metadata, not
		// per-point data - so this is a safe point to wait an arbitrary amount of time.
		// continueAfterSurfaceReview() (Continue/Full grid buttons) picks up exactly where
		// this leaves off, via runMeasurementPhase().
		if (m_scanControl) {
			// Show the live view at the position the stage was at before this acquisition
			// touched anything, not wherever the surface pre-scan's last column left it.
			m_scanControl->setPositionCompensated(m_startPosition);
			m_scanControl->setPreset(ScanPreset::SCAN_BRIGHTFIELD);
			// The surface-review pause shows a live brightfield view - shutter must be
			// closed for it, same as any other brightfield capture.
			m_scanControl->setRLShutterOpen(false);
			// The grid/ROI overlay itself is fixed (see ScanControl::getPositionOffset())
			// and doesn't need refreshing here. The laser-position marker does, though: the
			// periodic announcer is stopped for the whole acquisition (see stopAnnouncing()
			// above), and setPosition() on a translation stage doesn't announce as a side
			// effect the way NIDAQ's does - so without this it would still show wherever the
			// pre-scan's last column left it, not back here where the stage genuinely is now.
			m_scanControl->announcePosition();
		}
		setAcquisitionStatus(ACQUISITION_STATUS::WAITFORSURFACEREVIEW);
		return;
	}

	runMeasurementPhase(storage);
}

void Brillouin::continueAfterSurfaceReview(bool fullGrid) {
	if (getStatus() != ACQUISITION_STATUS::WAITFORSURFACEREVIEW) {
		return;
	}
	if (m_abort) {
		// Aborted while paused (e.g. the application is closing) - don't start measuring.
		abortMode(m_acquisition->m_storage);
		return;
	}

	if (!fullGrid) {
		// Keep only the (x, y) columns that ended up with a surface z value (found or
		// interpolated) - "Full grid" instead measures every point, letting ones without a
		// surface keep whatever flat z the scan plan already gave them.
		std::vector<POINT3> filteredPositions;
		std::vector<POINT3> filteredPositionsRelative;
		std::vector<INDEX3> filteredIndices;
		std::vector<bool> filteredCalibrationAllowed;
		filteredPositions.reserve(m_orderedPositions.size());
		filteredPositionsRelative.reserve(m_orderedPositionsRelative.size());
		filteredIndices.reserve(m_orderedIndices.size());
		filteredCalibrationAllowed.reserve(m_calibrationAllowed.size());
		for (size_t ll = 0; ll < m_orderedPositions.size(); ll++) {
			const auto key = std::make_pair(m_orderedIndices[ll].x, m_orderedIndices[ll].y);
			if (m_surfaceFoundXYIndices.find(key) == m_surfaceFoundXYIndices.end()) {
				continue;
			}
			filteredPositions.push_back(m_orderedPositions[ll]);
			filteredPositionsRelative.push_back(m_orderedPositionsRelative[ll]);
			filteredIndices.push_back(m_orderedIndices[ll]);
			filteredCalibrationAllowed.push_back(m_calibrationAllowed[ll]);
		}
		m_orderedPositions = std::move(filteredPositions);
		m_orderedPositionsRelative = std::move(filteredPositionsRelative);
		m_orderedIndices = std::move(filteredIndices);
		m_calibrationAllowed = std::move(filteredCalibrationAllowed);
	}

	if (m_scanControl) {
		m_scanControl->setPreset(ScanPreset::SCAN_BRILLOUIN);
		// Resuming actual measurement after the surface-review pause - shutter must be
		// open again.
		m_scanControl->setRLShutterOpen(true);
	}
	setAcquisitionStatus(ACQUISITION_STATUS::STARTED);
	runMeasurementPhase(m_acquisition->m_storage);

	// Mirrors waitForNextRepetition()'s own post-acquire() check: if the measurement
	// itself aborted (hardware failure, user abort, ...), don't advance to the next
	// repetition as if it had completed - hand off to the same abort path instead.
	if (m_abort) {
		abortMode(m_acquisition->m_storage);
		return;
	}
	finishRepetition();
}

void Brillouin::approachGridPosition(const POINT3& position) {
	if (!m_scanControl) {
		return;
	}
	if (m_settings.useGridHysteresisCompensation) {
		m_scanControl->setPositionCompensated(position);
	} else {
		m_scanControl->setPosition(position);
	}
}

void Brillouin::runMeasurementPhase(std::unique_ptr<StorageWrapper>& storage) {
	/*
	 * Construct positions vector for H5 file with row-major order: z, x, y
	 */
	 // construct directions vectors
	auto directionsX{ simplemath::linspace(m_settings.xMin, m_settings.xMax, m_settings.xSteps) };
	auto directionsY{ simplemath::linspace(m_settings.yMin, m_settings.yMax, m_settings.ySteps) };
	auto directionsZ{ simplemath::linspace(m_settings.zMin, m_settings.zMax, m_settings.zSteps) };

	// total number of positions to measure (can be sparse if ROI masking is active)
	auto nrPositions = (int)m_orderedPositions.size();
	if (nrPositions <= 0) {
		m_abort = true;
		return;
	}
	const auto gridPositionCount = m_settings.xSteps * m_settings.ySteps * m_settings.zSteps;
	auto positionsX = std::vector<double>(gridPositionCount);
	auto positionsY = std::vector<double>(gridPositionCount);
	auto positionsZ = std::vector<double>(gridPositionCount);
	auto posIndex{ 0 };
	for (gsl::index ii{ 0 }; ii < m_settings.zSteps; ii++) {
		for (gsl::index jj{ 0 }; jj < m_settings.xSteps; jj++) {
			for (gsl::index kk{ 0 }; kk < m_settings.ySteps; kk++) {
				positionsX[posIndex] = m_settings.gridCoordinatesAbsolute ? directionsX[jj] : directionsX[jj] + m_startPosition.x;
				positionsY[posIndex] = m_settings.gridCoordinatesAbsolute ? directionsY[kk] : directionsY[kk] + m_startPosition.y;
				positionsZ[posIndex] = m_settings.gridCoordinatesAbsolute ? directionsZ[ii] : directionsZ[ii] + m_startPosition.z;
				posIndex++;
			}
		}
	}

	auto rank{ 3 };
	auto dims = new hsize_t[rank];
	dims[0] = m_settings.zSteps;
	dims[1] = m_settings.xSteps;
	dims[2] = m_settings.ySteps;

	storage->setPositions("x", positionsX, rank, dims);
	storage->setPositions("y", positionsY, rank, dims);
	storage->setPositions("z", positionsZ, rank, dims);
	const hsize_t originDims[1] = { 1 };
	storage->setPositions("absolute-origin-x", std::vector<double>{ m_settings.absoluteGridOriginUm.x }, 1, originDims);
	storage->setPositions("absolute-origin-y", std::vector<double>{ m_settings.absoluteGridOriginUm.y }, 1, originDims);
	storage->setPositions("absolute-origin-z", std::vector<double>{ m_settings.absoluteGridOriginUm.z }, 1, originDims);
	storage->setPositions("grid-coordinates-absolute", std::vector<double>{ m_settings.gridCoordinatesAbsolute ? 1.0 : 0.0 }, 1, originDims);
	// The objective/FOV-offset context this specific run actually resolved its absolute-mode
	// origin against (see resolvedGridOriginUm()) - "absolute-origin-x/y/z" above is always the
	// raw, unmodified reference-frame value the operator set. This used to also be duplicated
	// here as "objective-*" datasets; it now lives only in writeScaleCalibration()'s
	// scaleCalibration group (objectiveSlot, hasFovOffset, fovOffset, missingOffsetAccepted, ...),
	// which every acquisition mode writes through, not just this one - see
	// AcquisitionMode::writeScaleCalibration() and H5BM::setScaleCalibration().

	// Explicitly store which grid points were sampled to keep metadata consistent for sparse ROI scans.
	// Must match the [zSteps, xSteps, ySteps] row-major layout the "x"/"y"/"z" datasets above use
	// (x varies before y) - NOT the (z, y, x) convention h5bm's per-spectrum dataset naming uses,
	// which is a different, unrelated indexing scheme.
	auto sampledMask = std::vector<double>(gridPositionCount, 0.0);
	for (gsl::index ll{ 0 }; ll < (gsl::index)m_orderedIndices.size(); ll++) {
		const auto idx = m_orderedIndices[ll];
		const auto flat = idx.z * (m_settings.xSteps * m_settings.ySteps) + idx.x * m_settings.ySteps + idx.y;
		if (flat >= 0 && flat < gridPositionCount) {
			sampledMask[flat] = 1.0;
		}
	}
	storage->setPositions("sampled-mask", sampledMask, rank, dims);

	// Surface-mapping diagnostics: what the surface pre-scan (if any) actually found, and
	// what settings produced it - written unconditionally (surface-follow-used is 0 when
	// the feature was off) so every file has the same schema. surface-found-mask uses
	// 0 = no surface z value, 1 = found purely from genuine measurements, 2 = leans on at
	// least one gap-filled coarse cell (see the gap-fill pass in runSurfacePreScan()).
	const hsize_t surfaceDims[2] = { (hsize_t)m_settings.xSteps, (hsize_t)m_settings.ySteps };
	auto surfaceFoundMask = std::vector<double>((size_t)m_settings.xSteps * m_settings.ySteps, 0.0);
	const auto foundXYIndices = getSurfaceFoundXYIndices();
	const auto interpolatedXYIndices = getSurfaceInterpolatedXYIndices();
	for (const auto& xy : foundXYIndices) {
		const auto flat = xy.first * m_settings.ySteps + xy.second;
		if (flat >= 0 && flat < (int)surfaceFoundMask.size()) {
			surfaceFoundMask[flat] = interpolatedXYIndices.count(xy) > 0 ? 2.0 : 1.0;
		}
	}
	storage->setPositions("surface-found-mask", surfaceFoundMask, 2, surfaceDims);
	storage->setPositions("surface-follow-used", std::vector<double>{ m_settings.useSurfaceFollow ? 1.0 : 0.0 }, 1, originDims);
	storage->setPositions("surface-drop-fraction-used", std::vector<double>{ m_settings.surfaceDropFraction }, 1, originDims);
	storage->setPositions("surface-medium-reference-value-used", std::vector<double>{ m_settings.mediumReferenceValue }, 1, originDims);
	storage->setPositions("surface-z-offset-um-used", std::vector<double>{ m_settings.surfaceZOffsetUm }, 1, originDims);
	storage->setPositions("surface-follow-half-range-um-used", std::vector<double>{ m_settings.surfaceFollowHalfRangeUm }, 1, originDims);
	storage->setPositions("surface-max-rewind-um-used", std::vector<double>{ m_settings.surfaceMaxRewindUm }, 1, originDims);
	storage->setPositions("surface-verification-steps-used", std::vector<double>{ (double)m_settings.surfaceVerificationSteps }, 1, originDims);
	storage->setPositions("surface-verification-frame-average-used", std::vector<double>{ (double)m_settings.surfaceVerificationFrameAverage }, 1, originDims);
	storage->setPositions("surface-verification-tolerance-fraction-used", std::vector<double>{ m_settings.surfaceVerificationToleranceFraction }, 1, originDims);
	storage->setPositions("surface-pre-scan-xy-bin-used", std::vector<double>{ (double)m_settings.preScanXYBin }, 1, originDims);
	storage->setPositions("surface-pre-scan-additional-boundary-points-used", std::vector<double>{ (double)m_settings.additionalBoundaryPoints }, 1, originDims);
	storage->setPositions("surface-pre-scan-z-step-um-used", std::vector<double>{ m_settings.preScanZStepUm }, 1, originDims);
	storage->setPositions("surface-pre-scan-z-travel-range-um-used", std::vector<double>{ m_settings.preScanZTravelRangeUm }, 1, originDims);
	storage->setPositions("surface-metric-threshold-used", std::vector<double>{ m_settings.surfaceMetricThreshold }, 1, originDims);
	storage->setPositions("surface-smooth-sigma-um-used", std::vector<double>{ m_settings.surfaceSmoothSigmaUm }, 1, originDims);
	storage->setPositions("surface-scan-direction-used", std::vector<double>{ (double)m_settings.surfaceScanDirection }, 1, originDims);
	storage->setPositions("surface-medium-reference-frame-count-used", std::vector<double>{ (double)m_settings.mediumReferenceFrameCount }, 1, originDims);
	const hsize_t proxyRoiDims[1] = { 4 };
	storage->setPositions("surface-proxy-roi-1-used", std::vector<double>{
		(double)m_settings.surfaceProxyRoiLeft, (double)m_settings.surfaceProxyRoiTop,
		(double)m_settings.surfaceProxyRoiWidth, (double)m_settings.surfaceProxyRoiHeight
	}, 1, proxyRoiDims);
	storage->setPositions("surface-proxy-roi-2-used", std::vector<double>{
		(double)m_settings.surfaceProxyRoi2Left, (double)m_settings.surfaceProxyRoi2Top,
		(double)m_settings.surfaceProxyRoi2Width, (double)m_settings.surfaceProxyRoi2Height
	}, 1, proxyRoiDims);
	// The frame (crop/binning + its absolute sensor origin) the two ROIs above were drawn
	// against - see the surfaceProxyRoiFrameWidth et al. declarations in Brillouin.h for
	// why this matters (the ROI gets remapped through this if the camera's actual frame at
	// measurement time differs).
	const hsize_t proxyRoiFrameDims[1] = { 6 };
	storage->setPositions("surface-proxy-roi-1-frame-used", std::vector<double>{
		(double)m_settings.surfaceProxyRoiFrameWidth, (double)m_settings.surfaceProxyRoiFrameHeight,
		(double)m_settings.surfaceProxyRoiFrameOriginLeft, (double)m_settings.surfaceProxyRoiFrameOriginBottom,
		(double)m_settings.surfaceProxyRoiFrameWidthPhysical, (double)m_settings.surfaceProxyRoiFrameHeightPhysical
	}, 1, proxyRoiFrameDims);
	storage->setPositions("surface-proxy-roi-2-frame-used", std::vector<double>{
		(double)m_settings.surfaceProxyRoi2FrameWidth, (double)m_settings.surfaceProxyRoi2FrameHeight,
		(double)m_settings.surfaceProxyRoi2FrameOriginLeft, (double)m_settings.surfaceProxyRoi2FrameOriginBottom,
		(double)m_settings.surfaceProxyRoi2FrameWidthPhysical, (double)m_settings.surfaceProxyRoi2FrameHeightPhysical
	}, 1, proxyRoiFrameDims);
	// preScanXSteps/YSteps/ZSteps/ZMin/ZMax deliberately NOT saved: grep-confirmed dead -
	// only ever round-tripped through the app's own UI settings, never read by
	// runSurfacePreScan() (which uses preScanXYBin/preScanZStepUm/preScanZTravelRangeUm,
	// already saved above). Recording an unused setting doesn't tell a reader anything
	// about what actually happened during the scan, so it isn't saved here.
	storage->setPositions("surface-reference-threshold-computed",
		std::vector<double>{ m_surfaceReferenceThreshold }, 1, originDims);

	// Raw coarse surface pre-scan data: the (binned) grid runSurfacePreScan() itself
	// measured on, before any interpolation onto the dense scan-plan grid above - not
	// otherwise recoverable from surface-found-mask/positions-z, which only ever show the
	// already-interpolated, dense result. Skipped entirely (not even an empty dataset) if
	// the pre-scan never ran (surface-follow off) or bailed out before any column was
	// measured - "surface-follow-used" is what distinguishes that from a pre-scan that ran
	// but found nothing anywhere.
	if (!m_surfacePreScanXUm.empty()) {
		const hsize_t preScanXDims[1] = { (hsize_t)m_surfacePreScanXUm.size() };
		const hsize_t preScanYDims[1] = { (hsize_t)m_surfacePreScanYUm.size() };
		// m_surfacePreScanXUm/YUm are built from the same xMin/xMax-based "grid-plan" frame
		// as directionsX/Y (see coarseXYSamples()) - planPositionToGridFrame() converts that
		// into the same frame positions-x/y/z (and roi-polygon-x/y-um below) are saved in, so
		// a reader can overlay all of them without needing m_startPosition itself (never
		// saved).
		std::vector<double> preScanX(m_surfacePreScanXUm.size());
		std::vector<double> preScanY(m_surfacePreScanYUm.size());
		for (size_t i = 0; i < m_surfacePreScanXUm.size(); i++) {
			preScanX[i] = planPositionToGridFrame(
				POINT3{ m_surfacePreScanXUm[i], 0, 0 }).x;
		}
		for (size_t i = 0; i < m_surfacePreScanYUm.size(); i++) {
			preScanY[i] = planPositionToGridFrame(
				POINT3{ 0, m_surfacePreScanYUm[i], 0 }).y;
		}
		storage->setPositions("surface-prescan-x-um", preScanX, 1, preScanXDims);
		storage->setPositions("surface-prescan-y-um", preScanY, 1, preScanYDims);

		const hsize_t preScanGridDims[2] = {
			(hsize_t)m_surfacePreScanXUm.size(), (hsize_t)m_surfacePreScanYUm.size()
		};
		// 0 = no drop found within the travel range, 1 = a genuine verified measurement,
		// 2 = filled in from neighboring coarse cells - same meaning as the dense
		// surface-found-mask, at the coarse pre-scan's own resolution.
		storage->setPositions(
			"surface-prescan-found-mask", m_surfacePreScanFoundMask, 2, preScanGridDims);
		// z (relative to this column's own zOrigin) at which the surface was found/filled -
		// 0.0 where surface-prescan-found-mask == 0.
		storage->setPositions(
			"surface-prescan-z-um", m_surfacePreScanZUm, 2, preScanGridDims);
		// The drop metric actually measured at that column (frame-averaged where
		// verification ran) - the verified value where found-mask == 1, whatever was last
		// measured before giving up otherwise, so a failed column's metric can still be
		// compared against surface-reference-threshold-computed.
		storage->setPositions(
			"surface-prescan-metric", m_surfacePreScanMetric, 2, preScanGridDims);
	}

	// Additional boundary points that found a surface (additionalBoundaryPoints - see
	// additionalBoundaryXYPoints()/measureBoundarySurfaceZ()) - a flat list, not a grid, since
	// unlike the rectangular coarse scan above there is no fixed slot for a point that found
	// nothing. Skipped entirely (not even an empty dataset) if none were requested or none of
	// the requested ones found a surface.
	if (!m_surfaceBoundaryPointsUm.empty()) {
		const hsize_t boundaryDims[1] = { (hsize_t)m_surfaceBoundaryPointsUm.size() };
		std::vector<double> boundaryX(m_surfaceBoundaryPointsUm.size());
		std::vector<double> boundaryY(m_surfaceBoundaryPointsUm.size());
		std::vector<double> boundaryZ(m_surfaceBoundaryPointsUm.size());
		for (size_t i = 0; i < m_surfaceBoundaryPointsUm.size(); i++) {
			const auto stored = planPositionToGridFrame(
				POINT3{ m_surfaceBoundaryPointsUm[i].x, m_surfaceBoundaryPointsUm[i].y, 0 });
			boundaryX[i] = stored.x;
			boundaryY[i] = stored.y;
			// z relative to this column's own zOrigin - same convention as surface-prescan-z-um.
			boundaryZ[i] = m_surfaceBoundaryPointsUm[i].z;
		}
		storage->setPositions("surface-prescan-boundary-x-um", boundaryX, 1, boundaryDims);
		storage->setPositions("surface-prescan-boundary-y-um", boundaryY, 1, boundaryDims);
		storage->setPositions("surface-prescan-boundary-z-um", boundaryZ, 1, boundaryDims);
	}

	// Grid extent as configured (redundant with the "x"/"y"/"z" position arrays above, but
	// explicit scalars are easier for a reader to check at a glance than reconstructing
	// min/max from those arrays).
	const hsize_t gridExtentDims[1] = { 2 };
	storage->setPositions("grid-x-range-um-used", std::vector<double>{ m_settings.xMin, m_settings.xMax }, 1, gridExtentDims);
	storage->setPositions("grid-y-range-um-used", std::vector<double>{ m_settings.yMin, m_settings.yMax }, 1, gridExtentDims);
	storage->setPositions("grid-z-range-um-used", std::vector<double>{ m_settings.zMin, m_settings.zMax }, 1, gridExtentDims);

	// Calibration schedule actually used for this repetition.
	storage->setPositions("pre-calibration-used", std::vector<double>{ m_settings.preCalibration ? 1.0 : 0.0 }, 1, originDims);
	storage->setPositions("post-calibration-used", std::vector<double>{ m_settings.postCalibration ? 1.0 : 0.0 }, 1, originDims);
	storage->setPositions("con-calibration-used", std::vector<double>{ m_settings.conCalibration ? 1.0 : 0.0 }, 1, originDims);
	storage->setPositions("con-calibration-interval-min-used", std::vector<double>{ m_settings.conCalibrationInterval }, 1, originDims);
	storage->setPositions("nr-calibration-images-used", std::vector<double>{ (double)m_settings.nrCalibrationImages }, 1, originDims);
	storage->setPositions("calibration-exposure-time-s-used", std::vector<double>{ m_settings.calibrationExposureTime }, 1, originDims);

	// Repetition schedule.
	storage->setPositions("repetitions-count-used", std::vector<double>{ (double)m_settings.repetitions.count }, 1, originDims);
	storage->setPositions("repetitions-interval-min-used", std::vector<double>{ m_settings.repetitions.interval }, 1, originDims);
	storage->setPositions("repetitions-file-per-repetition-used", std::vector<double>{ m_settings.repetitions.filePerRepetition ? 1.0 : 0.0 }, 1, originDims);

	// ROI polygon vertices (its effect on the plan is already in roi-scan-plan-mask below,
	// but the raw polygon itself wasn't recorded anywhere). m_settings.roiPolygonUm lives in
	// the same grid-plan frame as directionsX/Y (see isPointInPolygonUm() callers, which test
	// it against that frame directly) - planPositionToGridFrame() converts that into the same
	// frame positions-x/y/z is saved in, so a consumer overlaying the two can reconcile them
	// without needing m_startPosition itself (never saved anywhere in the file).
	if (!m_settings.roiPolygonUm.empty()) {
		const hsize_t roiPolyDims[1] = { (hsize_t)m_settings.roiPolygonUm.size() };
		std::vector<double> roiPolyX(m_settings.roiPolygonUm.size());
		std::vector<double> roiPolyY(m_settings.roiPolygonUm.size());
		for (size_t i = 0; i < m_settings.roiPolygonUm.size(); i++) {
			const auto stored = planPositionToGridFrame(
				POINT3{ m_settings.roiPolygonUm[i].x, m_settings.roiPolygonUm[i].y, 0 });
			roiPolyX[i] = stored.x;
			roiPolyY[i] = stored.y;
		}
		storage->setPositions("roi-polygon-x-um", roiPolyX, 1, roiPolyDims);
		storage->setPositions("roi-polygon-y-um", roiPolyY, 1, roiPolyDims);
	}

	// BF overview coverage settings actually used (their effect on shape is already visible
	// in overview-brightfield-x/y/z + point-count/point-stack-counts, but the flags
	// themselves weren't recorded, unlike useSurfaceFollow/useRoiMask above).
	storage->setPositions("overview-brightfield-save-per-z-used", std::vector<double>{ m_settings.saveOverviewBrightfieldPerZ ? 1.0 : 0.0 }, 1, originDims);
	storage->setPositions("overview-brightfield-full-grid-used", std::vector<double>{ m_settings.overviewBrightfieldFullGrid ? 1.0 : 0.0 }, 1, originDims);
	storage->setPositions("overview-brightfield-sampled-grid-used", std::vector<double>{ m_settings.overviewBrightfieldSampledGrid ? 1.0 : 0.0 }, 1, originDims);
	storage->setPositions("overview-brightfield-bin-used", std::vector<double>{ (double)m_settings.overviewBrightfieldBin }, 1, originDims);
	storage->setPositions("overview-brightfield-full-stack-used", std::vector<double>{ m_settings.overviewBrightfieldFullStack ? 1.0 : 0.0 }, 1, originDims);
	storage->setPositions("overview-brightfield-exposure-ms-used", std::vector<double>{ (double)m_settings.overviewBrightfieldExposureMs }, 1, originDims);
	storage->setPositions("overview-brightfield-gain-used", std::vector<double>{ m_settings.overviewBrightfieldGain }, 1, originDims);

	// Grid-stepping/camera settings not otherwise recorded per-image.
	storage->setPositions("use-grid-hysteresis-compensation-used", std::vector<double>{ m_settings.useGridHysteresisCompensation ? 1.0 : 0.0 }, 1, originDims);
	storage->setPositions("camera-frame-count-used", std::vector<double>{ (double)m_settings.camera.frameCount }, 1, originDims);
	storage->setPositions("camera-spurious-noise-filter-used", std::vector<double>{ m_settings.camera.spuriousNoiseFilter ? 1.0 : 0.0 }, 1, originDims);

	// Which (x, y) columns were geometrically inside the ROI mask (or every column, if no
	// ROI mask is active) - without this, a 0 in surface-found-mask or sampled-mask is
	// ambiguous between "outside the ROI, never considered" and "inside the ROI but not
	// found / not measured for another reason". Uses the same xMin/xMax-based coordinates
	// (not origin-shifted) that ScanPlanner and runSurfacePreScan() already test the ROI
	// polygon against, so this is consistent with what was actually excluded upstream.
	auto roiScanPlanMask = std::vector<double>((size_t)m_settings.xSteps * m_settings.ySteps, 1.0);
	if (m_settings.useRoiMask) {
		for (int ix = 0; ix < m_settings.xSteps; ix++) {
			for (int iy = 0; iy < m_settings.ySteps; iy++) {
				const POINT2 point{ directionsX[ix], directionsY[iy] };
				const auto flat = ix * m_settings.ySteps + iy;
				roiScanPlanMask[flat] = isPointInPolygonUm(point, m_settings.roiPolygonUm) ? 1.0 : 0.0;
			}
		}
	}
	storage->setPositions("roi-scan-plan-mask", roiScanPlanMask, 2, surfaceDims);
	storage->setPositions("roi-mask-used", std::vector<double>{ m_settings.useRoiMask ? 1.0 : 0.0 }, 1, originDims);

	// Store the actual sampled path as 1D vectors in acquisition order.
	const int sampledRank{ 1 };
	hsize_t sampledDims[1] = { (hsize_t)m_orderedPositions.size() };
	auto sampledX = std::vector<double>(m_orderedPositions.size());
	auto sampledY = std::vector<double>(m_orderedPositions.size());
	auto sampledZ = std::vector<double>(m_orderedPositions.size());
	const auto sampledOrigin = resolvedGridOriginUm();
	for (gsl::index ll{ 0 }; ll < (gsl::index)m_orderedPositions.size(); ll++) {
		sampledX[ll] = m_settings.gridCoordinatesAbsolute
			? m_orderedPositions[ll].x - sampledOrigin.x
			: m_orderedPositions[ll].x;
		sampledY[ll] = m_settings.gridCoordinatesAbsolute
			? m_orderedPositions[ll].y - sampledOrigin.y
			: m_orderedPositions[ll].y;
		sampledZ[ll] = m_settings.gridCoordinatesAbsolute
			? m_orderedPositions[ll].z - sampledOrigin.z
			: m_orderedPositions[ll].z;
	}
	storage->setPositions("sampled-x", sampledX, sampledRank, sampledDims);
	storage->setPositions("sampled-y", sampledY, sampledRank, sampledDims);
	storage->setPositions("sampled-z", sampledZ, sampledRank, sampledDims);

	if (m_settings.saveOverviewBrightfieldPerZ) {
		// Flattened as [z0_point0_stack0, z0_point0_stack1, ..., z0_point1_stack0, ...,
		// z1_point0_stack0, ...]; both the number of capture points (the overview image's
		// xy point(s), plus "sampled grid points" if that's additionally on) and each
		// point's own stack depth are constant across z (each only depends on
		// z-independent settings), so the shape can be read once from z-index 0.
		// overview-brightfield-point-count/-point-stack-counts let a reader reshape this
		// back into each point's own stack - stack depth is 1 for every point except the
		// overview image's own when overviewBrightfieldFullStack is on (see
		// overviewCapturePoints()).
		const auto shape = overviewCapturePoints(0, directionsZ);
		const auto pointCount = shape.size();
		auto pointStackCounts = std::vector<double>(pointCount);
		size_t totalPerZ = 0;
		for (size_t p = 0; p < pointCount; p++) {
			pointStackCounts[p] = (double)shape[p].zAbs.size();
			totalPerZ += shape[p].zAbs.size();
		}
		const auto totalOverviewCount = (size_t)m_settings.zSteps * totalPerZ;
		hsize_t overviewDims[1] = { (hsize_t)totalOverviewCount };
		auto overviewX = std::vector<double>(totalOverviewCount);
		auto overviewY = std::vector<double>(totalOverviewCount);
		auto overviewZ = std::vector<double>(totalOverviewCount);
		const auto overviewOrigin = resolvedGridOriginUm();
		for (gsl::index ii{ 0 }; ii < m_settings.zSteps; ii++) {
			const auto capturePoints = overviewCapturePoints((int)ii, directionsZ);
			auto flatIndex = (size_t)ii * totalPerZ;
			for (size_t p = 0; p < capturePoints.size() && p < pointCount; p++) {
				const auto& point = capturePoints[p];
				for (size_t ss = 0; ss < point.zAbs.size(); ss++) {
					// Use the same convention as sampled-x/y/z above, so overview and
					// Brillouin positions can be compared/overlaid directly without the
					// caller having to know which fields are absolute vs. origin-relative.
					overviewX[flatIndex] = m_settings.gridCoordinatesAbsolute
						? point.xy.x - overviewOrigin.x
						: point.xy.x;
					overviewY[flatIndex] = m_settings.gridCoordinatesAbsolute
						? point.xy.y - overviewOrigin.y
						: point.xy.y;
					overviewZ[flatIndex] = m_settings.gridCoordinatesAbsolute
						? point.zAbs[ss] - overviewOrigin.z
						: point.zAbs[ss];
					flatIndex++;
				}
			}
		}
		storage->setPositions("overview-brightfield-x", overviewX, sampledRank, overviewDims);
		storage->setPositions("overview-brightfield-y", overviewY, sampledRank, overviewDims);
		storage->setPositions("overview-brightfield-z", overviewZ, sampledRank, overviewDims);
		const hsize_t pointCountDims[1] = { 1 };
		storage->setPositions("overview-brightfield-point-count", std::vector<double>{ (double)pointCount }, 1, pointCountDims);
		const hsize_t pointStackDims[1] = { (hsize_t)pointCount };
		storage->setPositions("overview-brightfield-point-stack-counts", pointStackCounts, 1, pointStackDims);
	}
	delete[] dims;

	// do actual measurement
	QMetaObject::invokeMethod(
		storage.get(),
		[&storage = storage]() { storage.get()->startWritingQueues(); },
		Qt::AutoConnection
	);

	auto rank_data{ 3 };
	hsize_t dims_data[3] = {
		(hsize_t)m_settings.camera.frameCount,
		(hsize_t)m_settings.camera.roi.height_binned,
		(hsize_t)m_settings.camera.roi.width_binned
	};

	// reset number of calibrations
	nrCalibrations = 1;
	// do pre calibration
	if (m_settings.preCalibration) {
		calibrate(storage);
	}

	auto measurementTimer = QElapsedTimer{};
	measurementTimer.start();

	auto calibrationTimer = QElapsedTimer{};
	calibrationTimer.start();
	// Last traversal index at which each z-index appears, so its overview can be captured
	// once it's actually done being measured - not before any of it has, and not affected by
	// which axis (x/y/z) is scanned outermost (m_scanOrder), unlike triggering on each
	// z-index's *first* appearance would be: with z scanned innermost, that first-appearance
	// approach would trigger for most z-indices within the first few points, looking like
	// every overview gets captured upfront.
	auto lastIndexForZ = std::vector<gsl::index>(m_settings.zSteps, -1);
	for (gsl::index ll{ 0 }; ll < (gsl::index)nrPositions; ll++) {
		const auto zIdx = std::clamp(m_orderedIndices[ll].z, 0, std::max(0, m_settings.zSteps - 1));
		lastIndexForZ[zIdx] = ll;
	}

	// move stage to first position, wait 50 ms for it to finish
	if (m_scanControl) {
		// Approach from a consistent direction to compensate for stage hysteresis (unless
		// useGridHysteresisCompensation is off), so the grid is reached reproducibly
		// regardless of where the stage was before.
		approachGridPosition(m_orderedPositions[0]);
		// The periodic position-announcer is stopped for the whole acquisition (see
		// stopAnnouncing() in acquire()), and a translation stage's setPosition() doesn't
		// announce as a side effect the way the galvo's does - so without this, the laser-
		// position marker would never visibly move through the grid while a scan runs.
		m_scanControl->announcePosition();
	} else {
		m_abort = true;
		return;
	}
	std::this_thread::sleep_for(std::chrono::milliseconds(50));

	for (gsl::index ll{ 0 }; ll < (gsl::index)nrPositions; ll++) {

		// do live calibration if required and possible at the moment
		if (m_settings.conCalibration && m_calibrationAllowed[ll]) {
			if (calibrationTimer.elapsed() > (60e3 * m_settings.conCalibrationInterval)) {
				calibrate(storage);
				calibrationTimer.start();
				// After we calibrated, we move back to the current position.
				// The calibration preset can move the stage away (e.g. to a reference sample),
				// so approach the grid point from a consistent direction to avoid hysteresis error.
				if (m_scanControl) {
					approachGridPosition(m_orderedPositions[ll]);
				} else {
					m_abort = true;
					return;
				}
				std::this_thread::sleep_for(std::chrono::milliseconds(100));
			}
		}

		auto nextCalibration = int{ (int)(100 * (1e-3 * calibrationTimer.elapsed()) / (60 * m_settings.conCalibrationInterval)) };
		emit(s_timeToCalibration(nextCalibration));

		const auto zIndex = std::clamp(m_orderedIndices[ll].z, 0, std::max(0, m_settings.zSteps - 1));

		std::vector<std::byte> images(m_settings.camera.roi.bytesPerFrame * m_settings.camera.frameCount);

		for (gsl::index mm{ 0 }; mm < m_settings.camera.frameCount; mm++) {
			if (m_abort) {
				m_abort = true;
				return;
			}
			const auto displayedPosition = m_settings.gridCoordinatesAbsolute
				? m_orderedPositions[ll] - resolvedGridOriginUm()
				: m_orderedPositions[ll] - m_startPosition;
			emit(s_positionChanged(displayedPosition, mm + 1));
			// acquire images
			auto pointerPos = (int64_t)m_settings.camera.roi.bytesPerFrame * mm;

			if (m_andor) {
				m_andor->getImageForAcquisition(&images[pointerPos]);
			} else {
				m_abort = true;
				return;
			}
		}


		// asynchronously write image to disk
		// the datetime has to be set here, otherwise it would be determined by the time the queue is processed
		auto date = QDateTime::currentDateTime().toOffsetFromUtc(QDateTime::currentDateTime().offsetFromUtc())
			.toString(Qt::ISODateWithMs).toStdString();

		if (m_settings.camera.readout.dataType == "unsigned short") {
			// cast the image to unsigned short
			auto images_ = (std::vector<unsigned short> *) & images;
			auto img = new IMAGE<unsigned short>(
				m_orderedIndices[ll].x,
				m_orderedIndices[ll].y,
				m_orderedIndices[ll].z,
				rank_data,
				dims_data,
				date,
				*images_,
				m_settings.camera.exposureTime,
				m_settings.camera.gain,
				m_settings.camera.roi
			);

			QMetaObject::invokeMethod(
				storage.get(),
				[&storage = storage, img]() { storage.get()->s_enqueuePayload(img); },
				Qt::AutoConnection
			);
		} else if (m_settings.camera.readout.dataType == "unsigned char") {
			// cast the image to unsigned char
			auto images_ = (std::vector<unsigned char> *) & images;
			auto img = new IMAGE<unsigned char>(
				m_orderedIndices[ll].x,
				m_orderedIndices[ll].y,
				m_orderedIndices[ll].z,
				rank_data,
				dims_data,
				date,
				*images_,
				m_settings.camera.exposureTime,
				m_settings.camera.gain,
				m_settings.camera.roi
			);

			QMetaObject::invokeMethod(
				storage.get(),
				[&storage = storage, img]() { storage.get()->s_enqueuePayload(img); },
				Qt::AutoConnection
			);
		} else if (m_settings.camera.readout.dataType == "unsigned int") {
			// cast the image to unsigned char
			auto images_ = (std::vector<unsigned int> *) & images;
			auto img = new IMAGE<unsigned int>(
				m_orderedIndices[ll].x,
				m_orderedIndices[ll].y,
				m_orderedIndices[ll].z,
				rank_data,
				dims_data,
				date,
				*images_,
				m_settings.camera.exposureTime,
				m_settings.camera.gain,
				m_settings.camera.roi
			);

			QMetaObject::invokeMethod(
				storage.get(),
				[&storage = storage, img]() { storage.get()->s_enqueuePayload(img); },
				Qt::AutoConnection
			);
		}

		// This z-plane's last point has now actually been measured - only now capture its
		// overview, not before any of it was (see lastIndexForZ above for why "last
		// occurrence" rather than "first" is what makes this robust to scan order).
		if (m_settings.saveOverviewBrightfieldPerZ && ll == lastIndexForZ[zIndex]) {
			// The overview image's xy point(s) each get their own stack (1 for the legacy
			// single-image-per-z behaviour, zSteps for a full stack), followed by "sampled
			// grid points" (if on) which always get a single flat image each - see
			// overviewCapturePoints(). imageNumber stays unique per (z, point, stack-slice)
			// triple since the point count and each point's stack depth are constant across
			// z slices (must match the flat-index scheme the "overview-brightfield-x/y/z"
			// metadata above uses).
			const auto capturePoints = overviewCapturePoints(zIndex, directionsZ);
			size_t totalPerZ = 0;
			for (const auto& point : capturePoints) {
				totalPerZ += point.zAbs.size();
			}
			auto flatIndexWithinZ = size_t{ 0 };
			for (const auto& point : capturePoints) {
				for (const auto z : point.zAbs) {
					const auto imageNumber = (int)((size_t)zIndex * totalPerZ + flatIndexWithinZ);
					const auto position = POINT3{ point.xy.x, point.xy.y, z };
					captureOverviewBrightfield(storage, imageNumber, zIndex, position);
					flatIndexWithinZ++;
					if (m_abort) {
						return;
					}
				}
			}
			if (m_scanControl) {
				// The overview brightfield capture moves the stage away from the grid point,
				// so approach it again from a consistent direction to avoid hysteresis error.
				approachGridPosition(m_orderedPositions[ll]);
				std::this_thread::sleep_for(std::chrono::milliseconds(100));
			} else {
				m_abort = true;
				return;
			}
		}

		// move stage to next position
		if (ll < ((gsl::index)nrPositions - 1)) {
			if (m_scanControl) {
				approachGridPosition(m_orderedPositions[ll + 1]);
				// See the matching comment where position 0 is approached above.
				m_scanControl->announcePosition();
			} else {
				m_abort = true;
				return;
			}
		}

		auto percentage{ 100 * (double)(ll + 1) / nrPositions };
		auto remaining{ (int)(1e-3 * measurementTimer.elapsed() / (ll + 1) * ((int64_t)nrPositions - ll + 1)) };
		emit(s_repetitionProgress(percentage, remaining));
	}
	// do post calibration
	if (m_settings.postCalibration) {
		calibrate(storage);
	}

	// close camera libraries, clear buffers
	if (m_andor) {
		m_andor->stopAcquisition();
	} else {
		m_abort = true;
		return;
	}

	if (m_scanControl) {
		m_scanControl->setPreset(ScanPreset::SCAN_LASEROFF);
		// Acquisition has finished - don't leave the RL shutter forced open.
		m_scanControl->setRLShutterOpen(false);

		m_scanControl->setPositionCompensated(m_startPosition);
		m_scanControl->enableMeasurementMode(false);
		emit(s_positionChanged({ 0, 0, 0 }, 0));
		QMetaObject::invokeMethod(
			m_scanControl,
			[scanControl = m_scanControl]() { scanControl->startAnnouncing(); },
			Qt::AutoConnection
		);
	} else {
		m_abort = true;
		return;
	}

	// Here we wait until the storage object indicate it finished to write to the file.
	QEventLoop loop;
	auto connection = QWidget::connect(
		storage.get(),
		&StorageWrapper::finished,
		&loop,
		&QEventLoop::quit
	);
	QMetaObject::invokeMethod(
		storage.get(),
		[&storage = storage]() { storage.get()->s_finishedQueueing(); },
		Qt::AutoConnection
	);
	loop.exec();

	auto info = std::string{ "Acquisition finished." };
	qInfo(logInfo()) << info.c_str();
	emit(s_calibrationRunning(false));
	setAcquisitionStatus(ACQUISITION_STATUS::FINISHED);
	emit(s_timeToCalibration(0));
}

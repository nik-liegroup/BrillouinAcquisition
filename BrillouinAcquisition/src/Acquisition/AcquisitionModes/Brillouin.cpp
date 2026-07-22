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

	// set exposure time for calibration
	if (m_andor) {
		m_andor->setCalibrationExposureTime(m_settings.calibrationExposureTime);
	}

	// move optical elements to position for calibration
	if (m_scanControl) {
		m_scanControl->setPreset(ScanPreset::SCAN_CALIBRATION);
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
	}

	// reset exposure time
	if (m_andor) {
		m_andor->setCalibrationExposureTime(m_settings.camera.exposureTime);
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
	plannerInput.absoluteGridOriginUm = settings.absoluteGridOriginUm;

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

	appendMetric(
		m_settings.surfaceProxyRoiLeft,
		m_settings.surfaceProxyRoiTop,
		m_settings.surfaceProxyRoiWidth,
		m_settings.surfaceProxyRoiHeight
	);
	appendMetric(
		m_settings.surfaceProxyRoi2Left,
		m_settings.surfaceProxyRoi2Top,
		m_settings.surfaceProxyRoi2Width,
		m_settings.surfaceProxyRoi2Height
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

Brillouin::SurfaceScanResult Brillouin::runSurfacePreScan() {
	// Cleared up front so every exit path - including the early-out ones below and an
	// abort partway through - leaves these reflecting only a scan that actually completed,
	// never stale found/interpolated data from an earlier, unrelated run.
	m_surfaceFoundXYIndices.clear();
	m_surfaceInterpolatedXYIndices.clear();

	if (!m_scanControl || !m_andor) {
		return {};
	}

	const auto xyBin = std::max(1, m_settings.preScanXYBin);
	const auto xStepsCoarse = std::max(1, (m_settings.xSteps + xyBin - 1) / xyBin);
	const auto yStepsCoarse = std::max(1, (m_settings.ySteps + xyBin - 1) / xyBin);
	const auto zTravel = std::max(0.0, m_settings.preScanZTravelRangeUm);
	const auto zStep = std::max(1e-6, m_settings.preScanZStepUm);
	const auto zStepsCoarse = std::max(2, (int)std::floor(zTravel / zStep) + 1);

	if (xStepsCoarse < 1 || yStepsCoarse < 1 || zStepsCoarse < 2) {
		return {};
	}

	const auto xSamples = simplemath::linspace(m_settings.xMin, m_settings.xMax, xStepsCoarse);
	const auto ySamples = simplemath::linspace(m_settings.yMin, m_settings.yMax, yStepsCoarse);

	std::vector<std::vector<double>> zSurface(ySamples.size(), std::vector<double>(xSamples.size(), 0.0));
	std::vector<std::vector<bool>> zSurfaceValid(ySamples.size(), std::vector<bool>(xSamples.size(), false));
	// Order each column was found in (-1 = not found yet), used to break seed-distance ties
	// in favor of whichever neighbor was measured most recently.
	std::vector<std::vector<int>> processOrder(ySamples.size(), std::vector<int>(xSamples.size(), -1));
	auto frame = std::vector<std::byte>(m_settings.camera.roi.bytesPerFrame);
	// Rough estimate only for the progress percentage - the neighbor-seeded search below
	// takes a variable number of steps per column, unlike the old fixed full sweep.
	const auto totalSurfaceSteps = std::max(1, (int)(xSamples.size() * ySamples.size() * zStepsCoarse));
	int completedSurfaceSteps = 0;
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
			const auto zOrigin = m_settings.gridCoordinatesAbsolute
				? m_settings.absoluteGridOriginUm.z
				: m_startPosition.z;
			referencePosition = m_settings.gridCoordinatesAbsolute
				? POINT3{ xSamples[xi] + m_settings.absoluteGridOriginUm.x, ySamples[yi] + m_settings.absoluteGridOriginUm.y, zOrigin }
				: POINT3{ m_startPosition.x + xSamples[xi], m_startPosition.y + ySamples[yi], zOrigin };
			referencePositionFound = true;
			break;
		}
	}
	if (referencePositionFound) {
		m_scanControl->setPositionCompensated(referencePosition);
		std::this_thread::sleep_for(std::chrono::milliseconds(20));
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

	// Measures the surface-drop metric at coarse column (xi, yi) and relative z `zRel`,
	// averaging `frameAverage` frames at that position to reduce noise (1 = single frame,
	// used for the fast initial walk; > 1 used during verification). Moves the stage there
	// first - setPositionCompensated() only pre-approaches x/y when they actually change,
	// so repeated calls within a column (z-only steps) add no extra motion. Returns
	// std::nullopt if the acquisition was aborted mid-measurement.
	auto measureAt = [&](gsl::index xi, gsl::index yi, double zRel, int frameAverage) -> std::optional<double> {
		if (m_abort) {
			return std::nullopt;
		}
		const auto xyPosition = m_settings.gridCoordinatesAbsolute
			? POINT2{ xSamples[xi] + m_settings.absoluteGridOriginUm.x, ySamples[yi] + m_settings.absoluteGridOriginUm.y }
			: POINT2{ m_startPosition.x + xSamples[xi], m_startPosition.y + ySamples[yi] };
		const auto zOrigin = m_settings.gridCoordinatesAbsolute
			? m_settings.absoluteGridOriginUm.z
			: m_startPosition.z;
		const auto target = POINT3{ xyPosition.x, xyPosition.y, zOrigin + zRel };
		m_scanControl->setPositionCompensated(target);
		std::this_thread::sleep_for(std::chrono::milliseconds(20));
		const auto frames = std::max(1, frameAverage);
		double sum = 0.0;
		for (int f = 0; f < frames; f++) {
			if (m_abort) {
				return std::nullopt;
			}
			m_andor->getImageForAcquisition(frame.data());
			sum += estimateFrameMetric(frame);
		}
		completedSurfaceSteps++;
		return sum / frames;
	};

	auto emitSurfaceProgress = [&](const QString& message) {
		const auto progress = std::clamp(5.0 + 95.0 * (double)completedSurfaceSteps / totalSurfaceSteps, 5.0, 99.9);
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
			if (!exactMatch && weightNorm <= 0.0) {
				continue;
			}
			const auto zInterp = exactMatch ? exactZ : (weightedSum / weightNorm);

			const auto zOrigin = m_settings.gridCoordinatesAbsolute
				? m_settings.absoluteGridOriginUm.z
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

	const auto zMid = 0.5 * (m_settings.zMin + m_settings.zMax);
	const auto halfRange = std::max(0.0, m_settings.surfaceFollowHalfRangeUm);
	auto localZOffsets = simplemath::linspace(-halfRange, halfRange, m_settings.zSteps);

	for (gsl::index ll{ 0 }; ll < (gsl::index)m_orderedPositions.size(); ll++) {
		const auto ix = m_orderedIndices[ll].x;
		const auto iy = m_orderedIndices[ll].y;
		const auto it = zCenterByXYIndex.find({ ix, iy });
		if (it == zCenterByXYIndex.end()) {
			continue;
		}
		const auto zIdx = std::clamp(m_orderedIndices[ll].z, 0, (int)localZOffsets.size() - 1);
		auto localRel = m_orderedPositionsRelative[ll].z - zMid;
		if (halfRange > 0.0) {
			localRel = localZOffsets[zIdx];
		}
		auto zAbs = it->second + localRel;
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

POINT3 Brillouin::overviewBrightfieldPositionForZ(int zIndex, const std::vector<double>& directionsZ, const POINT2& xy) const {
	const auto origin = m_settings.gridCoordinatesAbsolute ? m_settings.absoluteGridOriginUm : m_startPosition;
	const auto clampedZIndex = std::clamp(zIndex, 0, (int)directionsZ.size() - 1);

	auto position = POINT3{
		xy.x,
		xy.y,
		origin.z + directionsZ[clampedZIndex]
	};

	if (m_settings.useSurfaceFollow) {
		auto bestDistance2 = std::numeric_limits<double>::infinity();
		for (gsl::index ii{ 0 }; ii < (gsl::index)m_orderedIndices.size(); ii++) {
			if (m_orderedIndices[ii].z == clampedZIndex) {
				const auto dx = m_orderedPositions[ii].x - position.x;
				const auto dy = m_orderedPositions[ii].y - position.y;
				const auto distance2 = dx * dx + dy * dy;
				if (distance2 < bestDistance2) {
					bestDistance2 = distance2;
					position.z = m_orderedPositions[ii].z;
				}
			}
		}
	}

	return position;
}

/*
 * One position per z slice by default (at the current start position, matching the
 * legacy behaviour), or one position per mosaic tile when overviewBrightfieldFullGrid
 * is enabled in absolute grid mode, so the whole grid extent gets covered.
 */
std::vector<POINT3> Brillouin::overviewBrightfieldPositionsForZ(int zIndex, const std::vector<double>& directionsZ) const {
	std::vector<POINT3> positions;
	if (m_settings.overviewBrightfieldFullGrid && m_settings.gridCoordinatesAbsolute) {
		const auto tileCentersXY = overviewTileCentersXY();
		positions.reserve(tileCentersXY.size());
		for (const auto& xy : tileCentersXY) {
			positions.push_back(overviewBrightfieldPositionForZ(zIndex, directionsZ, xy));
		}
	} else {
		positions.push_back(overviewBrightfieldPositionForZ(zIndex, directionsZ, POINT2{ m_startPosition.x, m_startPosition.y }));
	}
	return positions;
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
	const auto origin = m_settings.gridCoordinatesAbsolute ? m_settings.absoluteGridOriginUm : POINT3{};
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

std::vector<std::pair<POINT2, POINT2>> Brillouin::overviewTileOutlinesUm() const {
	// See overviewTileCentersXY() for why relative mode must use m_orderedPositionsRelative
	// and a zero origin here, not m_orderedPositions/m_startPosition.
	const auto& positions = m_settings.gridCoordinatesAbsolute ? m_orderedPositions : m_orderedPositionsRelative;
	const auto origin = m_settings.gridCoordinatesAbsolute ? m_settings.absoluteGridOriginUm : POINT3{};
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
	const std::vector<std::byte>& image
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
		cameraSettings.roi
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
	m_scanControl->setPositionCompensated(position);
	std::this_thread::sleep_for(std::chrono::milliseconds(100));

	auto brightfieldSettings = m_brightfieldCamera->getSettings();
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
		enqueueOverviewBrightfieldImage<unsigned short>(storage, imageNumber, brightfieldSettings, image);
		queuedImage = true;
	} else if (brightfieldSettings.readout.dataType == "unsigned char") {
		enqueueOverviewBrightfieldImage<unsigned char>(storage, imageNumber, brightfieldSettings, image);
		queuedImage = true;
	}

	if (queuedImage) {
		emit(s_surfaceScanProgress(
			100.0 * (double)(zIndex + 1) / std::max(1, m_settings.zSteps),
			QString("Saved brightfield overview for z slice %1/%2").arg(zIndex + 1).arg(m_settings.zSteps)
		));
	}

	m_scanControl->setPreset(ScanPreset::SCAN_BRILLOUIN);
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
	for (gsl::index ll{ 0 }; ll < (gsl::index)m_orderedPositions.size(); ll++) {
		sampledX[ll] = m_settings.gridCoordinatesAbsolute
			? m_orderedPositions[ll].x - m_settings.absoluteGridOriginUm.x
			: m_orderedPositions[ll].x;
		sampledY[ll] = m_settings.gridCoordinatesAbsolute
			? m_orderedPositions[ll].y - m_settings.absoluteGridOriginUm.y
			: m_orderedPositions[ll].y;
		sampledZ[ll] = m_settings.gridCoordinatesAbsolute
			? m_orderedPositions[ll].z - m_settings.absoluteGridOriginUm.z
			: m_orderedPositions[ll].z;
	}
	storage->setPositions("sampled-x", sampledX, sampledRank, sampledDims);
	storage->setPositions("sampled-y", sampledY, sampledRank, sampledDims);
	storage->setPositions("sampled-z", sampledZ, sampledRank, sampledDims);

	if (m_settings.saveOverviewBrightfieldPerZ) {
		// Flattened as [z0_tile0, z0_tile1, ..., z1_tile0, ...]; tileCount is constant
		// across z since the tile layout only depends on the (z-independent) grid
		// extent. overview-brightfield-tile-count lets a reader reshape this back into
		// [zSteps, tileCount] and is 1 for the legacy single-image-per-z behaviour.
		const auto tileCount = overviewBrightfieldPositionsForZ(0, directionsZ).size();
		const auto totalOverviewCount = (size_t)m_settings.zSteps * tileCount;
		hsize_t overviewDims[1] = { (hsize_t)totalOverviewCount };
		auto overviewX = std::vector<double>(totalOverviewCount);
		auto overviewY = std::vector<double>(totalOverviewCount);
		auto overviewZ = std::vector<double>(totalOverviewCount);
		for (gsl::index ii{ 0 }; ii < m_settings.zSteps; ii++) {
			const auto positions = overviewBrightfieldPositionsForZ((int)ii, directionsZ);
			for (size_t tt = 0; tt < positions.size() && tt < tileCount; tt++) {
				const auto flatIndex = (size_t)ii * tileCount + tt;
				const auto& position = positions[tt];
				// Use the same convention as sampled-x/y/z above, so overview and
				// Brillouin positions can be compared/overlaid directly without the
				// caller having to know which fields are absolute vs. origin-relative.
				overviewX[flatIndex] = m_settings.gridCoordinatesAbsolute
					? position.x - m_settings.absoluteGridOriginUm.x
					: position.x;
				overviewY[flatIndex] = m_settings.gridCoordinatesAbsolute
					? position.y - m_settings.absoluteGridOriginUm.y
					: position.y;
				overviewZ[flatIndex] = m_settings.gridCoordinatesAbsolute
					? position.z - m_settings.absoluteGridOriginUm.z
					: position.z;
			}
		}
		storage->setPositions("overview-brightfield-x", overviewX, sampledRank, overviewDims);
		storage->setPositions("overview-brightfield-y", overviewY, sampledRank, overviewDims);
		storage->setPositions("overview-brightfield-z", overviewZ, sampledRank, overviewDims);
		const hsize_t tileCountDims[1] = { 1 };
		storage->setPositions("overview-brightfield-tile-count", std::vector<double>{ (double)tileCount }, 1, tileCountDims);
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
		// Approach from a consistent direction to compensate for stage hysteresis,
		// so the grid is reached reproducibly regardless of where the stage was before.
		m_scanControl->setPositionCompensated(m_orderedPositions[0]);
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
					m_scanControl->setPositionCompensated(m_orderedPositions[ll]);
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
				? m_orderedPositions[ll] - m_settings.absoluteGridOriginUm
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
			// One position (legacy behaviour) or one per mosaic tile covering the full
			// grid extent, depending on overviewBrightfieldFullGrid; imageNumber stays
			// unique per (z, tile) pair since tileCount is constant across z slices.
			const auto overviewPositions = overviewBrightfieldPositionsForZ(zIndex, directionsZ);
			for (size_t tt = 0; tt < overviewPositions.size(); tt++) {
				const auto imageNumber = zIndex * (int)overviewPositions.size() + (int)tt;
				captureOverviewBrightfield(storage, imageNumber, zIndex, overviewPositions[tt]);
				if (m_abort) {
					return;
				}
			}
			if (m_scanControl) {
				// The overview brightfield capture moves the stage away from the grid point,
				// so approach it again from a consistent direction to avoid hysteresis error.
				m_scanControl->setPositionCompensated(m_orderedPositions[ll]);
				std::this_thread::sleep_for(std::chrono::milliseconds(100));
			} else {
				m_abort = true;
				return;
			}
		}

		// move stage to next position
		if (ll < ((gsl::index)nrPositions - 1)) {
			if (m_scanControl) {
				m_scanControl->setPositionCompensated(m_orderedPositions[ll + 1]);
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

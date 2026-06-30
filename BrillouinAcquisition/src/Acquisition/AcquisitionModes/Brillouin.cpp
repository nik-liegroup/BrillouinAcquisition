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
#include <algorithm>
#include <cmath>

using namespace std::filesystem;

namespace {
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

		setAcquisitionStatus(ACQUISITION_STATUS::STARTED);
		acquire(m_acquisition->m_storage);

		if (m_abort) {
			this->abortMode(m_acquisition->m_storage);
			return;
		}
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
	} else {
		timeSinceLast = 1e-3 * m_startOfLastRepetition.elapsed();
		emit(s_totalProgress(m_currentRepetition, m_settings.repetitions.interval * 60 - timeSinceLast));
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
		m_scanControl->setPosition(m_startPosition);
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

	if (m_settings.gridCoordinatesAbsolute) {
		emit(s_orderedPositionsChanged(m_orderedPositions));
	} else {
		emit(s_orderedPositionsChanged(m_orderedPositionsRelative));
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

		double signalSum{ 0.0 };
		size_t signalCount{ 0 };
		for (int displayY = roiDisplayBottom; displayY < roiDisplayBottom + roiHeight; displayY++) {
			for (int x = roiLeft; x < roiLeft + roiWidth; x++) {
				signalSum += getDisplayValue(x, displayY);
				signalCount++;
			}
		}
		if (signalCount == 0) {
			return;
		}
		metrics.push_back(signalSum / (double)signalCount);
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

bool Brillouin::runSurfacePreScan() {
	if (!m_scanControl || !m_andor) {
		return false;
	}

	const auto xyBin = std::max(1, m_settings.preScanXYBin);
	const auto xStepsCoarse = std::max(1, (m_settings.xSteps + xyBin - 1) / xyBin);
	const auto yStepsCoarse = std::max(1, (m_settings.ySteps + xyBin - 1) / xyBin);
	const auto zTravel = std::max(0.0, m_settings.preScanZTravelRangeUm);
	const auto zStep = std::max(1e-6, m_settings.preScanZStepUm);
	const auto zStepsCoarse = std::max(2, (int)std::floor(zTravel / zStep) + 1);

	if (xStepsCoarse < 1 || yStepsCoarse < 1 || zStepsCoarse < 2) {
		return false;
	}

	const auto xSamples = simplemath::linspace(m_settings.xMin, m_settings.xMax, xStepsCoarse);
	const auto ySamples = simplemath::linspace(m_settings.yMin, m_settings.yMax, yStepsCoarse);
	// Always scan positive z direction from current position towards sample.
	auto zSamples = simplemath::linspace(0.0, zTravel, zStepsCoarse);

	std::vector<std::vector<double>> zSurface(ySamples.size(), std::vector<double>(xSamples.size(), 0.0));
	std::vector<std::vector<bool>> zSurfaceValid(ySamples.size(), std::vector<bool>(xSamples.size(), false));
	auto frame = std::vector<std::byte>(m_settings.camera.roi.bytesPerFrame);
	const auto totalSurfaceSteps = std::max(1, (int)(xSamples.size() * ySamples.size() * zSamples.size()));
	int completedSurfaceSteps = 0;

	// Measure medium reference before scanning if requested.
	if (m_settings.useMediumReference) {
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
			m_scanControl->setPosition(referencePosition);
			std::this_thread::sleep_for(std::chrono::milliseconds(20));
		}
		const int refFrames = std::max(1, m_settings.mediumReferenceFrameCount);
		double refSum = 0.0;
		for (int i = 0; i < refFrames; i++) {
			if (m_abort) {
				return false;
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
	}

	const auto dropFraction = std::clamp(m_settings.surfaceDropFraction, 0.0, 0.99);
	const auto referenceThreshold = (m_settings.useMediumReference && m_settings.mediumReferenceValue > 1e-12)
		? (1.0 - dropFraction) * m_settings.mediumReferenceValue
		: std::numeric_limits<double>::quiet_NaN();

	for (gsl::index yi{ 0 }; yi < (gsl::index)ySamples.size(); yi++) {
		for (gsl::index xi{ 0 }; xi < (gsl::index)xSamples.size(); xi++) {
			const POINT2 coarsePoint{ xSamples[xi], ySamples[yi] };
			if (m_settings.useRoiMask && !isPointInPolygonUm(coarsePoint, m_settings.roiPolygonUm)) {
				continue;
			}
			bool foundSurface = false;
			size_t foundIndex = 0;

			for (gsl::index zi{ 0 }; zi < (gsl::index)zSamples.size(); zi++) {
				if (m_abort) {
					return false;
				}
				const auto zRel = zSamples[zi];
				const auto xyPosition = [&]() {
					if (m_settings.gridCoordinatesAbsolute) {
						return POINT3{
							xSamples[xi] + m_settings.absoluteGridOriginUm.x,
							ySamples[yi] + m_settings.absoluteGridOriginUm.y,
							m_settings.absoluteGridOriginUm.z
						};
					}
					return POINT3{ m_startPosition.x + xSamples[xi], m_startPosition.y + ySamples[yi], 0.0 };
				}();
				const auto zOrigin = m_settings.gridCoordinatesAbsolute
					? m_settings.absoluteGridOriginUm.z
					: m_startPosition.z;
				const auto target = POINT3{ xyPosition.x, xyPosition.y, zOrigin + zRel };
				m_scanControl->setPosition(target);
				std::this_thread::sleep_for(std::chrono::milliseconds(20));
				m_andor->getImageForAcquisition(frame.data());
				const auto metric = estimateFrameMetric(frame);

				completedSurfaceSteps++;
				const auto progress = 5.0 + 95.0 * (double)completedSurfaceSteps / totalSurfaceSteps;
				emit(s_surfaceScanProgress(
					progress,
					QString("Surface scan %1%: x %2/%3, y %4/%5, z %6/%7, metric %8, threshold %9")
						.arg(progress, 0, 'f', 1)
						.arg((int)xi + 1)
						.arg((int)xSamples.size())
						.arg((int)yi + 1)
						.arg((int)ySamples.size())
						.arg((int)zi + 1)
						.arg((int)zSamples.size())
						.arg(metric, 0, 'f', 3)
						.arg(referenceThreshold, 0, 'f', 3)
				));

				// Stop as soon as the ROI metric has dropped enough from the measured
				// medium reference, but require three consecutive below-threshold frames
				// at the same z position to avoid accepting a single noisy drop.
				auto referenceDrop = std::isfinite(referenceThreshold) && metric <= referenceThreshold;
				if (referenceDrop) {
					for (int confirmationFrame = 0; confirmationFrame < 2; confirmationFrame++) {
						if (m_abort) {
							return false;
						}
						m_andor->getImageForAcquisition(frame.data());
						const auto confirmationMetric = estimateFrameMetric(frame);
						const auto confirmationDrop = confirmationMetric <= referenceThreshold;
						emit(s_surfaceScanProgress(
							progress,
							QString("Surface confirmation %1/2 at x %2/%3, y %4/%5, z step %6/%7: metric %8, threshold %9")
								.arg(confirmationFrame + 1)
								.arg((int)xi + 1)
								.arg((int)xSamples.size())
								.arg((int)yi + 1)
								.arg((int)ySamples.size())
								.arg((int)zi + 1)
								.arg((int)zSamples.size())
								.arg(confirmationMetric, 0, 'f', 3)
								.arg(referenceThreshold, 0, 'f', 3)
						));
						if (!confirmationDrop) {
							referenceDrop = false;
							break;
						}
					}
				}
				if (referenceDrop) {
					foundSurface = true;
					foundIndex = (size_t)zi;
					emit(s_surfaceScanProgress(
						progress,
						QString("Surface found at x %1/%2, y %3/%4, z step %5/%6: metric %7 <= threshold %8")
							.arg((int)xi + 1)
							.arg((int)xSamples.size())
							.arg((int)yi + 1)
							.arg((int)ySamples.size())
							.arg((int)zi + 1)
							.arg((int)zSamples.size())
							.arg(metric, 0, 'f', 3)
							.arg(referenceThreshold, 0, 'f', 3)
					));
					break;
				}
			}

			if (!foundSurface) {
				emit(s_surfaceScanProgress(
					5.0 + 95.0 * (double)completedSurfaceSteps / totalSurfaceSteps,
					QString("No surface drop found at x %1/%2, y %3/%4")
						.arg((int)xi + 1)
						.arg((int)xSamples.size())
						.arg((int)yi + 1)
						.arg((int)ySamples.size())
				));
				continue;
			}

			zSurface[yi][xi] = zSamples[foundIndex];
			zSurfaceValid[yi][xi] = true;
		}
	}

	// Smooth coarse surface map with a small local mean filter.
	const auto smoothingPasses = std::max(0, (int)std::round(m_settings.surfaceSmoothSigmaUm / 5.0));
	for (int pass = 0; pass < smoothingPasses; pass++) {
		auto smoothed = zSurface;
		for (gsl::index yi{ 0 }; yi < (gsl::index)zSurface.size(); yi++) {
			for (gsl::index xi{ 0 }; xi < (gsl::index)zSurface[yi].size(); xi++) {
				double sum = 0.0;
				int count = 0;
				for (int dy = -1; dy <= 1; dy++) {
					for (int dx = -1; dx <= 1; dx++) {
						const auto ny = yi + dy;
						const auto nx = xi + dx;
						if (ny >= 0 && ny < (gsl::index)zSurface.size()
							&& nx >= 0 && nx < (gsl::index)zSurface[yi].size()) {
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

	// Bilinear interpolation on coarse XY map; nearest sample on boundaries.
	const auto xDense = simplemath::linspace(m_settings.xMin, m_settings.xMax, m_settings.xSteps);
	const auto yDense = simplemath::linspace(m_settings.yMin, m_settings.yMax, m_settings.ySteps);
	std::map<std::pair<int, int>, double> zCenterByXYIndex;

	for (gsl::index yi{ 0 }; yi < (gsl::index)yDense.size(); yi++) {
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
			for (gsl::index yc{ 0 }; yc < (gsl::index)ySamples.size(); yc++) {
				for (gsl::index xc{ 0 }; xc < (gsl::index)xSamples.size(); xc++) {
					if (!zSurfaceValid[yc][xc]) {
						continue;
					}
					const auto dx = x - xSamples[xc];
					const auto dy = y - ySamples[yc];
					const auto d2 = dx * dx + dy * dy;
					if (d2 <= 1e-12) {
						exactMatch = true;
						exactZ = zSurface[yc][xc];
						break;
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
		}
	}

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

	return true;
}

void Brillouin::applySurfaceFollowPlan() {
	if (!m_settings.useSurfaceFollow) {
		return;
	}
	if (runSurfacePreScan()) {
		if (m_settings.gridCoordinatesAbsolute) {
			emit(s_orderedPositionsChanged(m_orderedPositions));
		} else {
			emit(s_orderedPositionsChanged(m_orderedPositionsRelative));
		}
	}
}

POINT3 Brillouin::overviewBrightfieldPositionForZ(int zIndex, const std::vector<double>& directionsZ) const {
	const auto origin = m_settings.gridCoordinatesAbsolute ? m_settings.absoluteGridOriginUm : m_startPosition;
	const auto clampedZIndex = std::clamp(zIndex, 0, (int)directionsZ.size() - 1);

	auto position = POINT3{
		origin.x + 0.5 * (m_settings.xMin + m_settings.xMax),
		origin.y + 0.5 * (m_settings.yMin + m_settings.yMax),
		origin.z + directionsZ[clampedZIndex]
	};

	if (m_settings.useSurfaceFollow) {
		for (gsl::index ii{ 0 }; ii < (gsl::index)m_orderedIndices.size(); ii++) {
			if (m_orderedIndices[ii].z == clampedZIndex) {
				position.z = m_orderedPositions[ii].z;
				break;
			}
		}
	}

	return position;
}

template <typename T>
void enqueueOverviewBrightfieldImage(
	std::unique_ptr<StorageWrapper>& storage,
	int imageNumber,
	CAMERA_SETTINGS& cameraSettings,
	const std::vector<std::byte>& image
) {
	auto image_ = (std::vector<T>*) &image;
	auto date = QDateTime::currentDateTime().toOffsetFromUtc(QDateTime::currentDateTime().offsetFromUtc())
		.toString(Qt::ISODateWithMs).toStdString();
	int rankData{ 3 };
	auto dimsData = new hsize_t[3]{ 1, (hsize_t)cameraSettings.roi.height_binned, (hsize_t)cameraSettings.roi.width_binned };
	auto img = new FLUOIMAGE<T>(
		imageNumber,
		rankData,
		dimsData,
		date,
		"Brightfield overview",
		*image_,
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
	m_scanControl->setPosition(position);
	std::this_thread::sleep_for(std::chrono::milliseconds(100));

	auto brightfieldSettings = m_brightfieldCamera->getSettings();
	brightfieldSettings.exposureTime = 1e-3 * std::max(1, m_settings.overviewBrightfieldExposureMs);
	brightfieldSettings.gain = m_settings.overviewBrightfieldGain;
	brightfieldSettings.frameCount = 1;
	brightfieldSettings.readout.triggerMode = L"Software";
	brightfieldSettings.readout.cycleMode = L"Fixed";

	m_brightfieldCamera->startAcquisition(brightfieldSettings);
	brightfieldSettings = m_brightfieldCamera->getSettings();

	std::vector<std::byte> image(brightfieldSettings.roi.bytesPerFrame);
	m_brightfieldCamera->getImageForAcquisition(&image[0], true);
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

	/*
	 * Update the positions vector
	 */
	updatePositions();
	applySurfaceFollowPlan();

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
	auto sampledMask = std::vector<double>(gridPositionCount, 0.0);
	for (gsl::index ll{ 0 }; ll < (gsl::index)m_orderedIndices.size(); ll++) {
		const auto idx = m_orderedIndices[ll];
		const auto flat = idx.z * (m_settings.xSteps * m_settings.ySteps) + idx.y * m_settings.xSteps + idx.x;
		if (flat >= 0 && flat < gridPositionCount) {
			sampledMask[flat] = 1.0;
		}
	}
	storage->setPositions("sampled-mask", sampledMask, rank, dims);

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
		hsize_t overviewDims[1] = { (hsize_t)m_settings.zSteps };
		auto overviewX = std::vector<double>(m_settings.zSteps);
		auto overviewY = std::vector<double>(m_settings.zSteps);
		auto overviewZ = std::vector<double>(m_settings.zSteps);
		for (gsl::index ii{ 0 }; ii < m_settings.zSteps; ii++) {
			const auto position = overviewBrightfieldPositionForZ((int)ii, directionsZ);
			overviewX[ii] = m_settings.gridCoordinatesAbsolute ? position.x - m_settings.absoluteGridOriginUm.x : position.x;
			overviewY[ii] = m_settings.gridCoordinatesAbsolute ? position.y - m_settings.absoluteGridOriginUm.y : position.y;
			overviewZ[ii] = m_settings.gridCoordinatesAbsolute ? position.z - m_settings.absoluteGridOriginUm.z : position.z;
		}
		storage->setPositions("overview-brightfield-x", overviewX, sampledRank, overviewDims);
		storage->setPositions("overview-brightfield-y", overviewY, sampledRank, overviewDims);
		storage->setPositions("overview-brightfield-z", overviewZ, sampledRank, overviewDims);
	}
	delete[] dims;

	bool zSafetyWarned{ false };
	auto warnIfZUnsafe = [this, &zSafetyWarned](const POINT3& position) {
		if (!m_settings.useSurfaceFollow || !m_settings.useMaxSafeZSafety || zSafetyWarned) {
			return;
		}
		if (position.z > m_settings.maxSafeZUm) {
			zSafetyWarned = true;
			emit(s_surfaceZSafetyWarning(position.z, m_settings.maxSafeZUm));
		}
	};

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
	auto overviewCapturedForZ = std::vector<bool>(m_settings.zSteps, false);

	// move stage to first position, wait 50 ms for it to finish
	if (m_scanControl) {
		warnIfZUnsafe(m_orderedPositions[0]);
		m_scanControl->setPosition(m_orderedPositions[0]);
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
				// After we calibrated, we move back to the current position
				if (m_scanControl) {
					warnIfZUnsafe(m_orderedPositions[ll]);
					m_scanControl->setPosition(m_orderedPositions[ll]);
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
		if (m_settings.saveOverviewBrightfieldPerZ && !overviewCapturedForZ[zIndex]) {
			overviewCapturedForZ[zIndex] = true;
			captureOverviewBrightfield(storage, zIndex, zIndex, overviewBrightfieldPositionForZ(zIndex, directionsZ));
			if (m_abort) {
				return;
			}
			if (m_scanControl) {
				warnIfZUnsafe(m_orderedPositions[ll]);
				m_scanControl->setPosition(m_orderedPositions[ll]);
				std::this_thread::sleep_for(std::chrono::milliseconds(100));
			} else {
				m_abort = true;
				return;
			}
		}

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

		// move stage to next position
		if (ll < ((gsl::index)nrPositions - 1)) {
			if (m_scanControl) {
				warnIfZUnsafe(m_orderedPositions[ll + 1]);
				m_scanControl->setPosition(m_orderedPositions[ll + 1]);
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

		m_scanControl->setPosition(m_startPosition);
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

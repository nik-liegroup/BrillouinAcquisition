#include "stdafx.h"
#include "Brillouin.h"
#include "BrillouinResume.h"
#include "ResumeFile.h"
#include "src/helper/logger.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <set>
#include <tuple>
#include <thread>

/*
 * Continue of an interrupted measurement: crash, cancel, pause. Everything needed is in the file of the measurement (what is written and how it
 * is read back is in ResumeFile.h):
 *  - the state of the repetition ("preparing", "surface-scan", "measuring", "stopped", "finished", ... - attribute acquisition_state),
 *  - the settings in full, the start position, the objective, the plan (positions in measuring order, z as the surface scan left them),
 *  - which points are in the file (the points themselves), and the overview events and surface rescans of the engine.
 * A continue appends to the same repetition, starts at the first point that is not complete in the file and notes in the file where and when
 * the measurement was stopped (resume-event-N-*). It is never started by itself: the operator loads the file and presses Continue.
 */

bool Brillouin::readResumeInfo(StorageWrapper* storage, ResumeInfo* info, std::string* problem) {
	return resume::readInfo(storage, ACQUISITION_MODE::BRILLOUIN, resume::settingNames(true), resume::settingNames(false), info, problem);
}

void Brillouin::setStorageState(const std::string& state) {
	auto* storage = m_acquisition ? m_acquisition->m_storage.get() : nullptr;
	if (!storage) {
		return;
	}
	QMetaObject::invokeMethod(storage, [storage, state]() {
		storage->setRepetitionState(ACQUISITION_MODE::BRILLOUIN, state);
	}, Qt::BlockingQueuedConnection);
}

void Brillouin::writeResumePlan(std::unique_ptr<StorageWrapper>& storage, bool direct) {
	auto* target = storage.get();
	resume::Plan plan;
	plan.positions = m_orderedPositions;
	plan.positionsRelative = m_orderedPositionsRelative;
	plan.indices = m_orderedIndices;
	plan.calibrationAllowed = m_calibrationAllowed;
	if (direct) {
		resume::writePlan(target, plan);
	} else {
		// mid-measurement the storage thread is busy writing: the plan goes through its queue
		QMetaObject::invokeMethod(target, [target, plan]() { resume::writePlan(target, plan); }, Qt::AutoConnection);
	}
}

void Brillouin::writeResumeBasics(std::unique_ptr<StorageWrapper>& storage) {
	auto* target = storage.get();
	const auto fluorescence = overviewImagesExpected() ? target->getModeHandle(ACQUISITION_MODE::FLUORESCENCE)->repetitionCount : -1;
	resume::writeBasics(target, resume::encodeSettings(m_settings), m_startPosition, m_scanControl ? m_scanControl->getActiveObjectiveSlot() : -1,
		m_currentRepetition, fluorescence, m_scanOrder.automatical, m_scanOrder.x, m_scanOrder.y, m_scanOrder.z);
}

void Brillouin::writeResumeRecord(std::unique_ptr<StorageWrapper>& storage) {
	writeResumeBasics(storage);
	writeResumePlan(storage, true);
}

void Brillouin::writeSurfaceCheckpointReference(double mediumReference, size_t columnsX, size_t columnsY) {
	auto* storage = m_acquisition ? m_acquisition->m_storage.get() : nullptr;
	if (!storage) {
		return;
	}
	resume::writeSurfaceReference(storage, mediumReference, m_settings.surfaceDropFraction, columnsX, columnsY);
	storage->flushFile();
}

void Brillouin::writeSurfaceCheckpointColumn(gsl::index xi, gsl::index yi, bool valid, double z, int order, double metric,
	const std::vector<double>& curveZ, const std::vector<double>& curveMetric) {
	auto* storage = m_acquisition ? m_acquisition->m_storage.get() : nullptr;
	if (!storage) {
		return;
	}
	resume::writeSurfaceColumn(storage, (int)xi, (int)yi, valid, z, order, metric, curveZ, curveMetric);
	storage->flushFile();
}

void Brillouin::continueRepetition() {
	auto* storagePtr = m_acquisition ? m_acquisition->m_storage.get() : nullptr;
	if (!storagePtr) {
		return;
	}
	const auto refuse = [this](const std::string& text) {
		qWarning(logWarning()) << "Brillouin::continueRepetition: not continued -" << text.c_str();
		emit(s_surfaceScanProgress(0.0, QString::fromStdString("Not continued: " + text)));
		m_resuming = false;
		m_acquisition->disableMode(ACQUISITION_MODE::BRILLOUIN);
		setAcquisitionStatus(ACQUISITION_STATUS::ABORTED);
	};
	if (!m_acquisition->enableMode(ACQUISITION_MODE::BRILLOUIN)) {
		return;
	}

	ResumeInfo info;
	std::string problem;
	auto readable = false;
	QMetaObject::invokeMethod(storagePtr, [&]() { readable = readResumeInfo(storagePtr, &info, &problem); }, Qt::BlockingQueuedConnection);
	if (!readable) {
		refuse(problem);
		return;
	}
	if (!info.orderedPrefix) {
		refuse("The points that are in the file are not the first ones of the plan; that cannot be continued safely.");
		return;
	}

	// The settings of the file are those the measurement runs with (the GUI has put them in place as well)
	BRILLOUIN_SETTINGS restored;
	restored = m_settings;
	resume::decodeSettings(info.settings, &restored);
	m_settings = restored;
	if (info.hasScanOrder) {
		SCAN_ORDER order;
		order.automatical = info.scanAutomatic;
		order.x = info.scanX;
		order.y = info.scanY;
		order.z = info.scanZ;
		setScanOrder(order);
	}

	if (!m_scanControl) {
		refuse("No scan control is connected.");
		return;
	}
	// e.g. the software stopped during an overview with another objective: the measurement continues with the objective it was made with
	std::string reason;
	const auto activeSlot = m_scanControl->getActiveObjectiveSlot();
	const auto changerAllows = info.objectiveSlot >= 0 && activeSlot != info.objectiveSlot
		&& m_scanControl->canSwitchObjectiveAutomatically(activeSlot, info.objectiveSlot, &reason);
	switch (resume::decideObjective(activeSlot, info.objectiveSlot, m_resumeSwitchObjective, m_settings.overviewObjectiveRetractUm, changerAllows)) {
		case resume::ObjectiveAction::NONE:
			break;
		case resume::ObjectiveAction::SWITCH: {
			closeDoseProtectionShutter();
			// away from the sample first, as in every automatic switch
			m_scanControl->movePosition(POINT3{ 0.0, 0.0, m_settings.overviewObjectiveRetractUm });
			std::string error;
			if (!m_scanControl->switchObjectiveSlot(info.objectiveSlot, &error)) {
				refuse("The nosepiece could not be turned to slot " + std::to_string(info.objectiveSlot) + ": " + error);
				return;
			}
			break;
		}
		case resume::ObjectiveAction::REFUSE:
			refuse("The objective in slot " + std::to_string(activeSlot) + " is in the beam, the measurement was made with slot " + std::to_string(info.objectiveSlot)
				+ ". Turn the nosepiece back." + (reason.empty() ? std::string{} : " (" + reason + ")"));
			return;
	}
	if (m_settings.gridCoordinatesAbsolute) {
		const auto activeCalibration = m_scanControl->getActiveObjectiveCalibration();
		if (!activeCalibration.hasFovOffset && !m_scanControl->isMissingObjectiveOffsetAccepted()) {
			refuse("The grid is in absolute coordinates and the active objective has no calibrated FOV-center offset.");
			return;
		}
	}
	std::string overviewProblem;
	if (!checkOverviewSetup(&overviewProblem)) {
		refuse(overviewProblem);
		return;
	}

	m_abort = false;
	m_resumeInfo = info;
	m_resuming = true;
	m_resumeStartIndex = info.pointsDone;
	m_resumeCarry = POINT2{ 0.0, 0.0 };

	// the file: same repetitions, nothing created, points that were cut short removed, what exists is not written again
	auto prepared = false;
	QMetaObject::invokeMethod(storagePtr, [&]() {
		prepared = storagePtr->openRepetition(ACQUISITION_MODE::BRILLOUIN, info.repetition);
		if (prepared && info.fluorescenceRepetition >= 0) {
			prepared = storagePtr->openRepetition(ACQUISITION_MODE::FLUORESCENCE, info.fluorescenceRepetition);
		}
		if (prepared) {
			for (const auto point : info.incompletePoints) {
				storagePtr->deletePayloadPoint(point);
			}
			storagePtr->setSkipExistingDatasets(true);
		}
	}, Qt::BlockingQueuedConnection);
	if (!prepared) {
		refuse("The repetitions of the measurement cannot be opened in the file.");
		return;
	}
	m_fileRepetition = info.repetition;
	m_currentRepetition = info.seriesRepetition;
	m_startOfLastRepetition.restart();
	if (m_repetitionTimer == nullptr) {
		m_repetitionTimer = new QTimer();
		QWidget::connect(m_repetitionTimer, &QTimer::timeout, this, &Brillouin::waitForNextRepetition);
	} else {
		m_repetitionTimer->stop();
	}

	// the note in the file: where and when the measurement was stopped, and that it was continued
	{
		const auto now = (double)QDateTime::currentSecsSinceEpoch();
		const auto date = QDateTime::currentDateTime().toOffsetFromUtc(QDateTime::currentDateTime().offsetFromUtc()).toString(Qt::ISODateWithMs).toStdString();
		QMetaObject::invokeMethod(storagePtr, [&]() {
			resume::writeResumeEvent(storagePtr, info.resumeEvents, info.pointsDone, info.pointsTotal, now, info.state == "stopped" || info.state == "surface-scan-stopped",
				(int)info.incompletePoints.size(), info.state, info.lastDataDate, date);
		}, Qt::BlockingQueuedConnection);
	}

	if (info.surfaceScanPhase) {
		// stopped during the surface scan: acquire() takes it up again (its checkpoint is in m_resumeInfo) and goes on as always
		qInfo(logInfo()) << "Continuing the surface scan of repetition" << info.repetition << "with" << (long long)info.surfaceColumns.size() << "columns done.";
		m_resumeSurface = true;
		try {
			acquire(m_acquisition->m_storage);
		} catch (const std::exception& e) {
			qCritical(logCritical()) << "Brillouin::continueRepetition: uncaught exception in acquire():" << e.what();
			m_abort = true;
		} catch (...) {
			qCritical(logCritical()) << "Brillouin::continueRepetition: uncaught unknown exception in acquire().";
			m_abort = true;
		}
		if (m_abort) {
			abortMode(m_acquisition->m_storage);
			return;
		}
		if (getStatus() == ACQUISITION_STATUS::WAITFORSURFACEREVIEW) {
			// continueAfterSurfaceReview() runs the measurement from here, as after any surface scan
			return;
		}
		finishRepetition();
		return;
	}

	// the camera and the optics, as acquire() does
	if (m_andor) {
		m_andor->startAcquisition(m_settings.camera);
		m_settings.camera = m_andor->getSettings();
	} else {
		m_abort = true;
		abortMode(m_acquisition->m_storage);
		return;
	}
	QMetaObject::invokeMethod(
		m_scanControl,
		[scanControl = m_scanControl]() { scanControl->stopAnnouncing(); },
		Qt::AutoConnection
	);
	m_scanControl->setPreset(ScanPreset::SCAN_BRILLOUIN);
	setRLShutterRestingState();
	std::this_thread::sleep_for(std::chrono::milliseconds(500));

	// back to the position the measurement started from (relative mode: the grid is anchored to it)
	m_startPosition = info.startPosition;
	if (!m_settings.gridCoordinatesAbsolute && m_resumeMoveToStart) {
		m_scanControl->setPositionCompensated(m_startPosition);
		std::this_thread::sleep_for(std::chrono::milliseconds(300));
	}
	m_scanControl->enableMeasurementMode(true, m_startPosition);

	// the plan of the file (the surface scan's z values are in it), the surface scan the rescans start from
	updatePositions();
	m_orderedPositions = info.positions;
	m_orderedPositionsRelative = info.positionsRelative;
	m_orderedIndices = info.indices;
	m_calibrationAllowed = info.calibrationAllowed;
	if (m_settings.gridCoordinatesAbsolute) {
		emit(s_orderedPositionsChanged(m_orderedPositions, true));
	} else {
		emit(s_orderedPositionsChanged(m_orderedPositionsRelative, false));
	}
	m_surfacePreScanZUm = info.prescanZ;
	m_surfacePreScanFoundMask = info.prescanFound;
	m_surfaceReferenceThreshold = info.referenceThreshold;

	appendLiveLog("resume," + QDateTime::currentDateTime().toString(Qt::ISODate).toStdString() + "," + std::to_string(info.repetition) + ",point "
		+ std::to_string(info.pointsDone) + " of " + std::to_string(info.pointsTotal) + "," + info.state);
	qInfo(logInfo()) << "Continuing the measurement of repetition" << info.repetition << "at point" << info.pointsDone << "of" << info.pointsTotal
		<< "(" << (info.state == "stopped" ? "it was cancelled" : "it was not stopped properly") << ").";
	setAcquisitionStatus(ACQUISITION_STATUS::STARTED);
	try {
		runMeasurementPhase(m_acquisition->m_storage);
	} catch (const std::exception& e) {
		qCritical(logCritical()) << "Brillouin::continueRepetition: uncaught exception in runMeasurementPhase():" << e.what();
		m_abort = true;
		abortMode(m_acquisition->m_storage);
		return;
	} catch (...) {
		qCritical(logCritical()) << "Brillouin::continueRepetition: uncaught unknown exception in runMeasurementPhase().";
		m_abort = true;
		abortMode(m_acquisition->m_storage);
		return;
	}
	if (m_abort) {
		abortMode(m_acquisition->m_storage);
		return;
	}
	finishRepetition();
}

void Brillouin::endResumeMode(std::unique_ptr<StorageWrapper>& storage) {
	if (m_resuming) {
		auto* storageObject = storage.get();
		if (storageObject) {
			QMetaObject::invokeMethod(storageObject, [storageObject]() { storageObject->setSkipExistingDatasets(false); }, Qt::BlockingQueuedConnection);
		}
	}
	m_resuming = false;
}

/*
 * The surface scan of an earlier repetition of the file. A new repetition (Restart, or the next repetition of a series) can start its surface
 * scan from it (the offset between the two scans found in the first columns moves the old surface of every other column, see
 * m_surfacePriorZUm) or, if the plan of that repetition is in the file, use its surface as it is without scanning again. The operator is asked
 * in a popup with a timer; without an answer the scan starts from the earlier one (if the grid is the same), else from scratch.
 */

namespace {

	constexpr int kSurfacePriorTimeoutS = 6;

}

void Brillouin::setSurfacePriorAnswer(int choice) {
	m_surfacePriorAnswer = choice;
}

Brillouin::SURFACE_STRATEGY Brillouin::chooseSurfaceStrategy(std::unique_ptr<StorageWrapper>& storage, PreviousSurface* previous) {
	if (!m_settings.useSurfaceFollow || !storage) {
		return SURFACE_STRATEGY::NEW_SCAN;
	}
	auto* storageObject = storage.get();
	auto available = false;
	QMetaObject::invokeMethod(storageObject, [&]() {
		available = resume::readPreviousSurface(storageObject, ACQUISITION_MODE::BRILLOUIN, m_fileRepetition, previous);
	}, Qt::BlockingQueuedConnection);
	if (!available) {
		return SURFACE_STRATEGY::NEW_SCAN;
	}
	const auto [xSamples, ySamples] = coarseXYSamples(m_settings.preScanXYBin);
	resume::GridSignature now;
	now.xMin = m_settings.xMin;
	now.xMax = m_settings.xMax;
	now.xSteps = m_settings.xSteps;
	now.yMin = m_settings.yMin;
	now.yMax = m_settings.yMax;
	now.ySteps = m_settings.ySteps;
	now.zMin = m_settings.zMin;
	now.zMax = m_settings.zMax;
	now.zSteps = m_settings.zSteps;
	now.absolute = m_settings.gridCoordinatesAbsolute;
	now.preScanXYBin = m_settings.preScanXYBin;
	now.originX = m_settings.absoluteGridOriginUm.x;
	now.originY = m_settings.absoluteGridOriginUm.y;
	const auto options = resume::surfaceOptions(*previous, now, xSamples.size() * ySamples.size());
	if (!options.priorCompatible && !options.canReuse) {
		qInfo(logInfo()) << "The surface scan of repetition" << previous->repetition << "is not offered: the grid or the surface scan settings differ.";
		return SURFACE_STRATEGY::NEW_SCAN;
	}

	auto text = QString("This file holds the surface scan of repetition %1.\n\n").arg(previous->repetition);
	if (options.priorCompatible) {
		text += "Yes: scan again, starting from it. The first columns find the offset between the two surfaces and all other columns start next to "
			"where the surface is expected, which makes the scan faster.\n";
	}
	text += "No: scan from scratch, as always.\n";
	if (!options.xyGridSame) {
		text += "(The grid or the surface scan bin differs from that repetition, so only a new scan is possible.)\n";
	} else if (!options.zGridSame) {
		text += "(The z planes differ from that repetition: its surface cannot be reused without a new scan.)\n";
	}
	if (options.canReuse) {
		text += "Reuse: no new scan, the surface of that repetition is used as it is.\n";
	}
	m_surfacePriorAnswer = -1;
	QEventLoop loop;
	QTimer poll;
	QElapsedTimer waited;
	waited.start();
	QWidget::connect(&poll, &QTimer::timeout, &loop, [&]() {
		// answered, cancelled, or (no GUI to answer) well past the time the popup counts down
		if (m_surfacePriorAnswer >= 0 || m_abort || waited.elapsed() > 1000LL * (kSurfacePriorTimeoutS + 15)) {
			loop.quit();
		}
	});
	poll.start(100);
	emit(s_askSurfacePrior(text, options.priorCompatible, options.canReuse, kSurfacePriorTimeoutS));
	loop.exec();
	poll.stop();

	const auto answer = resume::resolveSurfaceAnswer(m_surfacePriorAnswer, options);
	m_surfacePriorAnswer = -1;
	return answer == 1 ? SURFACE_STRATEGY::PRIOR : (answer == 2 ? SURFACE_STRATEGY::REUSE : SURFACE_STRATEGY::NEW_SCAN);
}

void Brillouin::applyPreviousSurface(const PreviousSurface& previous) {
	const auto zOrigin = m_settings.gridCoordinatesAbsolute ? resolvedGridOriginUm().z : m_startPosition.z;
	// relative mode: the surface is the same distance from the start z as it was then
	const auto zShift = m_settings.gridCoordinatesAbsolute ? 0.0 : m_startPosition.z - previous.startPosition.z;
	std::map<std::tuple<int, int, int>, double> storedZ;
	for (size_t i = 0; i < previous.planIndices.size() && i < previous.planZ.size(); i++) {
		storedZ[std::make_tuple(previous.planIndices[i].x, previous.planIndices[i].y, previous.planIndices[i].z)] = previous.planZ[i];
	}
	m_surfaceFoundXYIndices.clear();
	m_surfaceInterpolatedXYIndices.clear();
	std::map<std::pair<int, int>, double> centreByXY;
	for (size_t ll = 0; ll < m_orderedPositions.size(); ll++) {
		const auto& index = m_orderedIndices[ll];
		const auto stored = storedZ.find(std::make_tuple(index.x, index.y, index.z));
		if (stored == storedZ.end()) {
			continue;
		}
		const auto zAbs = stored->second + zShift;
		// the surface under this column: the stored z minus the offset of this z plane (same as applySurfaceFollowPlan() builds it)
		centreByXY[std::make_pair(index.x, index.y)] = zAbs - (m_orderedPositions[ll].z - zOrigin);
		m_orderedPositions[ll].z = zAbs;
		m_orderedPositionsRelative[ll].z = zAbs - m_startPosition.z;
	}
	for (const auto& entry : centreByXY) {
		m_surfaceFoundXYIndices.insert(entry.first);
	}
	m_surfaceZRangeValid = !centreByXY.empty();
	if (m_surfaceZRangeValid) {
		const auto minMax = std::minmax_element(centreByXY.begin(), centreByXY.end(), [](const auto& a, const auto& b) { return a.second < b.second; });
		m_surfaceZMinAbs = minMax.first->second;
		m_surfaceZMaxAbs = minMax.second->second;
		std::vector<double> found;
		for (const auto& entry : centreByXY) {
			found.push_back(entry.second);
		}
		std::sort(found.begin(), found.end());
		m_surfaceZMedianAbs = found.size() % 2 ? found[found.size() / 2] : 0.5 * (found[found.size() / 2 - 1] + found[found.size() / 2]);
	}
	// the coarse scan the file keeps for this repetition is the one that was reused (so a later repetition can start from it as well)
	const auto [xSamples, ySamples] = coarseXYSamples(m_settings.preScanXYBin);
	if (previous.z.size() == xSamples.size() * ySamples.size()) {
		m_surfacePreScanXUm = xSamples;
		m_surfacePreScanYUm = ySamples;
		m_surfacePreScanZUm = previous.z;
		m_surfacePreScanFoundMask = previous.found;
		m_surfacePreScanMetric.assign(previous.z.size(), std::numeric_limits<double>::quiet_NaN());
	}
	if (m_settings.gridCoordinatesAbsolute) {
		emit(s_orderedPositionsChanged(m_orderedPositions, true));
	} else {
		emit(s_orderedPositionsChanged(m_orderedPositionsRelative, false));
	}
	emit(s_surfaceScanProgress(100.0, QString("No new surface scan: the surface of repetition %1 is used (%2 columns).")
		.arg(previous.repetition).arg((int)centreByXY.size())));
	qInfo(logInfo()) << "Surface of repetition" << previous.repetition << "used without a new scan:" << (long long)centreByXY.size() << "columns.";
}

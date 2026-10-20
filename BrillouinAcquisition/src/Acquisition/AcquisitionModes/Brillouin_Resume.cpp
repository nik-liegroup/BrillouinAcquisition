#include "stdafx.h"
#include "Brillouin.h"
#include "BrillouinResume.h"
#include "src/helper/logger.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <set>
#include <tuple>
#include <thread>

/*
 * Continue of an interrupted measurement: crash, cancel, pause. Everything needed is in the file of the measurement:
 *  - the state of the repetition ("preparing", "measuring", "stopped", "finished" - attribute acquisition_state of the repetition group),
 *  - the settings in full, the start position, the objective, the plan (positions in measuring order, z as the surface scan left them),
 *  - which points are in the file (the points themselves), and the overview events and surface rescans of the engine.
 * A continue appends to the same repetition, starts at the first point that is not complete in the file and notes in the file where and when
 * the measurement was stopped (resume-event-N-*). It is never started by itself: the operator loads the file and presses Continue.
 */

namespace {

	std::string padded(int number, int digits) {
		auto text = std::to_string(number);
		while ((int)text.size() < digits) {
			text = "0" + text;
		}
		return text;
	}

}

bool Brillouin::readResumeInfo(StorageWrapper* storage, ResumeInfo* info, std::string* problem) {
	auto fail = [&](const std::string& text) {
		if (problem) {
			*problem = text;
		}
		return false;
	};
	if (!storage || !storage->isWritable()) {
		return fail("No acquisition file is open for writing.");
	}
	const auto repetition = storage->lastRepetition(ACQUISITION_MODE::BRILLOUIN);
	if (repetition < 0) {
		return fail("The open file holds no Brillouin measurement.");
	}
	const auto state = storage->getRepetitionState(ACQUISITION_MODE::BRILLOUIN, repetition);
	if (state.empty()) {
		return fail("The last Brillouin measurement of this file was written by a version that cannot be continued.");
	}
	if (state == "finished") {
		return fail("The last Brillouin measurement of this file is finished.");
	}
	if (state == "preparing") {
		return fail("The last Brillouin measurement was stopped before it started measuring (during the surface scan or before it), so there is nothing to continue; start it again.");
	}
	const auto surfacePhase = state == "surface-scan" || state == "surface-scan-stopped";
	if (state != "measuring" && state != "stopped" && !surfacePhase) {
		return fail("The last Brillouin measurement of this file is in the state \"" + state + "\", which cannot be continued.");
	}
	if (!storage->openRepetition(ACQUISITION_MODE::BRILLOUIN, repetition)) {
		return fail("The last Brillouin measurement of this file cannot be opened.");
	}

	ResumeInfo result;
	result.repetition = repetition;
	result.state = state;

	std::vector<double> values;
	const auto number = [&](const std::string& name, size_t count) {
		return storage->getResumeNumbers(name, &values) && values.size() >= count;
	};
	if (!number("version", 1)) {
		return fail("The file has no continue information for its last Brillouin measurement.");
	}
	if (!number("start-position", 3)) {
		return fail("The start position of the measurement is not in the file.");
	}
	result.startPosition = POINT3{ values[0], values[1], values[2] };
	if (number("objective-slot", 1)) {
		result.objectiveSlot = (int)std::llround(values[0]);
	}
	if (number("series", 1)) {
		result.seriesRepetition = (int)std::llround(values[0]);
	}
	if (number("fluorescence-repetition", 1)) {
		result.fluorescenceRepetition = (int)std::llround(values[0]);
	}

	// the settings
	for (const auto& name : resume::settingNames(true)) {
		if (storage->getResumeNumbers("s." + name, &values)) {
			result.settings.numbers[name] = values;
		}
	}
	for (const auto& name : resume::settingNames(false)) {
		std::string text;
		if (storage->getResumeText("s." + name, &text)) {
			result.settings.strings[name] = text;
		}
	}
	if (result.settings.numbers.empty()) {
		return fail("The settings of the measurement are not in the file.");
	}
	const auto absolute = result.settings.numbers.find("gridCoordinatesAbsolute");
	result.gridAbsolute = absolute != result.settings.numbers.end() && !absolute->second.empty() && absolute->second.front() != 0.0;

	if (surfacePhase) {
		// stopped during the surface scan: the checkpoint (medium reference, every column that was finished), no plan yet
		result.surfaceScanPhase = true;
		std::vector<double> reference;
		if (!storage->getResumeNumbers("surface-reference", &reference) || reference.size() < 4) {
			return fail("The measurement stopped during its surface scan before the medium reference was measured; start it again.");
		}
		result.surfaceReference = reference[0];
		const auto columnsX = (int)std::llround(reference[2]);
		const auto columnsY = (int)std::llround(reference[3]);
		for (auto yi = 0; yi < columnsY; yi++) {
			for (auto xi = 0; xi < columnsX; xi++) {
				std::vector<double> column;
				if (!storage->getResumeNumbers("surface-col-" + std::to_string(yi) + "-" + std::to_string(xi), &column) || column.size() < 5) {
					continue;
				}
				ResumeInfo::SurfaceColumn entry;
				entry.xi = xi;
				entry.yi = yi;
				entry.valid = column[0] != 0.0;
				entry.z = column[1];
				entry.order = (int)std::llround(column[2]);
				entry.metric = column[3];
				const auto samples = (size_t)std::llround(column[4]);
				if (column.size() >= 5 + 2 * samples) {
					entry.curveZ.assign(column.begin() + 5, column.begin() + 5 + samples);
					entry.curveMetric.assign(column.begin() + 5 + samples, column.begin() + 5 + 2 * samples);
				}
				result.surfaceColumns.push_back(std::move(entry));
			}
		}
		for (auto k = 0; storage->getResumeNumbers("event-" + padded(k, 4) + "-point-index", &values); k++) {
			result.resumeEvents++;
		}
		*info = std::move(result);
		return true;
	}

	// the plan
	std::vector<double> ix, iy, iz, px, py, pz, rx, ry, rz, allowed;
	if (!storage->getResumeNumbers("plan-index-x", &ix) || !storage->getResumeNumbers("plan-index-y", &iy) || !storage->getResumeNumbers("plan-index-z", &iz)
		|| !storage->getResumeNumbers("plan-pos-x", &px) || !storage->getResumeNumbers("plan-pos-y", &py) || !storage->getResumeNumbers("plan-pos-z", &pz)
		|| !storage->getResumeNumbers("plan-posrel-x", &rx) || !storage->getResumeNumbers("plan-posrel-y", &ry) || !storage->getResumeNumbers("plan-posrel-z", &rz)
		|| !storage->getResumeNumbers("plan-calibration-allowed", &allowed)) {
		return fail("The plan of the measurement (its points) is not in the file.");
	}
	const auto total = ix.size();
	if (total == 0 || iy.size() != total || iz.size() != total || px.size() != total || py.size() != total || pz.size() != total || rx.size() != total
		|| ry.size() != total || rz.size() != total || allowed.size() != total) {
		return fail("The plan of the measurement in the file is inconsistent.");
	}
	for (size_t i = 0; i < total; i++) {
		result.indices.push_back(INDEX3{ (int)std::llround(ix[i]), (int)std::llround(iy[i]), (int)std::llround(iz[i]) });
		result.positions.push_back(POINT3{ px[i], py[i], pz[i] });
		result.positionsRelative.push_back(POINT3{ rx[i], ry[i], rz[i] });
		result.calibrationAllowed.push_back(allowed[i] != 0.0);
	}
	result.pointsTotal = (int)total;

	// which points are in the file
	std::vector<int> complete;
	storage->listPayloadPoints(&complete, &result.incompletePoints);
	const std::set<int> completeSet(complete.begin(), complete.end());
	std::vector<bool> done(total, false);
	for (size_t i = 0; i < total; i++) {
		done[i] = completeSet.count(storage->payloadIndex(result.indices[i].x, result.indices[i].y, result.indices[i].z)) > 0;
	}
	result.pointsDone = 0;
	while ((size_t)result.pointsDone < total && done[(size_t)result.pointsDone]) {
		result.pointsDone++;
	}
	for (size_t i = (size_t)result.pointsDone; i < total; i++) {
		if (done[i]) {
			result.orderedPrefix = false;
		}
	}
	if (result.pointsDone > 0) {
		const auto& last = result.indices[(size_t)result.pointsDone - 1];
		result.lastDataDate = storage->getPayloadDateByIndex(storage->payloadIndex(last.x, last.y, last.z));
	}
	result.calibrations = storage->countCalibrationData();
	result.backgrounds = storage->countBackgroundData();

	// the engine: overview events (with the grid offset from them on), surface rescans, earlier continues
	for (auto k = 0;; k++) {
		std::vector<double> point, offset;
		const auto prefix = "overview-event-" + padded(k, 4) + "-";
		if (!storage->readPositions(prefix + "point-index", &point) || point.empty() || !storage->readPositions(prefix + "grid-offset-after-um", &offset)
			|| offset.size() < 2) {
			break;
		}
		result.eventPointIndex.push_back((int)std::llround(point[0]));
		result.eventOffsetAfter.push_back(POINT2{ offset[0], offset[1] });
	}
	result.overviewEvents = (int)result.eventPointIndex.size();
	for (auto k = 0; storage->readPositions("surface-rescan-" + padded(k, 4) + "-success", &values); k++) {
		result.surfaceRescans++;
	}
	for (auto k = 0; storage->getResumeNumbers("event-" + padded(k, 4) + "-point-index", &values); k++) {
		result.resumeEvents++;
	}
	std::vector<double> carriedX, carriedY;
	if (storage->readPositions("drift-carried-over-x-um", &carriedX) && storage->readPositions("drift-carried-over-y-um", &carriedY) && !carriedX.empty() && !carriedY.empty()) {
		result.carriedOffset = POINT2{ carriedX[0], carriedY[0] };
	}

	// the last surface scan (the prior of a later rescan)
	storage->readPositions("surface-prescan-z-um", &result.prescanZ);
	storage->readPositions("surface-prescan-found-mask", &result.prescanFound);
	if (storage->readPositions("surface-reference-threshold-computed", &values) && !values.empty()) {
		result.referenceThreshold = values[0];
	}

	*info = std::move(result);
	return true;
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
	const auto put = [&](const std::string& name, const std::vector<double>& values) {
		if (direct) {
			target->setResumeNumbers(name, values);
		} else {
			QMetaObject::invokeMethod(target, [target, name, values]() { target->setResumeNumbers(name, values); }, Qt::AutoConnection);
		}
	};
	const auto count = m_orderedPositions.size();
	std::vector<double> ix(count), iy(count), iz(count), px(count), py(count), pz(count), rx(count), ry(count), rz(count), allowed(count);
	for (size_t i = 0; i < count; i++) {
		ix[i] = m_orderedIndices[i].x;
		iy[i] = m_orderedIndices[i].y;
		iz[i] = m_orderedIndices[i].z;
		px[i] = m_orderedPositions[i].x;
		py[i] = m_orderedPositions[i].y;
		pz[i] = m_orderedPositions[i].z;
		rx[i] = m_orderedPositionsRelative[i].x;
		ry[i] = m_orderedPositionsRelative[i].y;
		rz[i] = m_orderedPositionsRelative[i].z;
		allowed[i] = m_calibrationAllowed[i] ? 1.0 : 0.0;
	}
	put("plan-index-x", ix);
	put("plan-index-y", iy);
	put("plan-index-z", iz);
	put("plan-pos-x", px);
	put("plan-pos-y", py);
	put("plan-pos-z", pz);
	put("plan-posrel-x", rx);
	put("plan-posrel-y", ry);
	put("plan-posrel-z", rz);
	put("plan-calibration-allowed", allowed);
}

void Brillouin::writeResumeBasics(std::unique_ptr<StorageWrapper>& storage) {
	auto* target = storage.get();
	const auto record = resume::encodeSettings(m_settings);
	for (const auto& entry : record.numbers) {
		target->setResumeNumbers("s." + entry.first, entry.second);
	}
	for (const auto& entry : record.strings) {
		target->setResumeText("s." + entry.first, entry.second);
	}
	target->setResumeNumbers("version", std::vector<double>{ 1.0 });
	target->setResumeNumbers("start-position", std::vector<double>{ m_startPosition.x, m_startPosition.y, m_startPosition.z });
	target->setResumeNumbers("objective-slot", std::vector<double>{ (double)(m_scanControl ? m_scanControl->getActiveObjectiveSlot() : -1) });
	target->setResumeNumbers("series", std::vector<double>{ (double)m_currentRepetition });
	const auto fluorescence = overviewImagesExpected() ? target->getModeHandle(ACQUISITION_MODE::FLUORESCENCE)->repetitionCount : -1;
	target->setResumeNumbers("fluorescence-repetition", std::vector<double>{ (double)fluorescence });
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
	// [medium reference, drop fraction, columns in x, columns in y]
	storage->setResumeNumbers("surface-reference", std::vector<double>{ mediumReference, m_settings.surfaceDropFraction, (double)columnsX, (double)columnsY });
	storage->flushFile();
}

void Brillouin::writeSurfaceCheckpointColumn(gsl::index xi, gsl::index yi, bool valid, double z, int order, double metric,
	const std::vector<double>& curveZ, const std::vector<double>& curveMetric) {
	auto* storage = m_acquisition ? m_acquisition->m_storage.get() : nullptr;
	if (!storage) {
		return;
	}
	// [found, z, order, metric, number of samples, curve z..., curve metric...]
	std::vector<double> values{ valid ? 1.0 : 0.0, z, (double)order, metric, (double)curveZ.size() };
	values.insert(values.end(), curveZ.begin(), curveZ.end());
	values.insert(values.end(), curveMetric.begin(), curveMetric.end());
	storage->setResumeNumbers("surface-col-" + std::to_string(yi) + "-" + std::to_string(xi), values);
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

	if (!m_scanControl) {
		refuse("No scan control is connected.");
		return;
	}
	if (info.objectiveSlot >= 0 && m_scanControl->getActiveObjectiveSlot() != info.objectiveSlot) {
		// e.g. the software stopped during an overview with another objective: the measurement continues with the objective it was made with
		std::string reason;
		if (m_resumeSwitchObjective && m_settings.overviewObjectiveRetractUm != 0.0
			&& m_scanControl->canSwitchObjectiveAutomatically(m_scanControl->getActiveObjectiveSlot(), info.objectiveSlot, &reason)) {
			closeDoseProtectionShutter();
			// away from the sample first, as in every automatic switch
			m_scanControl->movePosition(POINT3{ 0.0, 0.0, m_settings.overviewObjectiveRetractUm });
			std::string error;
			if (!m_scanControl->switchObjectiveSlot(info.objectiveSlot, &error)) {
				refuse("The nosepiece could not be turned to slot " + std::to_string(info.objectiveSlot) + ": " + error);
				return;
			}
		} else {
			refuse("The objective in slot " + std::to_string(m_scanControl->getActiveObjectiveSlot()) + " is in the beam, the measurement was made with slot "
				+ std::to_string(info.objectiveSlot) + ". Turn the nosepiece back." + (reason.empty() ? std::string{} : " (" + reason + ")"));
			return;
		}
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
		const auto k = padded(info.resumeEvents, 4);
		const auto now = (double)QDateTime::currentSecsSinceEpoch();
		QMetaObject::invokeMethod(storagePtr, [&]() {
			storagePtr->setResumeNumbers("event-" + k + "-point-index", std::vector<double>{ (double)info.pointsDone });
			storagePtr->setResumeNumbers("event-" + k + "-points-total", std::vector<double>{ (double)info.pointsTotal });
			storagePtr->setResumeNumbers("event-" + k + "-resumed-epoch-seconds", std::vector<double>{ now });
			// 0: nothing was written when the software stopped (crash, power loss), 1: the measurement was cancelled
			storagePtr->setResumeNumbers("event-" + k + "-was-cancelled", std::vector<double>{ info.state == "stopped" ? 1.0 : 0.0 });
			storagePtr->setResumeNumbers("event-" + k + "-incomplete-points-removed", std::vector<double>{ (double)info.incompletePoints.size() });
			storagePtr->setResumeText("event-" + k + "-previous-state", info.state);
			storagePtr->setResumeText("event-" + k + "-last-point-date", info.lastDataDate);
			storagePtr->setResumeText("event-" + k + "-resumed-date", QDateTime::currentDateTime().toOffsetFromUtc(QDateTime::currentDateTime().offsetFromUtc())
				.toString(Qt::ISODateWithMs).toStdString());
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
 * in a popup with a timer; without an answer the scan is made from scratch, as always.
 */

namespace {

	constexpr int kSurfacePriorTimeoutS = 20;

}

bool Brillouin::readPreviousSurface(StorageWrapper* storage, int beforeRepetition, PreviousSurface* previous) {
	for (auto repetition = beforeRepetition - 1; repetition >= 0; repetition--) {
		std::vector<double> z, found;
		if (!storage->readPositionsOfRepetition(ACQUISITION_MODE::BRILLOUIN, repetition, "surface-prescan-z-um", &z)
			|| !storage->readPositionsOfRepetition(ACQUISITION_MODE::BRILLOUIN, repetition, "surface-prescan-found-mask", &found)
			|| z.empty() || z.size() != found.size()) {
			continue;
		}
		PreviousSurface result;
		result.repetition = repetition;
		result.z = std::move(z);
		result.found = std::move(found);
		std::vector<double> ix, iy, iz, pz, start;
		// the resume vectors carry their length in front (see H5BM::setResumeNumbers())
		const auto resumeVector = [&](const std::string& name, std::vector<double>* out) {
			if (!storage->readPositionsOfRepetition(ACQUISITION_MODE::BRILLOUIN, repetition, "resume-" + name, out) || out->empty()) {
				return false;
			}
			out->erase(out->begin());
			return true;
		};
		if (resumeVector("plan-index-x", &ix) && resumeVector("plan-index-y", &iy) && resumeVector("plan-index-z", &iz) && resumeVector("plan-pos-z", &pz)
			&& resumeVector("start-position", &start)
			&& ix.size() == iy.size() && ix.size() == iz.size() && ix.size() == pz.size() && !ix.empty() && start.size() >= 3) {
			result.hasPlan = true;
			for (size_t i = 0; i < ix.size(); i++) {
				result.planIndices.push_back(INDEX3{ (int)std::llround(ix[i]), (int)std::llround(iy[i]), (int)std::llround(iz[i]) });
			}
			result.planZ = std::move(pz);
			result.startPosition = POINT3{ start[0], start[1], start[2] };
		}
		// the grid and the surface scan settings of that repetition (a surface is only reused for the same grid)
		for (const char* name : { "xMin", "xMax", "xSteps", "yMin", "yMax", "ySteps", "zMin", "zMax", "zSteps", "gridCoordinatesAbsolute", "preScanXYBin",
			"absoluteGridOriginUm.x", "absoluteGridOriginUm.y", "absoluteGridOriginUm.z" }) {
			std::vector<double> value;
			if (storage->readPositionsOfRepetition(ACQUISITION_MODE::BRILLOUIN, repetition, std::string("resume-s.") + name, &value) && value.size() >= 2) {
				result.settings[name] = value[1];		// [length, value]
			}
		}
		*previous = std::move(result);
		return true;
	}
	return false;
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
	QMetaObject::invokeMethod(storageObject, [&]() { available = readPreviousSurface(storageObject, m_fileRepetition, previous); }, Qt::BlockingQueuedConnection);
	if (!available) {
		return SURFACE_STRATEGY::NEW_SCAN;
	}
	const auto [xSamples, ySamples] = coarseXYSamples(m_settings.preScanXYBin);
	// The surface of an earlier repetition only fits the same grid: its columns are where this repetition's columns are only if the extent,
	// steps and surface scan bin are the same (and in absolute coordinates the same origin); its plan only if the z planes are the same too.
	const auto same = [&](const char* name, double current) {
		const auto found = previous->settings.find(name);
		return found != previous->settings.end() && std::abs(found->second - current) < 1e-9;
	};
	const auto xyGridSame = same("xMin", m_settings.xMin) && same("xMax", m_settings.xMax) && same("xSteps", m_settings.xSteps)
		&& same("yMin", m_settings.yMin) && same("yMax", m_settings.yMax) && same("ySteps", m_settings.ySteps)
		&& same("gridCoordinatesAbsolute", m_settings.gridCoordinatesAbsolute ? 1.0 : 0.0) && same("preScanXYBin", m_settings.preScanXYBin)
		&& (!m_settings.gridCoordinatesAbsolute || (same("absoluteGridOriginUm.x", m_settings.absoluteGridOriginUm.x) && same("absoluteGridOriginUm.y", m_settings.absoluteGridOriginUm.y)));
	const auto zGridSame = same("zMin", m_settings.zMin) && same("zMax", m_settings.zMax) && same("zSteps", m_settings.zSteps);
	const auto priorCompatible = xyGridSame && previous->z.size() == xSamples.size() * ySamples.size();
	const auto canReuse = previous->hasPlan && xyGridSame && zGridSame;
	if (!priorCompatible && !canReuse) {
		qInfo(logInfo()) << "The surface scan of repetition" << previous->repetition << "is not offered: the grid or the surface scan settings differ.";
		return SURFACE_STRATEGY::NEW_SCAN;
	}

	auto text = QString("This file holds the surface scan of repetition %1.\n\n").arg(previous->repetition);
	if (priorCompatible) {
		text += "Yes: scan again, starting from it. The first columns find the offset between the two surfaces and all other columns start next to "
			"where the surface is expected, which makes the scan faster.\n";
	}
	text += "No: scan from scratch, as always.\n";
	if (!xyGridSame) {
		text += "(The grid or the surface scan bin differs from that repetition, so only a new scan is possible.)\n";
	} else if (!zGridSame) {
		text += "(The z planes differ from that repetition: its surface cannot be reused without a new scan.)\n";
	}
	if (canReuse) {
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
	emit(s_askSurfacePrior(text, canReuse, kSurfacePriorTimeoutS));
	loop.exec();
	poll.stop();

	const auto answer = m_surfacePriorAnswer;
	m_surfacePriorAnswer = -1;
	if (answer == 1 && priorCompatible) {
		return SURFACE_STRATEGY::PRIOR;
	}
	if (answer == 2 && canReuse) {
		return SURFACE_STRATEGY::REUSE;
	}
	return SURFACE_STRATEGY::NEW_SCAN;
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

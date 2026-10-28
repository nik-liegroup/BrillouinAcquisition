#include "stdafx.h"
#include "Brillouin.h"
#include "src/lib/math/simplemath.h"
#include "src/helper/logger.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <atomic>
#include <condition_variable>
#include <cstring>
#include <deque>
#include <memory>
#include <mutex>
#include <filesystem>
#include <fstream>
#include <QImage>
#include <functional>
#include <limits>
#include <map>
#include <set>
#include <thread>

/*
 * Overviews with several objectives, overview schedule, drift correction and repeated surface scans of the Brillouin measurement.
 *
 * What runs when (see runMeasurementPhase()):
 *  - useOverviewEngine() false (no setting of this feature touched): none of it; the measurement loop is the one that has always run.
 *  - otherwise at every moment of OverviewSchedule - before the first point, after a z plane, after the last point - one overview
 *    event: every objective that takes overviews captures its overview (the measuring objective in place, any other one with a
 *    complete cycle: retract z, turn the nosepiece, move to that objective's field of view and focus, capture, turn back, restore),
 *    the overview of the drift objective is registered against the previous accepted one and the grid follows the measured drift, and
 *    the surface scan may be repeated. Every step is written to the file.
 *
 * Drift correction moves nothing but the targets: the grid, the overviews and the surface scan columns carry the offset (gridOffset()),
 * every measured point stores where it was planned (target position) and where the stage really was (stage position), and each event
 * stores the offset applied from it on, so the unshifted positions can always be reconstructed.
 */

namespace {

	// Image numbers of overview-event images (the per-z overview of the measuring objective keeps its own numbering, 0 .. zSteps * images
	// per z - 1, and per-point brightfield images are negative): base + event * spacing + objective * spacing + tile * 100 + stack index.
	constexpr int kEventImageBase = 1000000;
	constexpr int kEventImageSpacing = 1000000;
	constexpr int kObjectiveImageSpacing = 100000;
	constexpr int kMaxTilesPerOverview = 1000;
	constexpr int kMaxStackPerTile = 100;
	// larger focus differences between two objectives than this are not believed (a wrong calibration would drive into the sample)
	constexpr double kMaxFocusDeltaUm = 3000.0;
	// the drift of the previous repetition is only carried over if the new one starts at (nearly) the same place
	constexpr double kSeriesCarryOverDistanceUm = 5.0;

	std::string momentKindName(OVERVIEW_MOMENT_KIND kind) {
		switch (kind) {
			case OVERVIEW_MOMENT_KIND::BEFORE:
				return "before measurement";
			case OVERVIEW_MOMENT_KIND::PLANE:
				return "after z plane";
			case OVERVIEW_MOMENT_KIND::REPETITION:
				return "after repetition";
			case OVERVIEW_MOMENT_KIND::END:
				return "after measurement";
		}
		return "unknown";
	}

	std::string padded(int number, int digits) {
		auto text = std::to_string(number);
		while ((int)text.size() < digits) {
			text = "0" + text;
		}
		return text;
	}

	std::string slotName(int slot) {
		return slot < 0 ? std::string{ "x" } : std::to_string(slot);
	}

	// A raw brightfield frame as floats; false for a data type that is not handled.
	bool frameToFloat(const std::vector<std::byte>& image, const CAMERA_SETTINGS& settings, std::vector<float>* out, int* width, int* height) {
		*width = (int)settings.roi.width_binned;
		*height = (int)settings.roi.height_binned;
		const auto count = (size_t)(*width) * (size_t)(*height);
		if (count == 0) {
			return false;
		}
		out->assign(count, 0.0f);
		if (settings.readout.dataType == "unsigned short") {
			if (image.size() < count * sizeof(unsigned short)) {
				return false;
			}
			std::vector<unsigned short> raw(count);
			std::memcpy(raw.data(), image.data(), count * sizeof(unsigned short));
			for (size_t i = 0; i < count; i++) {
				(*out)[i] = (float)raw[i];
			}
			return true;
		}
		if (settings.readout.dataType == "unsigned char") {
			if (image.size() < count) {
				return false;
			}
			for (size_t i = 0; i < count; i++) {
				(*out)[i] = (float)std::to_integer<unsigned char>(image[i]);
			}
			return true;
		}
		return false;
	}

	template <typename T>
	void enqueueOverviewEventImage(
		std::unique_ptr<StorageWrapper>& storage, int imageNumber, const CAMERA_SETTINGS& cameraSettings, const std::vector<std::byte>& image,
		const POINT3& targetPosition, const POINT3& stagePosition, int brillouinRepetitionIndex, const std::string& channel,
		const std::vector<std::pair<std::string, double>>& extraNumbers, const std::vector<std::pair<std::string, std::string>>& extraStrings
	) {
		auto date = QDateTime::currentDateTime().toOffsetFromUtc(QDateTime::currentDateTime().offsetFromUtc())
			.toString(Qt::ISODateWithMs).toStdString();
		const int rankData{ 3 };
		auto dimsData = new hsize_t[3]{ 1, (hsize_t)cameraSettings.roi.height_binned, (hsize_t)cameraSettings.roi.width_binned };
		const auto pixelCount = (size_t)cameraSettings.roi.height_binned * (size_t)cameraSettings.roi.width_binned;
		auto typedImage = std::vector<T>(pixelCount);
		const auto bytesToCopy = std::min(image.size(), typedImage.size() * sizeof(T));
		if (bytesToCopy > 0) {
			std::memcpy(typedImage.data(), image.data(), bytesToCopy);
		}
		auto img = new FLUOIMAGE<T>(
			imageNumber, rankData, dimsData, date, channel, typedImage, cameraSettings.exposureTime, cameraSettings.gain, cameraSettings.roi,
			targetPosition, true, stagePosition, false, true, brillouinRepetitionIndex, extraNumbers, extraStrings
		);
		QMetaObject::invokeMethod(
			storage.get(),
			[&storage = storage, img]() { storage.get()->s_enqueuePayload(img); },
			Qt::AutoConnection
		);
	}

	// A one-dimensional dataset of the current repetition, written through the storage thread (the file is being written to by it).
	void putDataset(std::unique_ptr<StorageWrapper>& storage, const std::string& name, const std::vector<double>& values) {
		auto* target = storage.get();
		QMetaObject::invokeMethod(
			target,
			[target, name, values]() {
				const hsize_t dims[1] = { (hsize_t)values.size() };
				target->setPositions(name, values, 1, dims);
			},
			Qt::AutoConnection
		);
	}

	// The same, written directly: only while the storage thread is not writing yet (before startWritingQueues()).
	void putDatasetDirect(std::unique_ptr<StorageWrapper>& storage, const std::string& name, const std::vector<double>& values) {
		const hsize_t dims[1] = { (hsize_t)values.size() };
		storage->setPositions(name, values, 1, dims);
	}

}

double Brillouin::steadySeconds() {
	return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

bool Brillouin::useOverviewEngine() const {
	if (m_settings.overviewTrigger != OVERVIEW_TRIGGER::PER_PLANE) {
		return true;
	}
	if (m_settings.overviewBeforeMeasurement || m_settings.overviewAfterMeasurement) {
		return true;
	}
	if (m_settings.driftCorrectionEnabled || m_settings.surfaceRescan != SURFACE_RESCAN::NEVER) {
		return true;
	}
	for (const auto& entry : m_settings.overviewObjectives) {
		if (entry.active) {
			return true;
		}
	}
	return false;
}

std::vector<Brillouin::OverviewObjectivePlan> Brillouin::planOverviewObjectives() const {
	std::vector<OverviewObjectivePlan> plans;
	const auto activeSlot = m_scanControl ? m_scanControl->getActiveObjectiveSlot() : -1;
	if (m_settings.saveOverviewBrightfieldPerZ) {
		OverviewObjectivePlan plan;
		plan.slot = activeSlot;
		plan.isMeasurementObjective = true;
		plan.settings.slot = activeSlot;
		plan.settings.active = true;
		plan.settings.exposureMs = m_settings.overviewBrightfieldExposureMs;
		plan.settings.gain = m_settings.overviewBrightfieldGain;
		plan.settings.fullGrid = m_settings.overviewBrightfieldFullGrid;
		plan.settings.fullStack = m_settings.overviewBrightfieldFullGrid
			? m_settings.overviewBrightfieldFullStackMosaic : m_settings.overviewBrightfieldFullStackSingle;
		plans.push_back(plan);
	}
	for (const auto& entry : m_settings.overviewObjectives) {
		if (!entry.active || entry.slot == activeSlot) {
			continue;
		}
		OverviewObjectivePlan plan;
		plan.slot = entry.slot;
		plan.isMeasurementObjective = false;
		plan.settings = entry;
		plans.push_back(plan);
	}
	return plans;
}

bool Brillouin::checkOverviewSetup(std::string* problem) const {
	auto fail = [&](const std::string& text) {
		if (problem) {
			*problem = text;
		}
		return false;
	};
	if (!useOverviewEngine()) {
		return true;
	}
	if (m_settings.surfaceRescan != SURFACE_RESCAN::NEVER && !m_settings.useSurfaceFollow) {
		return fail("Repeating the surface scan needs surface scanning to be on.");
	}
	const auto plans = planOverviewObjectives();
	if (!plans.empty() || m_settings.driftCorrectionEnabled) {
		if (!m_brightfieldCamera || !m_brightfieldCamera->getConnectionStatus()) {
			return fail("Overviews and drift correction need the brightfield camera, which is not connected.");
		}
	}
	if (!m_scanControl) {
		return fail("No scan control connected.");
	}
	const auto activeSlot = m_scanControl->getActiveObjectiveSlot();
	for (const auto& plan : plans) {
		if (plan.isMeasurementObjective) {
			continue;
		}
		if (!m_scanControl->hasObjectiveChanger()) {
			return fail("Overviews with another objective need a motorized objective changer.");
		}
		std::string reason;
		if (!m_scanControl->canSwitchObjectiveAutomatically(activeSlot, plan.slot, &reason)) {
			return fail(reason);
		}
		auto delta = 0.0;
		if (!m_scanControl->getFocusOffsetBetweenSlots(activeSlot, plan.slot, &delta) || std::abs(delta) > kMaxFocusDeltaUm) {
			return fail("The focus offset between objective " + std::to_string(activeSlot) + " and " + std::to_string(plan.slot)
				+ " is missing or implausible (more than " + std::to_string((int)kMaxFocusDeltaUm) + " um).");
		}
		if (m_settings.overviewObjectiveRetractUm == 0.0) {
			return fail("Set the z retract distance (Overview tab): the objective has to leave the sample before the nosepiece turns.");
		}
	}
	if (m_settings.driftCorrectionEnabled) {
		auto found = false;
		for (const auto& plan : plans) {
			found = found || plan.slot == m_settings.driftObjectiveSlot;
		}
		if (!found) {
			return fail("The drift correction objective does not take overviews: activate its overview or choose another objective.");
		}
		const auto scale = m_settings.driftObjectiveSlot == activeSlot
			? m_scanControl->getScaleCalibration() : m_scanControl->getObjectiveCalibration(m_settings.driftObjectiveSlot);
		if (!drift::stageMapFromCalibration(scale.micrometerToPixX, scale.micrometerToPixY).isValid()) {
			return fail("The drift correction objective has no valid scale calibration.");
		}
	}
	return true;
}

bool Brillouin::canCarryDriftToNextStart(bool* sameStart) {
	if (sameStart) {
		*sameStart = true;
	}
	if (!m_settings.driftCorrectionEnabled || !m_driftController.hasReference() || m_driftReferenceTiles.empty()
		|| m_driftReferenceSlot != m_settings.driftObjectiveSlot) {
		return false;
	}
	if (sameStart && m_scanControl && !m_settings.gridCoordinatesAbsolute) {
		const auto position = m_scanControl->getPosition();
		*sameStart = std::hypot(position.x - m_driftSeriesStart.x, position.y - m_driftSeriesStart.y) < kSeriesCarryOverDistanceUm;
	}
	return true;
}

bool Brillouin::driftCarriesOverToThisStart() {
	// (an absolute grid hangs on the stage coordinates, not on where the repetition starts)
	const auto sameStart = m_settings.gridCoordinatesAbsolute
		|| std::hypot(m_startPosition.x - m_driftSeriesStart.x, m_startPosition.y - m_driftSeriesStart.y) < kSeriesCarryOverDistanceUm;
	return m_settings.driftCorrectionEnabled && m_driftController.hasReference() && sameStart
		&& m_driftReferenceSlot == m_settings.driftObjectiveSlot && !m_driftReferenceTiles.empty();
}

void Brillouin::measureDriftBeforeSurfaceScan(std::unique_ptr<StorageWrapper>& storage) {
	// The surface scan sets the z of every column, so it has to be placed on the sample as it is now: the drift left over from the end of
	// the previous repetition is the offset of a sample that has kept moving since. Only worth it with a drift to carry over - without
	// one the first overview of the measurement is the reference, taken right after the scan.
	if (!useOverviewEngine() || !m_settings.useSurfaceFollow || !m_settings.driftCorrectionEnabled || m_abort || !m_scanControl) {
		return;
	}
	// the overview z follows the surface of the previous repetition (still held, as the drift is): this repetition's own is not known yet
	if (!driftCarriesOverToThisStart() || !m_surfaceZRangeValid) {
		return;
	}
	beginOverviewSeries(storage);
	m_overviewSeriesBegun = true;
	const auto directionsZ = simplemath::linspace(m_settings.zMin, m_settings.zMax, m_settings.zSteps);
	OverviewMoment moment;
	moment.kind = OVERVIEW_MOMENT_KIND::BEFORE;
	moment.zIndex = 0;
	const auto ok = runOverviewEvent(storage, moment, directionsZ, -1, true);
	// the event's entries go to the file through the storage thread; the setup of the measurement phase writes directly, so let it finish
	auto* storageObject = storage.get();
	QMetaObject::invokeMethod(storageObject, []() {}, Qt::BlockingQueuedConnection);
	if (!ok && !m_abort) {
		qWarning(logWarning()) << "Brillouin: the drift could not be measured before the surface scan; it runs with the drift carried over.";
	}
}

void Brillouin::beginOverviewSeries(std::unique_ptr<StorageWrapper>& storage) {
	m_runDriftCorrection = m_settings.driftCorrectionEnabled;
	m_runDriftObjectiveSlot = m_settings.driftObjectiveSlot;
	m_runRetractUm = m_settings.overviewObjectiveRetractUm;
	m_runUseSurfaceFollow = m_settings.useSurfaceFollow;
	m_seriesActive = true;
	m_overviewPlans = planOverviewObjectives();
	m_overviewEventCount = 0;
	m_surfaceRescanCount = 0;
	m_seriesStartSeconds = steadySeconds();
	m_measurementObjectiveSlot = m_scanControl ? m_scanControl->getActiveObjectiveSlot() : -1;
	m_runningMeasurementObjectiveSlot = m_measurementObjectiveSlot;
	const auto nan = std::numeric_limits<double>::quiet_NaN();
	m_pointOffsetX.assign(m_orderedPositions.size(), nan);
	m_pointOffsetY.assign(m_orderedPositions.size(), nan);

	drift::DriftSettings driftSettings;
	driftSettings.enabled = m_settings.driftCorrectionEnabled;
	driftSettings.minNcc = m_settings.driftMinNcc;
	driftSettings.maxStepUm = m_settings.driftMaxStepUm;
	driftSettings.extrapolate = m_settings.driftExtrapolate;
	m_driftController.setSettings(driftSettings);

	// The next repetition of a series starts where the sample was at the end of the previous one (the stage went back to the start
	// position, but the sample did not): carry the drift over if the repetition starts at the same place and with the same objective.
	// (an absolute grid hangs on the stage coordinates, not on where the repetition starts)
	if (driftCarriesOverToThisStart()) {
		m_gridOffset = m_driftController.totalUm();
	} else {
		m_driftController.reset();
		m_driftReferenceTiles.clear();
		m_driftReferenceSlot = -1;
		m_gridOffset = POINT2{ 0.0, 0.0 };
	}
	m_driftSeriesStart = m_startPosition;

	// A continued measurement picks up the engine where it stopped: the numbering of events and rescans goes on, the grid keeps the offset
	// of the last event, and every point that is in the file keeps the offset it was measured with.
	if (m_resuming) {
		m_overviewEventCount = m_resumeInfo.overviewEvents;
		m_surfaceRescanCount = m_resumeInfo.surfaceRescans;
		for (size_t ll = 0; ll < (size_t)m_resumeStartIndex && ll < m_pointOffsetX.size(); ll++) {
			auto offset = m_resumeInfo.carriedOffset;
			for (size_t e = 0; e < m_resumeInfo.eventPointIndex.size(); e++) {
				if (m_resumeInfo.eventPointIndex[e] < (int)ll) {
					offset = m_resumeInfo.eventOffsetAfter[e];
				}
			}
			m_pointOffsetX[ll] = offset.x;
			m_pointOffsetY[ll] = offset.y;
		}
		m_gridOffset = m_resumeInfo.eventOffsetAfter.empty() ? m_resumeInfo.carriedOffset : m_resumeInfo.eventOffsetAfter.back();
		m_resumeCarry = m_gridOffset;
	}
	publishGridOffset();

	// What this repetition was set up with and the calibrations of the objectives that take overviews, written before the measurement
	// starts (the storage thread is not writing yet).
	auto scalar = [&](const std::string& name, double value) {
		putDatasetDirect(storage, name, std::vector<double>{ value });
	};
	scalar("overview-engine-used", useOverviewEngine() ? 1.0 : 0.0);
	scalar("overview-trigger-used", (double)(int)m_settings.overviewTrigger);
	scalar("overview-before-measurement-used", m_settings.overviewBeforeMeasurement ? 1.0 : 0.0);
	scalar("overview-after-measurement-used", m_settings.overviewAfterMeasurement ? 1.0 : 0.0);
	scalar("overview-objective-retract-um-used", m_settings.overviewObjectiveRetractUm);
	scalar("overview-measurement-objective-slot", (double)m_measurementObjectiveSlot);
	scalar("drift-correction-used", m_settings.driftCorrectionEnabled ? 1.0 : 0.0);
	scalar("drift-objective-slot-used", (double)m_settings.driftObjectiveSlot);
	scalar("drift-min-ncc-used", m_settings.driftMinNcc);
	scalar("drift-max-step-um-used", m_settings.driftMaxStepUm);
	scalar("drift-extrapolate-used", m_settings.driftExtrapolate ? 1.0 : 0.0);
	scalar("drift-carried-over-x-um", m_gridOffset.x);
	scalar("drift-carried-over-y-um", m_gridOffset.y);
	scalar("surface-rescan-used", (double)(int)m_settings.surfaceRescan);

	std::vector<double> objectiveSlots, active, measuring, exposures, gains, fullGrids, fullStacks;
	for (const auto& plan : m_overviewPlans) {
		objectiveSlots.push_back((double)plan.slot);
		active.push_back(1.0);
		measuring.push_back(plan.isMeasurementObjective ? 1.0 : 0.0);
		exposures.push_back((double)plan.settings.exposureMs);
		gains.push_back(plan.settings.gain);
		fullGrids.push_back(plan.settings.fullGrid ? 1.0 : 0.0);
		fullStacks.push_back(plan.settings.fullStack ? 1.0 : 0.0);
	}
	putDatasetDirect(storage, "overview-objectives-slot-used", objectiveSlots);
	putDatasetDirect(storage, "overview-objectives-is-measurement-objective", measuring);
	putDatasetDirect(storage, "overview-objectives-exposure-ms-used", exposures);
	putDatasetDirect(storage, "overview-objectives-gain-used", gains);
	putDatasetDirect(storage, "overview-objectives-full-grid-used", fullGrids);
	putDatasetDirect(storage, "overview-objectives-full-stack-used", fullStacks);

	if (!m_scanControl) {
		return;
	}
	for (const auto& plan : m_overviewPlans) {
		const auto calibration = plan.isMeasurementObjective && m_measurementObjectiveSlot < 0
			? ObjectiveCalibrationData{} : m_scanControl->getObjectiveCalibration(plan.slot);
		const ScaleCalibrationData scale = plan.isMeasurementObjective ? m_scanControl->getScaleCalibration() : static_cast<ScaleCalibrationData>(calibration);
		const auto prefix = "overview-objective-" + slotName(plan.slot) + "-";
		putDatasetDirect(storage, prefix + "magnification", { calibration.magnification });
		putDatasetDirect(storage, prefix + "micrometer-to-pix-x", { scale.micrometerToPixX.x, scale.micrometerToPixX.y });
		putDatasetDirect(storage, prefix + "micrometer-to-pix-y", { scale.micrometerToPixY.x, scale.micrometerToPixY.y });
		putDatasetDirect(storage, prefix + "pix-to-micrometer-x", { scale.pixToMicrometerX.x, scale.pixToMicrometerX.y });
		putDatasetDirect(storage, prefix + "pix-to-micrometer-y", { scale.pixToMicrometerY.x, scale.pixToMicrometerY.y });
		putDatasetDirect(storage, prefix + "origin-pix", { scale.originPix.x, scale.originPix.y });
		putDatasetDirect(storage, prefix + "fov-offset-um", { calibration.hasFovOffset ? 1.0 : 0.0, calibration.fovOffsetUm.x, calibration.fovOffsetUm.y,
			calibration.fovOffsetSigmaUm });
		putDatasetDirect(storage, prefix + "focus-offset-um", { calibration.hasFocusOffset ? 1.0 : 0.0, calibration.focusOffsetUm, calibration.focusOffsetSigmaUm });
		putDatasetDirect(storage, prefix + "is-reference-objective", { calibration.isReferenceObjective ? 1.0 : 0.0 });
	}
}

void Brillouin::setLiveOverviewCamera(int slot, int exposureMs, double gain) {
	std::lock_guard<std::mutex> lock(m_liveCameraMutex);
	m_liveCamera[slot] = std::make_pair(std::max(1, exposureMs), gain);
}

bool Brillouin::liveOverviewCamera(int slot, int* exposureMs, double* gain) const {
	std::lock_guard<std::mutex> lock(m_liveCameraMutex);
	const auto found = m_liveCamera.find(slot);
	if (found == m_liveCamera.end()) {
		return false;
	}
	*exposureMs = found->second.first;
	*gain = found->second.second;
	return true;
}

void Brillouin::refreshGridOffset() {
	m_gridOffset = driftCorrectionActive() ? m_driftController.offsetAt(steadySeconds()) : POINT2{ 0.0, 0.0 };
	publishGridOffset();
}

void Brillouin::publishGridOffset() {
	// with extrapolation the offset changes at every point: only redraw the overlay (every point projected again) for a visible change
	const auto exact = m_gridOffset.x == 0.0 && m_gridOffset.y == 0.0;
	if (std::abs(m_gridOffset.x - m_publishedGridOffset.x) < (exact ? 1e-12 : 0.2)
		&& std::abs(m_gridOffset.y - m_publishedGridOffset.y) < (exact ? 1e-12 : 0.2)) {
		return;
	}
	m_publishedGridOffset = m_gridOffset;
	if (m_scanControl) {
		// Directly: the scan control lives on this (the acquisition) thread, which does not get back to its event loop while a
		// measurement runs - a queued call would only be shown after the run.
		m_scanControl->setDriftDisplayOffset(m_gridOffset);
	}
}

void Brillouin::bakeDriftIntoGrid() {
	const auto offset = m_gridOffset;
	// the next run's surface scan compares its grid with this run's: moved by exactly this, it is the same grid (see chooseSurfaceStrategy())
	m_lastRunGridShift = offset;
	// The drift reference stays as it is: a Continue of this run (from the file, with the original grid and its offsets) needs it unchanged.
	// A new Start takes it over instead, minus what the grid has been moved by (see handOverDriftToNewStart()).
	m_pendingDriftHandOver = offset;
	if (offset.x == 0.0 && offset.y == 0.0) {
		return;
	}
	// the GUI moves the grid and the ROIs by the drift and takes the drift out of the live view in one step
	m_publishedGridOffset = POINT2{ 0.0, 0.0 };
	emit(s_driftBakedIntoGrid(offset));
}

bool Brillouin::handOverDriftToNewStart() {
	if (!m_pendingDriftHandOver || !m_driftController.hasReference() || m_driftReferenceTiles.empty()) {
		return false;
	}
	const auto moved = *m_pendingDriftHandOver;
	m_pendingDriftHandOver.reset();
	// The grid now carries `moved`: the drift still to apply is the drift since the series began minus that. The reference images stay -
	// the next overview is registered against them, so the drift since the end of the last run is measured (the step is computed from the
	// stage positions the frames were taken at, not from the grid). Their place in the planned layout moves with the grid, so they pair
	// with the same tiles of the moved grid.
	m_driftController.setReference(m_driftController.referenceTime(), POINT2{ m_driftController.totalUm().x - moved.x, m_driftController.totalUm().y - moved.y });
	for (auto& tile : m_driftReferenceTiles) {
		if (std::isfinite(tile.anchorUm.x) && std::isfinite(tile.anchorUm.y)) {
			tile.anchorUm.x += moved.x;
			tile.anchorUm.y += moved.y;
		}
	}
	m_gridOffset = m_driftController.totalUm();
	return true;
}

bool Brillouin::grabBrightfieldFrame(int exposureMs, double gain, std::vector<std::byte>* image, CAMERA_SETTINGS* usedSettings) {
	if (!m_brightfieldCamera || !m_brightfieldCamera->getConnectionStatus()) {
		return false;
	}
	// The same settings the per-z overview uses (see captureOverviewBrightfield()): the camera's current ROI, one software-triggered frame.
	auto settings = m_brightfieldCamera->getSettings();
	settings.exposureTime = 1e-3 * std::max(1, exposureMs);
	settings.gain = gain;
	settings.frameCount = 1;
	settings.readout.triggerMode = L"Software";
	settings.readout.cycleMode = L"Fixed";
	m_brightfieldCamera->startAcquisition(settings);
	settings = m_brightfieldCamera->getSettings();
	if (settings.roi.bytesPerFrame <= 0) {
		m_brightfieldCamera->stopAcquisition();
		return false;
	}
	image->assign((size_t)settings.roi.bytesPerFrame, std::byte{ 0 });
	m_brightfieldCamera->getImageForAcquisition(image->data(), false);
	m_brightfieldCamera->stopAcquisition();
	*usedSettings = settings;
	return true;
}

bool Brillouin::waitForStage(const POINT3& target, double timeoutS) {
	if (!m_scanControl) {
		return false;
	}
	// at the target: within 2 um on every axis; done moving elsewhere (a limit, a stand that stops short): no change of more than
	// 0.2 um for 0.5 s, once 1.5 s have passed (a stand that has not started to move yet looks the same)
	const auto isNear = [](const POINT3& a, const POINT3& b, double tolerance) {
		return std::abs(a.x - b.x) < tolerance && std::abs(a.y - b.y) < tolerance && std::abs(a.z - b.z) < tolerance;
	};
	const auto start = steadySeconds();
	auto last = m_scanControl->getPosition();
	auto restingSince = start;
	while (true) {
		const auto now = m_scanControl->getPosition();
		if (isNear(now, target, 2.0)) {
			return true;
		}
		const auto t = steadySeconds();
		if (!isNear(now, last, 0.2)) {
			last = now;
			restingSince = t;
		}
		if ((t - restingSince > 0.5 && t - start > 1.5) || t - start > timeoutS) {
			qWarning(logWarning()) << "Brillouin: the stage stopped at" << now.x << now.y << now.z << "um, not at the target" << target.x << target.y
				<< target.z << "um.";
			return false;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(20));
	}
}

void Brillouin::returnToMeasurementObjective(int measurementSlot, const POINT3& restorePosition, bool restorePositionToo) {
	if (!m_scanControl) {
		return;
	}
	closeDoseProtectionShutter();
	if (m_scanControl->getActiveObjectiveSlot() != measurementSlot) {
		// away from the sample before the nosepiece turns, as on the way there - and only turn once z is really there
		const auto before = m_scanControl->getPosition();
		m_scanControl->movePosition(POINT3{ 0.0, 0.0, overviewRetractUmActive() });
		waitForStage(before + POINT3{ 0.0, 0.0, overviewRetractUmActive() }, 30.0);
		if (std::abs(m_scanControl->getPosition().z - before.z) < 0.5 * std::abs(overviewRetractUmActive())) {
			qCritical(logCritical()) << "Brillouin: z did not reach the retract position, the nosepiece is not turned back; the measurement stops.";
			m_abort = true;
			return;
		}
		std::string error;
		if (!m_scanControl->switchObjectiveSlot(measurementSlot, &error)) {
			qCritical(logCritical()) << "Brillouin: could not turn back to the measuring objective:" << error.c_str();
			m_abort = true;
			return;
		}
	}
	if (restorePositionToo) {
		m_scanControl->setPositionCompensated(restorePosition);
		waitForStage(restorePosition, 30.0);
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
}

bool Brillouin::captureObjectiveOverview(
	std::unique_ptr<StorageWrapper>& storage, const OverviewObjectivePlan& plan, const OverviewMoment& moment,
	const std::vector<double>& directionsZ, int eventIndex, int objectiveOrder, gsl::index pointIndex,
	std::vector<drift::TileFrame>* driftTiles, OverviewCaptureLog* log
) {
	log->slot = plan.slot;
	log->isMeasurementObjective = plan.isMeasurementObjective;
	if (!m_scanControl || directionsZ.empty()) {
		log->skippedReason = "no scan control";
		return true;
	}
	const auto zIndex = std::clamp(moment.zIndex, 0, (int)directionsZ.size() - 1);
	const auto offset = gridOffset();
	const auto xyOrigin = m_settings.gridCoordinatesAbsolute ? POINT2{ 0.0, 0.0 } : POINT2{ m_startPosition.x, m_startPosition.y };
	// the per-z overview of the measuring objective keeps the numbering and channel it has always had
	const auto legacyNumbering = plan.isMeasurementObjective && moment.kind == OVERVIEW_MOMENT_KIND::PLANE;
	const std::string channel = legacyNumbering ? "Brightfield z overview" : "Brightfield overview";

	// targets: per tile, the stack of stage positions
	std::vector<std::vector<POINT3>> tiles;
	ScaleCalibrationData scale;
	ObjectiveCalibrationData calibration;
	POINT2 fovCompensation{ 0.0, 0.0 };
	auto focusDelta = 0.0;
	if (plan.isMeasurementObjective) {
		scale = m_scanControl->getScaleCalibration();
		calibration = m_scanControl->getObjectiveCalibration(plan.slot);
		for (const auto& point : overviewCapturePoints(zIndex, directionsZ)) {
			std::vector<POINT3> stack;
			for (const auto z : point.zAbs) {
				stack.push_back(POINT3{ point.xy.x + offset.x, point.xy.y + offset.y, z });
			}
			tiles.push_back(stack);
		}
	} else {
		std::string reason;
		if (!m_scanControl->canSwitchObjectiveAutomatically(m_measurementObjectiveSlot, plan.slot, &reason)) {
			log->skippedReason = reason;
			qWarning(logWarning()) << "Brillouin: overview with objective" << plan.slot << "skipped:" << reason.c_str();
			return true;
		}
		calibration = m_scanControl->getObjectiveCalibration(plan.slot);
		const auto home = m_scanControl->getObjectiveCalibration(m_measurementObjectiveSlot);
		scale = calibration;
		// The field of view of the other objective is centred on the same sample point by the difference of the FOV offsets (the stage move
		// that re-centres a feature, relative to the reference objective, see ObjectiveCalibrationData), and its focus plane lies at the
		// difference of the focus offsets away in z.
		const auto fovOf = [](const ObjectiveCalibrationData& data) { return data.isReferenceObjective || !data.hasFovOffset ? POINT2{ 0.0, 0.0 } : data.fovOffsetUm; };
		fovCompensation = POINT2{ fovOf(calibration).x - fovOf(home).x, fovOf(calibration).y - fovOf(home).y };
		if (!m_scanControl->getFocusOffsetBetweenSlots(m_measurementObjectiveSlot, plan.slot, &focusDelta)) {
			log->skippedReason = "no focus offset";
			return true;
		}
		const auto footprint = overviewTileFootprintUmFor(scale);
		const auto centres = plan.settings.fullGrid ? overviewTileCentersXYForFootprint(footprint) : std::vector<POINT2>{ overviewGridCenterXYForFootprint(footprint) };
		// the z planes of the measurement grid (the grid's z range, or the range of the found surface), at this objective's focus
		auto stackZ = overviewStackZAbsFor(plan.settings.fullStack, zIndex, directionsZ);
		for (auto& z : stackZ) {
			z += focusDelta;
		}
		for (const auto& centre : centres) {
			std::vector<POINT3> stack;
			for (const auto z : stackZ) {
				stack.push_back(POINT3{ centre.x + xyOrigin.x + fovCompensation.x + offset.x, centre.y + xyOrigin.y + fovCompensation.y + offset.y, z });
			}
			tiles.push_back(stack);
		}
	}
	if (tiles.empty()) {
		log->skippedReason = "no tiles";
		return true;
	}
	// A target position puts the LASER on that point (the stage goes to target - scanner), and the laser is wherever the marker was set in the
	// image - not in the middle of it; with the calibration's origin at pixel (0,0) a pixel p shows the stage position
	// (target - scanner + A * (p - origin) - FOV offset). To have the planned point in the middle of the image, the target is moved by
	// scanner - A * (centre - origin) + the FOV offset of the objective in the beam. The grid targets carry no FOV offset (it is only added when
	// a target is projected into the image), so the offset of the measuring objective is added here and, for another objective, the
	// difference to it is already in fovCompensation: together the absolute offset of the objective that takes the image.
	POINT2 centreShift{ 0.0, 0.0 };
	if (m_brightfieldCamera) {
		const auto roi = m_brightfieldCamera->getSettings().roi;
		const auto scanner = m_scanControl->getPosition(PositionType::SCANNER);
		const auto homeCalibration = m_scanControl->getObjectiveCalibration(m_measurementObjectiveSlot);
		const auto fovHome = homeCalibration.isReferenceObjective || !homeCalibration.hasFovOffset ? POINT2{ 0.0, 0.0 } : homeCalibration.fovOffsetUm;
		const auto dx = 0.5 * (double)roi.width_binned - scale.originPix.x;
		const auto dy = 0.5 * (double)roi.height_binned - scale.originPix.y;
		centreShift = POINT2{
			scanner.x - (dx * scale.pixToMicrometerX.x + dy * scale.pixToMicrometerY.x) + fovHome.x,
			scanner.y - (dx * scale.pixToMicrometerX.y + dy * scale.pixToMicrometerY.y) + fovHome.y
		};
		if (!std::isfinite(centreShift.x) || !std::isfinite(centreShift.y)) {
			centreShift = POINT2{ 0.0, 0.0 };
		}
		for (auto& stack : tiles) {
			for (auto& target : stack) {
				target.x += centreShift.x;
				target.y += centreShift.y;
			}
		}
	}
	log->tiles = (int)tiles.size();
	log->stackDepth = (int)tiles.front().size();
	log->fovCompensationUm = fovCompensation;
	log->focusDeltaUm = focusDelta;
	log->retractUm = plan.isMeasurementObjective ? 0.0 : overviewRetractUmActive();

	// The cycle with another objective. The guard turns the nosepiece back to the measuring objective on every way out (also an abort).
	struct ObjectiveCycleGuard {
		Brillouin* self;
		int measurementSlot{ -1 };
		POINT3 restorePosition{ 0.0, 0.0, 0.0 };
		bool switched{ false };
		~ObjectiveCycleGuard() {
			if (switched) {
				self->returnToMeasurementObjective(measurementSlot, restorePosition, false);
			}
		}
	} guard{ this, m_measurementObjectiveSlot };

	const auto positionBefore = m_scanControl->getPosition();
	log->positionBeforeUm = positionBefore;
	guard.restorePosition = positionBefore;
	if (!plan.isMeasurementObjective) {
		closeDoseProtectionShutter();
		const auto t0 = steadySeconds();
		m_scanControl->movePosition(POINT3{ 0.0, 0.0, overviewRetractUmActive() });
		// the nosepiece only turns once the objective is away from the sample (at least half the retract distance, if a limit stopped it)
		waitForStage(positionBefore + POINT3{ 0.0, 0.0, overviewRetractUmActive() }, 30.0);
		if (std::abs(m_scanControl->getPosition().z - positionBefore.z) < 0.5 * std::abs(overviewRetractUmActive())) {
			log->skippedReason = "z retract not reached";
			qCritical(logCritical()) << "Brillouin: z did not reach the retract position, the overview with objective" << plan.slot << "is skipped.";
			m_scanControl->setPositionCompensated(positionBefore);
			waitForStage(positionBefore, 30.0);
			return !m_abort;
		}
		std::string error;
		if (!m_scanControl->switchObjectiveSlot(plan.slot, &error)) {
			qCritical(logCritical()) << "Brillouin: could not turn to objective" << plan.slot << ":" << error.c_str();
			log->skippedReason = error;
			// the changer may be anywhere: try to go back to where it was
			guard.switched = true;
			return !m_abort;
		}
		guard.switched = true;
		log->switchToSeconds = steadySeconds() - t0;
	}

	if (!m_overviewBrightfieldPresetSet) {
		m_scanControl->setPreset(ScanPreset::SCAN_BRIGHTFIELD);
		m_overviewBrightfieldPresetSet = true;
	}

	// per-image bookkeeping shared by all images of this overview
	auto totalPerZ = (size_t)0;
	for (const auto& stack : tiles) {
		totalPerZ += stack.size();
	}
	const auto firstNumber = legacyNumbering
		? (int)((size_t)zIndex * totalPerZ)
		: kEventImageBase + eventIndex * kEventImageSpacing + objectiveOrder * kObjectiveImageSpacing;
	log->firstImageNumber = firstNumber;
	const auto captureStart = steadySeconds();
	auto flat = (size_t)0;
	for (size_t tileIndex = 0; tileIndex < tiles.size(); tileIndex++) {
		const auto centreSlice = tiles[tileIndex].size() / 2;
		for (size_t stackIndex = 0; stackIndex < tiles[tileIndex].size(); stackIndex++, flat++) {
			if (m_abort) {
				return false;
			}
			const auto target = tiles[tileIndex][stackIndex];
			m_scanControl->setPositionCompensated(target);
			waitForStage(target, 30.0);
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			const auto stageRaw = m_scanControl->getPosition();

			std::vector<std::byte> image;
			CAMERA_SETTINGS used;
			// exposure and gain as the operator has them now (they can be changed during the measurement)
			auto exposureMs = plan.settings.exposureMs;
			auto gain = plan.settings.gain;
			liveOverviewCamera(plan.slot, &exposureMs, &gain);
			if (!grabBrightfieldFrame(exposureMs, gain, &image, &used)) {
				qWarning(logWarning()) << "Brillouin: overview frame could not be captured (objective" << plan.slot << ")";
				continue;
			}
			int imageNumber = 0;
			if (legacyNumbering) {
				imageNumber = firstNumber + (int)flat;
			} else if ((int)tileIndex >= kMaxTilesPerOverview || (int)stackIndex >= kMaxStackPerTile) {
				qWarning(logWarning()) << "Brillouin: overview image not saved: too many tiles or stack slices.";
				continue;
			} else {
				imageNumber = firstNumber + (int)tileIndex * 100 + (int)stackIndex;
			}

			std::vector<std::pair<std::string, double>> numbers = {
				{ "overview_event", (double)eventIndex }, { "overview_kind", (double)(int)moment.kind },
				{ "overview_z_index", (double)zIndex }, { "overview_point_index", (double)pointIndex },
				{ "overview_objective_order", (double)objectiveOrder }, { "overview_objective_slot", (double)plan.slot },
				{ "overview_is_measurement_objective", plan.isMeasurementObjective ? 1.0 : 0.0 },
				{ "overview_tile_index", (double)tileIndex }, { "overview_tile_count", (double)tiles.size() },
				{ "overview_stack_index", (double)stackIndex }, { "overview_stack_count", (double)tiles[tileIndex].size() },
				{ "overview_is_centre_slice", stackIndex == centreSlice ? 1.0 : 0.0 },
				{ "overview_seconds_since_start", steadySeconds() - m_seriesStartSeconds },
				{ "grid_offset_x_um", offset.x }, { "grid_offset_y_um", offset.y },
				{ "fov_compensation_x_um", fovCompensation.x }, { "fov_compensation_y_um", fovCompensation.y },
				{ "image_centre_shift_x_um", centreShift.x }, { "image_centre_shift_y_um", centreShift.y },
				{ "focus_delta_um", focusDelta }, { "objective_magnification", calibration.magnification },
				{ "stage_position_raw_x_um", stageRaw.x }, { "stage_position_raw_y_um", stageRaw.y }, { "stage_position_raw_z_um", stageRaw.z },
				{ "scale_micrometer_to_pix_x_col", scale.micrometerToPixX.x }, { "scale_micrometer_to_pix_x_row", scale.micrometerToPixX.y },
				{ "scale_micrometer_to_pix_y_col", scale.micrometerToPixY.x }, { "scale_micrometer_to_pix_y_row", scale.micrometerToPixY.y },
				{ "scale_origin_col", scale.originPix.x }, { "scale_origin_row", scale.originPix.y }
			};
			std::vector<std::pair<std::string, std::string>> strings = {
				{ "overview_kind_name", momentKindName(moment.kind) }, { "objective_name", calibration.objectiveName }
			};
			const auto targetGrid = rawPositionToGridFrame(target);
			const auto stageGrid = rawPositionToGridFrame(stageRaw);
			if (stackIndex == centreSlice) {
				writeLivePreview("event" + padded(eventIndex, 4) + "_obj" + slotName(plan.slot) + "_kind" + std::to_string((int)moment.kind) + "_z"
					+ std::to_string(zIndex) + "_tile" + padded((int)tileIndex, 2), image, used);
			}
			if (used.readout.dataType == "unsigned short") {
				enqueueOverviewEventImage<unsigned short>(storage, imageNumber, used, image, targetGrid, stageGrid, m_fileRepetition, channel, numbers, strings);
				log->imagesSaved++;
			} else if (used.readout.dataType == "unsigned char") {
				enqueueOverviewEventImage<unsigned char>(storage, imageNumber, used, image, targetGrid, stageGrid, m_fileRepetition, channel, numbers, strings);
				log->imagesSaved++;
			}

			// the centre slice of the tile's stack is what the drift is measured on
			if (driftTiles && stackIndex == centreSlice) {
				std::vector<float> pixels;
				int width = 0;
				int height = 0;
				if (frameToFloat(image, used, &pixels, &width, &height)) {
					drift::TileFrame tile;
					tile.positionUm = POINT2{ stageRaw.x, stageRaw.y };
						// the planned position of the tile without the drift offset and (other objective) FOV compensation: the same for
						// the same tile of every overview, however far the grid has moved since
						tile.anchorUm = POINT2{ target.x - offset.x - fovCompensation.x - centreShift.x, target.y - offset.y - fovCompensation.y - centreShift.y };
					tile.image = registration::prepare(pixels, width, height, registration::automaticFactor(width, height));
					if (tile.image.isValid()) {
						driftTiles->push_back(std::move(tile));
					}
				}
			}
		}
	}
	log->captureSeconds = steadySeconds() - captureStart;
	log->captured = log->imagesSaved > 0;

	if (!plan.isMeasurementObjective) {
		const auto t0 = steadySeconds();
		guard.switched = false;
		returnToMeasurementObjective(m_measurementObjectiveSlot, positionBefore, true);
		log->switchBackSeconds = steadySeconds() - t0;
		if (m_abort) {
			return false;
		}
	}
	return !m_abort;
}

void Brillouin::processDriftOverview(const std::vector<drift::TileFrame>& tiles, int slot, double timeSeconds, OverviewDriftLog* log) {
	const auto t0 = steadySeconds();
	log->evaluated = true;
	const ScaleCalibrationData scale = slot == m_measurementObjectiveSlot
		? m_scanControl->getScaleCalibration() : static_cast<ScaleCalibrationData>(m_scanControl->getObjectiveCalibration(slot));
	const auto map = drift::stageMapFromCalibration(scale.micrometerToPixX, scale.micrometerToPixY);

	if (!m_driftController.hasReference() || m_driftReferenceSlot != slot || m_driftReferenceTiles.empty()) {
		// the first overview of the series (or after a change of the drift objective): the reference. A drift measured earlier is kept.
		// (a continued measurement starts from the offset it had when it stopped)
		const auto carried = m_driftController.hasReference() ? m_driftController.totalUm() : m_resumeCarry;
		m_resumeCarry = POINT2{ 0.0, 0.0 };
		const auto update = m_driftController.setReference(timeSeconds, carried);
		m_driftReferenceTiles = tiles;
		m_driftReferenceSlot = slot;
		log->decision = update.decision;
		log->totalUm = update.totalUm;
	} else {
		const auto maxShiftUm = std::clamp(1.5 * m_settings.driftMaxStepUm, 50.0, 500.0);
		const auto step = drift::measureStep(m_driftReferenceTiles, tiles, map, m_settings.driftMinNcc, maxShiftUm);
		const auto update = m_driftController.update(step, timeSeconds);
		log->decision = update.decision;
		log->stepUm = update.stepUm;
		log->totalUm = update.totalUm;
		log->meanNcc = update.meanNcc;
		log->tilesUsed = update.tilesUsed;
		log->tilesCompared = step.tilesCompared;
		log->secondsSinceReference = update.secondsSinceReference;
		if (update.decision == drift::DriftDecision::APPLIED) {
			m_driftReferenceTiles = tiles;
		}
	}
	refreshGridOffset();
	log->gridOffsetUm = m_gridOffset;
	log->registrationSeconds = steadySeconds() - t0;
	qInfo(logInfo()) << "Drift overview:" << (int)log->decision << "step" << log->stepUm.x << log->stepUm.y << "um, grid offset"
		<< m_gridOffset.x << m_gridOffset.y << "um, ncc" << log->meanNcc << ", tiles" << log->tilesUsed << "of" << log->tilesCompared;
}

bool Brillouin::runOverviewMoments(
	std::unique_ptr<StorageWrapper>& storage, const std::vector<OverviewMoment>& moments, const std::vector<double>& directionsZ, gsl::index pointIndex
) {
	for (const auto& moment : moments) {
		if (!runOverviewEvent(storage, moment, directionsZ, pointIndex)) {
			return false;
		}
	}
	return !m_abort;
}

bool Brillouin::runOverviewEvent(
	std::unique_ptr<StorageWrapper>& storage, const OverviewMoment& moment, const std::vector<double>& directionsZ, gsl::index pointIndex,
	bool driftOnly
) {
	if (!m_scanControl || m_abort) {
		return false;
	}
	if (m_overviewPlans.empty()) {
		return true;
	}
	const auto eventIndex = m_overviewEventCount++;
	const auto eventSeconds = steadySeconds();

	// brightfield for the whole event: one preset switch, the laser shutter closed
	// (the brightfield preset is set by the first objective, once it is the one in the beam)
	m_overviewBrightfieldPresetSet = false;
	m_scanControl->setRLShutterOpen(false);

	std::vector<OverviewCaptureLog> captures;
	OverviewDriftLog driftLog;
	std::vector<drift::TileFrame> driftTiles;
	auto driftSlot = -1;
	auto objectiveOrder = 0;
	for (const auto& plan : m_overviewPlans) {
		if (driftOnly && plan.slot != driftObjectiveSlotActive()) {
			continue;
		}
		OverviewCaptureLog captureLog;
		const auto isDriftObjective = driftCorrectionActive() && plan.slot == driftObjectiveSlotActive();
		std::vector<drift::TileFrame> tiles;
		const auto ok = captureObjectiveOverview(
			storage, plan, moment, directionsZ, eventIndex, objectiveOrder, pointIndex, isDriftObjective ? &tiles : nullptr, &captureLog);
		captures.push_back(captureLog);
		objectiveOrder++;
		if (!ok) {
			return false;
		}
		if (isDriftObjective && !tiles.empty()) {
			driftTiles = std::move(tiles);
			driftSlot = plan.slot;
		}
	}
	m_scanControl->setPreset(ScanPreset::SCAN_BRILLOUIN);
	setRLShutterRestingState();

	if (driftCorrectionActive()) {
		if (!driftTiles.empty()) {
			processDriftOverview(driftTiles, driftSlot, eventSeconds, &driftLog);
		} else {
			qWarning(logWarning()) << "Brillouin: the drift correction objective produced no overview in this event; the grid stays where it is.";
		}
	}
	writeOverviewEvent(storage, eventIndex, moment, pointIndex, eventSeconds - m_seriesStartSeconds, captures, driftLog);

	// The overview moved the stage away from the grid point: back to it, with the offset as it is now (the drift just measured, if any).
	if (pointIndex >= 0 && pointIndex < (gsl::index)m_orderedPositions.size()) {
		waitForStage(approachGridPosition(m_orderedPositions[pointIndex]), 30.0);
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
	return !m_abort;
}

void Brillouin::writeOverviewEvent(
	std::unique_ptr<StorageWrapper>& storage, int eventIndex, const OverviewMoment& moment, gsl::index pointIndex, double timeSeconds,
	const std::vector<OverviewCaptureLog>& captures, const OverviewDriftLog& driftLog
) {
	const auto prefix = "overview-event-" + padded(eventIndex, 4) + "-";
	auto put = [&](const std::string& name, const std::vector<double>& values) {
		putDataset(storage, prefix + name, values);
	};
	put("kind", { (double)(int)moment.kind });
	put("z-index", { (double)moment.zIndex });
	put("point-index", { (double)pointIndex });
	put("seconds-since-start", { timeSeconds });
	put("epoch-seconds", { (double)QDateTime::currentSecsSinceEpoch() });
	put("repetition", { (double)m_fileRepetition });

	std::vector<double> slot, measuring, captured, first, tiles, depth, saved, retract, toSeconds, captureSeconds, backSeconds, fovX, fovY, focus,
		beforeX, beforeY, beforeZ;
	for (const auto& log : captures) {
		slot.push_back((double)log.slot);
		measuring.push_back(log.isMeasurementObjective ? 1.0 : 0.0);
		captured.push_back(log.captured ? 1.0 : 0.0);
		first.push_back((double)log.firstImageNumber);
		tiles.push_back((double)log.tiles);
		depth.push_back((double)log.stackDepth);
		saved.push_back((double)log.imagesSaved);
		retract.push_back(log.retractUm);
		toSeconds.push_back(log.switchToSeconds);
		captureSeconds.push_back(log.captureSeconds);
		backSeconds.push_back(log.switchBackSeconds);
		fovX.push_back(log.fovCompensationUm.x);
		fovY.push_back(log.fovCompensationUm.y);
		focus.push_back(log.focusDeltaUm);
		beforeX.push_back(log.positionBeforeUm.x);
		beforeY.push_back(log.positionBeforeUm.y);
		beforeZ.push_back(log.positionBeforeUm.z);
	}
	put("objective-slot", slot);
	put("objective-is-measurement-objective", measuring);
	put("objective-captured", captured);
	put("objective-first-image-number", first);
	put("objective-tiles", tiles);
	put("objective-stack-depth", depth);
	put("objective-images-saved", saved);
	put("objective-retract-um", retract);
	put("objective-switch-to-seconds", toSeconds);
	put("objective-capture-seconds", captureSeconds);
	put("objective-switch-back-seconds", backSeconds);
	put("objective-fov-compensation-x-um", fovX);
	put("objective-fov-compensation-y-um", fovY);
	put("objective-focus-delta-um", focus);
	put("objective-stage-before-x-um", beforeX);
	put("objective-stage-before-y-um", beforeY);
	put("objective-stage-before-z-um", beforeZ);

	put("drift-evaluated", { driftLog.evaluated ? 1.0 : 0.0 });
	// 0 reference, 1 applied, 2 no match, 3 low confidence, 4 implausible
	put("drift-decision", { (double)(int)driftLog.decision });
	put("drift-step-um", { driftLog.stepUm.x, driftLog.stepUm.y });
	put("drift-total-um", { driftLog.totalUm.x, driftLog.totalUm.y });
	put("grid-offset-after-um", { driftLog.evaluated ? driftLog.gridOffsetUm.x : m_gridOffset.x, driftLog.evaluated ? driftLog.gridOffsetUm.y : m_gridOffset.y });
	put("drift-mean-ncc", { driftLog.meanNcc });
	put("drift-tiles-compared-used", { (double)driftLog.tilesCompared, (double)driftLog.tilesUsed });
	put("drift-seconds-since-reference", { driftLog.secondsSinceReference });
	put("drift-registration-seconds", { driftLog.registrationSeconds });

	// the same in the live log next to the file (one line per event)
	std::string objectives;
	for (const auto& capture : captures) {
		objectives += (objectives.empty() ? "" : ";") + slotName(capture.slot) + ":" + (capture.captured ? "ok" : (capture.skippedReason.empty() ? "none" : "skipped"))
			+ ":" + std::to_string(capture.imagesSaved) + "img:" + std::to_string((int)std::lround(capture.switchToSeconds + capture.switchBackSeconds)) + "s";
	}
	auto line = std::string{ "event," } + QDateTime::currentDateTime().toString(Qt::ISODate).toStdString() + "," + std::to_string(eventIndex) + "," + momentKindName(moment.kind)
		+ "," + std::to_string(pointIndex) + "," + objectives;
	if (driftLog.evaluated) {
		// 0 reference, 1 applied, 2 no match, 3 low confidence, 4 implausible
		line += ",drift " + std::to_string((int)driftLog.decision) + " step " + std::to_string(driftLog.stepUm.x) + " " + std::to_string(driftLog.stepUm.y)
			+ " ncc " + std::to_string(driftLog.meanNcc) + " tiles " + std::to_string(driftLog.tilesUsed) + "/" + std::to_string(driftLog.tilesCompared);
	}
	line += ",grid offset " + std::to_string(m_gridOffset.x) + " " + std::to_string(m_gridOffset.y);
	appendLiveLog(line);
}


// The previews are written on a thread of their own: the measurement only copies the frame and moves on. At most kMaxQueued frames wait; a
// frame that finds the queue full is dropped (the preview is for looking at, the data is in the file).
class LivePreviewWorker {
public:
	struct Job {
		std::string path;
		std::vector<std::byte> image;
		CAMERA_SETTINGS settings;
	};

	LivePreviewWorker() : m_thread([this]() { run(); }) {}
	~LivePreviewWorker() {
		{
			std::lock_guard<std::mutex> lock(m_mutex);
			m_stop = true;
		}
		m_condition.notify_all();
		m_thread.join();
	}
	bool enqueue(Job&& job) {
		{
			std::lock_guard<std::mutex> lock(m_mutex);
			if (m_queue.size() >= kMaxQueued) {
				return false;
			}
			m_queue.push_back(std::move(job));
		}
		m_condition.notify_one();
		return true;
	}
	bool failed() const { return m_failed; }

private:
	static constexpr size_t kMaxQueued = 8;

	void run() {
		for (;;) {
			Job job;
			{
				std::unique_lock<std::mutex> lock(m_mutex);
				m_condition.wait(lock, [this]() { return m_stop || !m_queue.empty(); });
				if (m_queue.empty()) {
					return;
				}
				job = std::move(m_queue.front());
				m_queue.pop_front();
			}
			render(job);
		}
	}

	void render(const Job& job) {
		try {
			std::vector<float> pixels;
			int width = 0;
			int height = 0;
			if (!frameToFloat(job.image, job.settings, &pixels, &width, &height) || width <= 0 || height <= 0) {
				return;
			}
			std::filesystem::create_directories(std::filesystem::path(job.path).parent_path());
			// contrast from a sparse sample (1 % to 99.5 %): this is only to look at
			std::vector<float> sample;
			for (size_t i = 0; i < pixels.size(); i += 17) {
				sample.push_back(pixels[i]);
			}
			std::sort(sample.begin(), sample.end());
			const auto low = sample[(size_t)(0.01 * (double)(sample.size() - 1))];
			auto high = sample[(size_t)(0.995 * (double)(sample.size() - 1))];
			if (high <= low) {
				high = low + 1.0f;
			}
			QImage preview(width, height, QImage::Format_Grayscale8);
			for (int row = 0; row < height; row++) {
				auto* line = preview.scanLine(row);
				for (int col = 0; col < width; col++) {
					const auto value = (pixels[(size_t)row * (size_t)width + (size_t)col] - low) / (high - low);
					line[col] = (uchar)std::clamp((int)std::lround(255.0f * value), 0, 255);
				}
			}
			if (!preview.save(QString::fromStdString(job.path), "PNG", 80)) {
				m_failed = true;
			}
		} catch (const std::exception&) {
			m_failed = true;
		}
	}

	std::mutex m_mutex;
	std::condition_variable m_condition;
	std::deque<Job> m_queue;
	bool m_stop{ false };
	std::atomic<bool> m_failed{ false };
	std::thread m_thread;
};

std::string Brillouin::livePreviewFolder() const {
	if (!m_acquisition) {
		return {};
	}
	const std::filesystem::path file = std::filesystem::path(m_acquisition->getCurrentFolder()) / m_acquisition->getCurrentFilename();
	return (file.parent_path() / (file.stem().string() + "_live")).string();
}

void Brillouin::writeLivePreview(const std::string& name, const std::vector<std::byte>& image, const CAMERA_SETTINGS& settings) {
	if (!m_settings.livePreviews || m_livePreviewFailed || image.empty()) {
		return;
	}
	const auto folder = livePreviewFolder();
	if (folder.empty()) {
		return;
	}
	if (!m_livePreviewWorker) {
		m_livePreviewWorker = std::make_shared<LivePreviewWorker>();
	}
	if (m_livePreviewWorker->failed()) {
		m_livePreviewFailed = true;
		return;
	}
	// the raw frame is copied (a few ms at most), everything else - contrast, PNG, disk - happens on the worker's thread
	if (!m_livePreviewWorker->enqueue(LivePreviewWorker::Job{ folder + "/" + name + ".png", image, settings })) {
		qWarning(logWarning()) << "Brillouin: live preview" << name.c_str() << "dropped, the writer is behind.";
	}
}

void Brillouin::appendLiveLog(const std::string& line) {
	if (!m_settings.livePreviews || m_livePreviewFailed) {
		return;
	}
	try {
		const auto folder = livePreviewFolder();
		if (folder.empty()) {
			return;
		}
		std::filesystem::create_directories(folder);
		const auto path = std::filesystem::path(folder) / "events.csv";
		const auto isNew = !std::filesystem::exists(path);
		std::ofstream out(path, std::ios::app);
		if (!out) {
			m_livePreviewFailed = true;
			return;
		}
		if (isNew) {
			out << "kind,time,number,what,point,details\n";
		}
		out << line << "\n";
	} catch (const std::exception&) {
		m_livePreviewFailed = true;
	}
}

void Brillouin::writePointGridOffsets(std::unique_ptr<StorageWrapper>& storage) {
	if (m_pointOffsetX.empty()) {
		return;
	}
	putDataset(storage, "grid-offset-per-point-x-um", m_pointOffsetX);
	putDataset(storage, "grid-offset-per-point-y-um", m_pointOffsetY);
}

void Brillouin::repeatSurfaceScan(std::unique_ptr<StorageWrapper>& storage, gsl::index pointIndex, const std::vector<double>& directionsZ) {
	if (!m_scanControl || !useSurfaceFollowActive() || m_abort || directionsZ.empty()) {
		return;
	}
	const auto t0 = steadySeconds();
	const auto round = m_surfaceRescanCount++;

	// runSurfacePreScan() puts the found surface on the flat plan z of every point, so the points start from that again; the points that are
	// measured already, and those the new scan finds no surface for, get their previous z back afterwards.
	const auto previous = m_orderedPositions;
	const auto previousRelative = m_orderedPositionsRelative;
	const auto zOrigin = m_settings.gridCoordinatesAbsolute ? resolvedGridOriginUm().z : m_startPosition.z;
	for (size_t i = 0; i < m_orderedPositions.size(); i++) {
		const auto zIndex = std::clamp(m_orderedIndices[i].z, 0, (int)directionsZ.size() - 1);
		const auto flat = zOrigin + directionsZ[(size_t)zIndex];
		m_orderedPositions[i].z = flat;
		m_orderedPositionsRelative[i].z = flat - m_startPosition.z;
	}
	m_scanControl->setPreset(ScanPreset::SCAN_BRILLOUIN);
	setRLShutterRestingState();
	// The previous scan (the first one, or the last repetition of this) is the prior of this one: the offset found in its first columns
	// moves the old surface of every other column to where the new one is expected, so the search starts next to it (see m_surfacePriorZUm).
	m_surfacePriorZUm = m_surfacePreScanZUm;
	m_surfacePriorValid = m_surfacePreScanFoundMask;
	const auto hadPrior = !m_surfacePriorZUm.empty();
	const auto result = runSurfacePreScan();
	m_surfacePriorZUm.clear();
	m_surfacePriorValid.clear();
	if (m_abort) {
		return;
	}
	auto failed = 0;
	for (size_t i = 0; i < m_orderedPositions.size(); i++) {
		const auto found = m_surfaceFoundXYIndices.count(std::make_pair(m_orderedIndices[i].x, m_orderedIndices[i].y)) > 0;
		if ((gsl::index)i <= pointIndex || !result.success || !found) {
			if ((gsl::index)i > pointIndex) {
				failed++;
			}
			m_orderedPositions[i].z = previous[i].z;
			m_orderedPositionsRelative[i].z = previousRelative[i].z;
		}
	}
	if (result.success) {
		if (m_settings.gridCoordinatesAbsolute) {
			emit(s_orderedPositionsChanged(m_orderedPositions, true));
		} else {
			emit(s_orderedPositionsChanged(m_orderedPositionsRelative, false));
		}
	}

	// Everything of this round goes to the file: the coarse scan as measured (with the curves behind it), the threshold and medium
	// reference it used, the dense result and the z every point is measured at from now on.
	const auto prefix = "surface-rescan-" + padded(round, 4) + "-";
	auto put = [&](const std::string& name, const std::vector<double>& values) {
		putDataset(storage, prefix + name, values);
	};
	put("success", { result.success ? 1.0 : 0.0 });
	put("point-index", { (double)pointIndex });
	put("seconds-since-start", { t0 - m_seriesStartSeconds });
	put("duration-seconds", { steadySeconds() - t0 });
	put("grid-offset-um", { m_gridOffset.x, m_gridOffset.y });
	put("total-columns-failed-columns", { (double)result.totalColumns, (double)result.failedColumns });
	put("points-without-new-surface", { (double)failed });
	put("prior-used", { hadPrior ? 1.0 : 0.0 });
	put("prior-offset-um", { m_surfacePriorOffsetUm });
	put("prior-offset-columns", { (double)m_surfacePriorOffsetCount });
	put("reference-threshold", { m_surfaceReferenceThreshold });
	put("medium-reference-value", { m_settings.mediumReferenceValue });
	put("prescan-x-um", m_surfacePreScanXUm);
	put("prescan-y-um", m_surfacePreScanYUm);
	put("prescan-found-mask", m_surfacePreScanFoundMask);
	put("prescan-z-um", m_surfacePreScanZUm);
	put("prescan-metric", m_surfacePreScanMetric);
	put("curve-z-um", m_surfaceCurveZUm);
	put("curve-metric", m_surfaceCurveMetric);
	put("curve-sample-counts", m_surfaceCurveSampleCounts);
	put("curve-max-samples", { (double)m_surfaceCurveMaxSamples });
	std::vector<double> boundaryX, boundaryY, boundaryZ;
	for (const auto& point : m_surfaceBoundaryPointsUm) {
		boundaryX.push_back(point.x);
		boundaryY.push_back(point.y);
		boundaryZ.push_back(point.z);
	}
	put("boundary-x-um", boundaryX);
	put("boundary-y-um", boundaryY);
	put("boundary-z-um", boundaryZ);
	auto foundMask = std::vector<double>((size_t)m_settings.xSteps * (size_t)m_settings.ySteps, 0.0);
	for (const auto& xy : m_surfaceFoundXYIndices) {
		const auto index = (size_t)xy.first * (size_t)m_settings.ySteps + (size_t)xy.second;
		if (index < foundMask.size()) {
			foundMask[index] = m_surfaceInterpolatedXYIndices.count(xy) > 0 ? 2.0 : 1.0;
		}
	}
	put("found-mask", foundMask);
	// the z of every planned point in the frame sampled-z is saved in
	const auto sampledOrigin = resolvedGridOriginUm();
	std::vector<double> sampledZ(m_orderedPositions.size());
	for (size_t i = 0; i < m_orderedPositions.size(); i++) {
		sampledZ[i] = m_settings.gridCoordinatesAbsolute ? m_orderedPositions[i].z - sampledOrigin.z : m_orderedPositions[i].z;
	}
	put("sampled-z", sampledZ);
	// the plan in the file follows the new surface (a continue starts from it)
	writeResumePlan(storage, false);
	appendLiveLog("rescan," + QDateTime::currentDateTime().toString(Qt::ISODate).toStdString() + "," + std::to_string(round) + ",after point," + std::to_string((long long)pointIndex)
		+ ",success " + std::to_string(result.success ? 1 : 0) + " columns without surface " + std::to_string(result.failedColumns) + " of " + std::to_string(result.totalColumns)
		+ " took " + std::to_string((int)std::lround(steadySeconds() - t0)) + "s");
	qInfo(logInfo()) << "Surface scan repeated after point" << (long long)pointIndex << ": success" << result.success << ", columns without a surface"
		<< result.failedColumns << "of" << result.totalColumns << ", took" << (steadySeconds() - t0) << "s";
}

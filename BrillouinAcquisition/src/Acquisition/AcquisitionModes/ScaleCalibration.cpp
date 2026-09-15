#include "stdafx.h"
#include "ScaleCalibration.h"

#include "src/helper/h5_helper.h"

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"

#include "filesystem"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>
#include <type_traits>

using namespace std::filesystem;

/*
 * Public definitions
 */

ScaleCalibration::ScaleCalibration(QObject* parent, Acquisition* acquisition, Camera*& camera, ScanControl*& scanControl)
	: AcquisitionMode(parent, acquisition, scanControl), m_camera(camera) {
}

ScaleCalibration::~ScaleCalibration() {}

/*
 * Public slots
 */

void ScaleCalibration::startRepetitions() {
	// Reset up front, regardless of which path below this call takes - startScaleCalibrationCycle()
	// reads this right after every call to decide whether to average this cycle's result in, and
	// a stale "true" left over from a previous, unrelated successful call must never leak into a
	// call that (for whatever reason) didn't actually run/complete a measurement this time.
	m_lastAcquireSucceeded = false;

	bool allowed = m_acquisition->enableMode(ACQUISITION_MODE::SCALECALIBRATION);
	if (!allowed) {
		return;
	}

	// reset abort flag
	m_abort = false;

	// Check that we have camera and scancontrol
	if (!m_camera) {
		return;
	}
	if (!m_scanControl) {
		return;
	}

	configureCalibrationCameraRoi();

	acquire();

	m_acquisition->disableMode(ACQUISITION_MODE::SCALECALIBRATION);

}

void ScaleCalibration::startScaleCalibrationCycle(int cycles) {
	if (cycles < 1 || !m_camera || !m_scanControl) {
		return;
	}

	auto samples = std::vector<ScaleCalibrationData>{};
	for (int cycle = 1; cycle <= cycles; cycle++) {
		emit(s_scaleCalibrationCycleProgress(cycle, cycles));
		// Fully autonomous - unlike the FOV-offset automated cycle, no objective switch/operator
		// refocus is needed between repetitions, so this just runs the existing single-shot
		// procedure (three small stage moves + captures + template match, see __acquire()) back
		// to back, on this same call/thread.
		startRepetitions();
		if (m_lastAcquireSucceeded) {
			samples.push_back(static_cast<ScaleCalibrationData>(m_scaleCalibration));
		}
		if (m_abort) {
			break;
		}
	}
	emit(s_scaleCalibrationCycleProgress(0, cycles));

	if (samples.empty()) {
		emit(s_scaleCalibrationStatus("Scale calibration cycle failed",
			"No cycle produced a valid scale calibration - please make sure there are distinct structures visible."));
		return;
	}

	// Average the pix->um matrix across every successful cycle (component-wise mean), the same
	// "M cycles -> one averaged result" convention as the automated FOV-offset cycle. Sigma is
	// the std dev of each cycle's isotropic pixel pitch (see
	// ScaleCalibrationHelper::isotropicPixelPitchUm()) around that mean - a single, rotation-
	// independent repeatability number, not one component out of the four in the matrix.
	auto meanPixToMicrometerX = POINT2{ 0, 0 };
	auto meanPixToMicrometerY = POINT2{ 0, 0 };
	auto pitchesUm = std::vector<double>{};
	for (const auto& sample : samples) {
		meanPixToMicrometerX.x += sample.pixToMicrometerX.x;
		meanPixToMicrometerX.y += sample.pixToMicrometerX.y;
		meanPixToMicrometerY.x += sample.pixToMicrometerY.x;
		meanPixToMicrometerY.y += sample.pixToMicrometerY.y;
		pitchesUm.push_back(ScaleCalibrationHelper::isotropicPixelPitchUm(sample));
	}
	meanPixToMicrometerX.x /= samples.size();
	meanPixToMicrometerX.y /= samples.size();
	meanPixToMicrometerY.x /= samples.size();
	meanPixToMicrometerY.y /= samples.size();

	auto meanPitchUm = 0.0;
	for (auto pitchUm : pitchesUm) {
		meanPitchUm += pitchUm;
	}
	meanPitchUm /= pitchesUm.size();
	auto sigma = 0.0;
	if (pitchesUm.size() > 1) {
		auto sumSq = 0.0;
		for (auto pitchUm : pitchesUm) {
			auto d = pitchUm - meanPitchUm;
			sumSq += d * d;
		}
		sigma = std::sqrt(sumSq / (pitchesUm.size() - 1));
	}

	m_scaleCalibration.pixToMicrometerX = meanPixToMicrometerX;
	m_scaleCalibration.pixToMicrometerY = meanPixToMicrometerY;
	try {
		// Derives the (redundant, internal-only) micrometerToPix direction from the averaged
		// pixToMicrometer and validates it is still a real basis.
		ScaleCalibrationHelper::initializeCalibrationFromPixel(&m_scaleCalibration);
	} catch (std::exception&) {
	}
	m_scaleCalibration.scaleCalibrationSigmaUm = sigma;

	emit(s_scaleCalibrationChanged(m_scaleCalibration));
	emit(s_objectiveCalibrationChanged(m_scaleCalibration));

	auto message = "Averaged " + std::to_string(samples.size()) + " of " + std::to_string(cycles)
		+ " requested cycle(s). Pixel pitch " + std::to_string(meanPitchUm) + " um/pix, sigma "
		+ std::to_string(sigma) + " um/pix. \"Save calibration (no acquire needed)\" to persist.";
	emit(s_scaleCalibrationStatus("Scale calibration cycle finished", message));
}

void ScaleCalibration::loadCalibrationForSlot(int slot, std::string filepath, std::string objectiveName, double magnification) {
	if (!m_scanControl) {
		return;
	}
	if (!m_scanControl->isValidObjectiveSlot(slot)) {
		emit(s_scaleCalibrationStatus("Could not link calibration file",
			"Slot " + std::to_string(slot) + " is not a physically-possible objective position on this device."));
		return;
	}

	ObjectiveCalibrationData data{};
	try {
		readCalibrationFile(filepath, &data);
		// A freshly-created ("New") file legitimately has an all-zero scale calibration until a
		// real "Acquire" has been run and applied for it - that is not the same as the file
		// being corrupt/invalid, so only recompute (and validate) micrometerToPix when
		// pixToMicrometer actually forms a basis; otherwise leave the all-zero matrices as read,
		// exactly like createEmptyCalibrationFile() itself never runs
		// initializeCalibrationFromPixel() either. Linking such a file just means "no scale
		// calibration yet for this slot", not a rejected file.
		if (ScaleCalibrationHelper::isBasis(data.pixToMicrometerX, data.pixToMicrometerY)) {
			ScaleCalibrationHelper::initializeCalibrationFromPixel(&data);
		}
	} catch (...) {
		emit(s_scaleCalibrationStatus("Could not link calibration file",
			"\"" + filepath + "\" is not a valid scale calibration file - only files written by "
			"this version of the software (Save calibration, or Objective Setup's \"New\" button) "
			"are accepted."));
		return;
	}

	// Objective Setup's own name/magnification for this slot is authoritative - overwrite
	// whatever the file itself says (if anything), rather than trusting a name that may be
	// stale, absent (legacy file), or simply never matched this slot's current name. This is
	// also what "extends" a legacy file with no objective fields at all.
	data.objectiveName = objectiveName;
	data.magnification = magnification;
	data.objectiveSlot = slot;

	// Explicit link - register against the requested slot regardless of what the file's own
	// objectiveSlot said. setObjectiveCalibration() applies it live immediately if slot is
	// already active, and unconditionally on every future switch to it either way
	// (ScanControl::handleObjectiveSlotObserved()).
	m_scanControl->setObjectiveCalibration(slot, data);
}

void ScaleCalibration::createEmptyCalibrationFile(int slot, std::string objectiveName, double magnification, std::string filepath) {
	if (!m_scanControl) {
		return;
	}
	if (!m_scanControl->isValidObjectiveSlot(slot)) {
		emit(s_scaleCalibrationStatus("Could not create calibration file",
			"Slot " + std::to_string(slot) + " is not a physically-possible objective position on this device."));
		return;
	}

	// Deliberately not run through ScaleCalibrationHelper::initializeCalibrationFromPixel() -
	// an all-zero scale calibration is not a valid basis and would throw. This is a genuinely
	// blank placeholder (no scale calibration, no FOV offset yet), the same state a freshly-
	// constructed ObjectiveCalibrationData{} already represents elsewhere in this codebase -
	// only registered directly, not validated as if it were a real measured calibration.
	ObjectiveCalibrationData data{};
	data.objectiveName = objectiveName;
	data.magnification = magnification;
	data.objectiveSlot = slot;
	data.calibrationDate = QDateTime::currentDateTime().toOffsetFromUtc(QDateTime::currentDateTime().offsetFromUtc())
		.toString(Qt::ISODateWithMs).toStdString();

	try {
		auto file = H5::H5File(&filepath[0], H5F_ACC_TRUNC);
		auto root = file.openGroup("/");
		writeAttribute(root, "date", data.calibrationDate);
		writePoint(root, "origin", data.originPix);
		writePoint(root, "pixToMicrometerX", data.pixToMicrometerX);
		writePoint(root, "pixToMicrometerY", data.pixToMicrometerY);
		writePoint(root, "micrometerToPixX", data.micrometerToPixX);
		writePoint(root, "micrometerToPixY", data.micrometerToPixY);
		writeAttribute(root, "objectiveName", data.objectiveName);
		writeAttribute(root, "magnification", data.magnification);
		writeAttribute(root, "referenceObjectiveName", data.referenceObjectiveName);
		writeAttribute(root, "calibrationDate", data.calibrationDate);
		writeAttribute(root, "hasFovOffset", data.hasFovOffset ? 1.0 : 0.0);
		writeAttribute(root, "fovOffsetX", data.fovOffsetUm.x);
		writeAttribute(root, "fovOffsetY", data.fovOffsetUm.y);
		writeAttribute(root, "fovOffsetSigma", data.fovOffsetSigmaUm);
		writeAttribute(root, "scaleCalibrationSigma", data.scaleCalibrationSigmaUm);
		writeAttribute(root, "objectiveSlot", (double)data.objectiveSlot);
		writeAttribute(root, "isReferenceObjective", data.isReferenceObjective ? 1.0 : 0.0);
	} catch (H5::Exception&) {
		emit(s_scaleCalibrationStatus("Could not create calibration file", "\"" + filepath + "\" is not writable."));
		return;
	}

	m_scanControl->setObjectiveCalibration(slot, data);
}

void ScaleCalibration::writeCalibrationToSlot(int slot, std::string filepath, ObjectiveCalibrationData data) {
	if (!m_scanControl) {
		return;
	}
	if (!m_scanControl->isValidObjectiveSlot(slot)) {
		emit(s_scaleCalibrationStatus("Could not update calibration",
			"Slot " + std::to_string(slot) + " is not a physically-possible objective position on this device."));
		return;
	}

	data.objectiveSlot = slot;
	data.calibrationDate = QDateTime::currentDateTime().toOffsetFromUtc(QDateTime::currentDateTime().offsetFromUtc())
		.toString(Qt::ISODateWithMs).toStdString();

	if (!filepath.empty()) {
		try {
			auto file = H5::H5File(&filepath[0], H5F_ACC_TRUNC);
			auto root = file.openGroup("/");
			writeAttribute(root, "date", data.calibrationDate);
			writePoint(root, "origin", data.originPix);
			writePoint(root, "pixToMicrometerX", data.pixToMicrometerX);
			writePoint(root, "pixToMicrometerY", data.pixToMicrometerY);
			writePoint(root, "micrometerToPixX", data.micrometerToPixX);
			writePoint(root, "micrometerToPixY", data.micrometerToPixY);
			writeAttribute(root, "objectiveName", data.objectiveName);
			writeAttribute(root, "magnification", data.magnification);
			writeAttribute(root, "referenceObjectiveName", data.referenceObjectiveName);
			writeAttribute(root, "calibrationDate", data.calibrationDate);
			writeAttribute(root, "hasFovOffset", data.hasFovOffset ? 1.0 : 0.0);
			writeAttribute(root, "fovOffsetX", data.fovOffsetUm.x);
			writeAttribute(root, "fovOffsetY", data.fovOffsetUm.y);
			writeAttribute(root, "fovOffsetSigma", data.fovOffsetSigmaUm);
			writeAttribute(root, "scaleCalibrationSigma", data.scaleCalibrationSigmaUm);
			writeAttribute(root, "objectiveSlot", (double)data.objectiveSlot);
			writeAttribute(root, "isReferenceObjective", data.isReferenceObjective ? 1.0 : 0.0);
		} catch (H5::Exception&) {
			emit(s_scaleCalibrationStatus("Could not save calibration", "\"" + filepath + "\" is not writable."));
			return;
		}
	}

	m_scanControl->setObjectiveCalibration(slot, data);
	// If this happens to be the active slot (and/or its edit buffer is currently showing on the
	// dialog), keep both in sync exactly like persistPartial() does.
	if (slot == m_scanControl->getActiveObjectiveSlot()) {
		m_scaleCalibration = data;
		emit(s_scaleCalibrationChanged(m_scaleCalibration));
		emit(s_objectiveCalibrationChanged(m_scaleCalibration));
	}
}

/*
 * Private definitions
 */

void ScaleCalibration::abortMode(std::unique_ptr <StorageWrapper>& storage) {}

void ScaleCalibration::abortMode() {
	m_acquisition->disableMode(ACQUISITION_MODE::SCALECALIBRATION);

	m_scanControl->setPosition(m_startPosition);

	QMetaObject::invokeMethod(
		m_scanControl,
		[scanControl = m_scanControl]() { scanControl->startAnnouncing(); },
		Qt::AutoConnection
	);

	setAcquisitionStatus(ACQUISITION_STATUS::ABORTED);
}

std::string ScaleCalibration::newCalibrationFilePath() const {
	auto folder = m_acquisition->getCurrentFolder();

	auto shortdate = QDateTime::currentDateTime().toOffsetFromUtc(QDateTime::currentDateTime().offsetFromUtc())
		.toString("yyyy-MM-ddTHHmmss").toStdString();
	auto dt1 = QDateTime::currentDateTime();
	auto dt2 = dt1.toUTC();
	dt1.setTimeSpec(Qt::UTC);

	auto offset = dt2.secsTo(dt1) / 3600;

	auto offse = QString("+%1").arg(offset, 2, 10, QChar('0')).toStdString();

	auto filepath{ folder + "/_scaleCalibration_" };
	filepath += shortdate + offse + ".h5";
	return filepath;
}

void ScaleCalibration::readCalibrationFile(const std::string& filepath, ObjectiveCalibrationData* out) {
	auto file = H5::H5File(&filepath[0], H5F_ACC_RDONLY);
	auto root = file.openGroup("/");

	out->originPix = readPoint(root, "origin");

	out->pixToMicrometerX = readPoint(root, "pixToMicrometerX");
	out->pixToMicrometerY = readPoint(root, "pixToMicrometerY");

	out->micrometerToPixX = readPoint(root, "micrometerToPixX");
	out->micrometerToPixY = readPoint(root, "micrometerToPixY");

	// Objective-identity/FOV-offset/slot fields - required now (throws if any is missing, e.g.
	// an old legacy file saved before they existed). See this function's header doc comment.
	readAttribute(root, "objectiveName", &out->objectiveName);
	readAttribute(root, "magnification", &out->magnification);
	readAttribute(root, "referenceObjectiveName", &out->referenceObjectiveName);
	readAttribute(root, "calibrationDate", &out->calibrationDate);
	auto hasFovOffsetValue = 0.0;
	readAttribute(root, "hasFovOffset", &hasFovOffsetValue);
	out->hasFovOffset = hasFovOffsetValue != 0.0;
	readAttribute(root, "fovOffsetX", &out->fovOffsetUm.x);
	readAttribute(root, "fovOffsetY", &out->fovOffsetUm.y);
	readAttribute(root, "fovOffsetSigma", &out->fovOffsetSigmaUm);
	// Added after the fields above - kept optional (existence-checked) rather than required, so
	// calibration files created just before this field existed don't also need recreating: a
	// missing sigma is non-critical QC metadata, not something that can silently misapply a
	// calibration the way a missing objectiveName/hasFovOffset could.
	out->scaleCalibrationSigmaUm = 0.0;
	if (root.attrExists("scaleCalibrationSigma")) {
		readAttribute(root, "scaleCalibrationSigma", &out->scaleCalibrationSigmaUm);
	}
	auto objectiveSlotValue = 0.0;
	readAttribute(root, "objectiveSlot", &objectiveSlotValue);
	out->objectiveSlot = (int)objectiveSlotValue;
	// Added after the fields above - same optional/existence-checked treatment as
	// scaleCalibrationSigma, for the same reason (an older file predating this field is not a
	// reference objective, not an invalid file).
	out->isReferenceObjective = false;
	if (root.attrExists("isReferenceObjective")) {
		auto isReferenceObjectiveValue = 0.0;
		readAttribute(root, "isReferenceObjective", &isReferenceObjectiveValue);
		out->isReferenceObjective = isReferenceObjectiveValue != 0.0;
	}
}

void ScaleCalibration::writeCalibrationMetadata(H5::Group& root) {
	// Record which slot this calibration is being saved from - both callers (save(), the
	// acquire-based procedure, and writeLinkedCalibrationFile(), used by persistPartial())
	// already assume the operator is standing at the objective being calibrated, so the
	// currently active slot is exactly the right value to persist (informational -
	// loadCalibrationForSlot() overwrites it with whatever slot the file is explicitly linked
	// to, regardless of what was recorded here at save time).
	m_scaleCalibration.objectiveSlot = m_scanControl->getActiveObjectiveSlot();

	auto fulldate = QDateTime::currentDateTime().toOffsetFromUtc(QDateTime::currentDateTime().offsetFromUtc())
		.toString(Qt::ISODateWithMs).toStdString();
	writeAttribute(root, "date", fulldate);

	writePoint(root, "origin", m_scaleCalibration.originPix);

	writePoint(root, "pixToMicrometerX", m_scaleCalibration.pixToMicrometerX);
	writePoint(root, "pixToMicrometerY", m_scaleCalibration.pixToMicrometerY);

	writePoint(root, "micrometerToPixX", m_scaleCalibration.micrometerToPixX);
	writePoint(root, "micrometerToPixY", m_scaleCalibration.micrometerToPixY);

	writeAttribute(root, "objectiveName", m_scaleCalibration.objectiveName);
	writeAttribute(root, "magnification", m_scaleCalibration.magnification);
	writeAttribute(root, "referenceObjectiveName", m_scaleCalibration.referenceObjectiveName);
	writeAttribute(root, "calibrationDate", m_scaleCalibration.calibrationDate);
	writeAttribute(root, "hasFovOffset", m_scaleCalibration.hasFovOffset ? 1.0 : 0.0);
	writeAttribute(root, "fovOffsetX", m_scaleCalibration.fovOffsetUm.x);
	writeAttribute(root, "fovOffsetY", m_scaleCalibration.fovOffsetUm.y);
	writeAttribute(root, "fovOffsetSigma", m_scaleCalibration.fovOffsetSigmaUm);
	writeAttribute(root, "scaleCalibrationSigma", m_scaleCalibration.scaleCalibrationSigmaUm);
	writeAttribute(root, "objectiveSlot", (double)m_scaleCalibration.objectiveSlot);
	writeAttribute(root, "isReferenceObjective", m_scaleCalibration.isReferenceObjective ? 1.0 : 0.0);
}

void ScaleCalibration::setLinkedCalibrationFilePath(std::string path) {
	m_linkedCalibrationFilePath = path;
}

void ScaleCalibration::writeLinkedCalibrationFile() {
	if (m_linkedCalibrationFilePath.empty()) {
		emit(s_scaleCalibrationStatus("Calibration not saved to a file",
			"This slot has no linked calibration file yet - link or create one first "
			"(Devices > Objective Setup), then Save again to persist this calibration to disk."));
		return;
	}
	try {
		auto file = H5::H5File(&m_linkedCalibrationFilePath[0], H5F_ACC_TRUNC);
		auto root = file.openGroup("/");
		writeCalibrationMetadata(root);
	} catch (H5::Exception& exception) {
		emit(s_scaleCalibrationStatus("Could not save the scale calibration",
			"\"" + m_linkedCalibrationFilePath + "\" is not writable."));
	}
}

void ScaleCalibration::persistPartial(bool includeScale, bool includeFov) {
	if (!m_scanControl) {
		return;
	}
	auto slot = m_scanControl->getActiveObjectiveSlot();
	// Start from whatever is currently stored for this slot (already registered in ScanControl,
	// i.e. whatever the linked file last held) - not from m_scaleCalibration wholesale - so the
	// half the caller does NOT own is left exactly as it was, regardless of what might currently
	// be sitting, not-yet-committed, in that half of the dialog's shared edit buffer.
	auto data = m_scanControl->getObjectiveCalibration(slot);
	// Snapshot of the FOV-offset half exactly as it was BEFORE this save overwrites it below -
	// only meaningful/used when includeFov (see the s_fovOffsetSaved emit at the end), kept
	// outside that condition only because `data` itself is about to be overwritten either way.
	auto oldHasFovOffset = data.hasFovOffset;
	auto oldFovOffsetUm = data.fovOffsetUm;
	// Identity fields are not "owned" by either half - refreshScaleCalibrationObjectiveDisplay()
	// always keeps these correct on m_scaleCalibration, so always safe (and necessary, since a
	// blank "New" file's stored copy may still be empty) to carry them over.
	data.objectiveName = m_scaleCalibration.objectiveName;
	data.magnification = m_scaleCalibration.magnification;
	if (includeScale) {
		static_cast<ScaleCalibrationData&>(data) = static_cast<ScaleCalibrationData&>(m_scaleCalibration);
		data.scaleCalibrationSigmaUm = m_scaleCalibration.scaleCalibrationSigmaUm;
	}
	if (includeFov) {
		data.hasFovOffset = m_scaleCalibration.hasFovOffset;
		data.fovOffsetUm = m_scaleCalibration.fovOffsetUm;
		data.fovOffsetSigmaUm = m_scaleCalibration.fovOffsetSigmaUm;
		data.referenceObjectiveName = m_scaleCalibration.referenceObjectiveName;
	}

	m_scanControl->setObjectiveCalibration(slot, data);
	// Re-read the merged, actually-saved result back into the edit buffer (rather than leaving
	// whatever was there before) and re-emit the change signals, so the dialog visually reverts
	// any abandoned edit in the half that was NOT included - it did not get saved, so it should
	// not keep appearing to be.
	m_scaleCalibration = data;
	writeLinkedCalibrationFile();
	emit(s_scaleCalibrationChanged(m_scaleCalibration));
	emit(s_objectiveCalibrationChanged(m_scaleCalibration));
	if (includeFov) {
		// Unlike an objective switch (ScanControl::s_objectiveSwitched), nothing else forces the
		// on-screen grid to recompute after this - s_scaleCalibrationChanged/
		// s_objectiveCalibrationChanged above only refresh the dialog's own display fields and
		// re-project the already-cached grid pixel positions, they never rebuild them. Carries
		// the pre-save/post-save FOV-offset state (slot is always the active one - this dialog
		// only ever edits/saves against whichever objective is currently active) so a listener
		// can apply the same "shift the not-yet-visited relative-mode grid by the delta" an
		// objective switch gets, and force an absolute-mode redraw - see
		// BrillouinAcquisition::onFovOffsetSaved().
		emit(s_fovOffsetSaved(slot, oldFovOffsetUm, oldHasFovOffset, data.fovOffsetUm, data.hasFovOffset));
	}
}

void ScaleCalibration::saveScaleCalibration() {
	try {
		// Keeps micrometerToPix in sync with a possibly-just-edited pixToMicrometer field - the
		// same validation the former apply() performed.
		ScaleCalibrationHelper::initializeCalibrationFromPixel(&m_scaleCalibration);
	} catch (std::exception&) {
	}
	persistPartial(true, false);
}

void ScaleCalibration::saveFovOffsetCalibration() {
	// No scale-calibration validation here at all - this half never touches those fields, so a
	// still-degenerate (not yet "Acquire"d) scale calibration must not block saving just an
	// FOV-center offset.
	persistPartial(false, true);
}

template <typename T>
void ScaleCalibration::save(std::vector<std::vector<T>> images, std::vector<POINT2> positions) {
	auto filepath = newCalibrationFilePath();

	try {
		/*
		 * Open file for writing, overwrite existing file
		 */
		auto file = H5::H5File(&filepath[0], H5F_ACC_TRUNC);
		auto root = file.openGroup("/");
		writeCalibrationMetadata(root);

		// Write images and set positions as attributes
		auto imageGroup = root.createGroup("images");
		hsize_t dims[3] = {
			(hsize_t)m_cameraSettings.frameCount,
			(hsize_t)m_cameraSettings.roi.height_binned,
			(hsize_t)m_cameraSettings.roi.width_binned
		};
		auto names = std::vector<std::string>{ "origin", "dx", "dy" };
		auto i = gsl::index{ 0 };
		for (const auto& image : images) {
			// Check that we don't run into trouble iterating over two arrays
			if (i >= names.size()) {
				continue;
			}
			auto dataspace = H5::DataSpace(3, dims, dims);
			auto type_id = h5_helper::get_memtype<T>();
			// For some unknown reason using the overloaded function createDataSet(const H5std_string&, const DataType&, const DataSpace&)
			// only produces corrupted datasets in Debug mode (probably the HDF5 libraries would need to be build in Debug mode as well),
			// so we have to use the const char* version to better debug it.
			auto dataset = imageGroup.createDataSet(names[i].c_str(), type_id, dataspace);
			dataset.write(image.data(), type_id);

			writeAttribute(dataset, "CLASS", "IMAGE");
			writeAttribute(dataset, "IMAGE_VERSION", "1.2");
			writeAttribute(dataset, "IMAGE_SUBCLASS", "IMAGE_GRAYSCALE");

			writeAttribute(dataset, "dx", positions[i].x);
			writeAttribute(dataset, "dy", positions[i].y);

			dataset.close();
			++i;
		}
	} catch (H5::Exception& exception) {
		emit(s_scaleCalibrationStatus("Could not save the scale calibration", "Please select a writable working directory."));
	}
}

void ScaleCalibration::writePoint(H5::Group group, std::string name, POINT2 point) {
	using namespace H5;

	auto subgroup = H5::Group(group.createGroup(&name[0]));

	writeAttribute(subgroup, "x", point.x);
	writeAttribute(subgroup, "y", point.y);

	subgroup.close();
}

POINT2 ScaleCalibration::readPoint(H5::Group group, const std::string& name) {
	auto point = POINT2{};

	// Open point group
	try {
		auto pointGroup = group.openGroup(&name[0]);

		// Read x coordinate
		readAttribute(pointGroup, "x", &point.x);
		// Read y coordinate
		readAttribute(pointGroup, "y", &point.y);

	} catch (H5::GroupIException& exception) {
		throw;
	}

	return point;
}

void ScaleCalibration::writeAttribute(H5::H5Object& parent, std::string name, double value) {
	auto attr_dataspace = H5::DataSpace(H5S_SCALAR);
	// Create new string datatype for attribute
	auto strdatatype = H5::PredType::NATIVE_DOUBLE;
	auto attr = parent.createAttribute(name.c_str(), strdatatype, attr_dataspace);
	attr.write(strdatatype, &value);
	attr.close();
}

void ScaleCalibration::writeAttribute(H5::H5Object& parent, std::string name, std::string value) {
	auto attr_dataspace = H5::DataSpace(H5S_SCALAR);
	// Create new string datatype for attribute. Previously only ever called with a
	// guaranteed non-empty date string; the new objective-identity fields (objectiveName,
	// referenceObjectiveName, calibrationDate) can legitimately still be "" (never typed in),
	// and a zero-length H5::StrType is not something to rely on behaving well, so clamp to at
	// least 1 byte.
	auto strdatatype = H5::StrType(H5::PredType::C_S1, std::max<size_t>(1, value.size()));
	auto attr = parent.createAttribute(name.c_str(), strdatatype, attr_dataspace);
	attr.write(strdatatype, value.c_str());
	attr.close();
}

void ScaleCalibration::readAttribute(const H5::H5Object& parent, std::string name, double* value) {
	auto attr = parent.openAttribute(name.c_str());
	auto type = attr.getDataType();
	// If this is not a double, return NaN
	if (type != H5::PredType::NATIVE_DOUBLE) {
		return;
	}
	attr.read(type, value);
	type.close();
	attr.close();
}

void ScaleCalibration::readAttribute(const H5::H5Object& parent, std::string name, std::string* value) {
	// openAttribute() throws (H5::AttributeIException) if the attribute is missing -
	// readCalibrationFile() relies on this to reject a file missing any "new"-format field
	// rather than silently defaulting it.
	auto attr = parent.openAttribute(name.c_str());
	auto type = attr.getDataType();
	auto size = type.getSize();
	auto buffer = std::string(size, '\0');
	if (size > 0) {
		attr.read(type, &buffer[0]);
	}
	*value = buffer;
	type.close();
	attr.close();
}

template <typename T>
void ScaleCalibration::__acquire() {
	setAcquisitionStatus(ACQUISITION_STATUS::STARTED);

	QMetaObject::invokeMethod(
		m_scanControl,
		[scanControl = m_scanControl]() { scanControl->stopAnnouncing(); },
		Qt::AutoConnection
	);
	// Set optical elements for brightfield/Brillouin imaging
	m_scanControl->setPreset(ScanPreset::SCAN_BRIGHTFIELD);
	std::this_thread::sleep_for(std::chrono::milliseconds(500));

	// Get the current stage position
	m_startPosition = m_scanControl->getPosition();

	/*
	 * We acquire three images here, one at the origin, and one each shifted in x- and y-direction.
	 */
	auto hysteresisCompensation{ 10.0 };// [µm] distance for compensation of the stage hysteresis
	// Construct the positions
	auto positions = std::vector<POINT2>{ { 0, 0 }, { m_Ds.x, 0 }, { 0, m_Ds.y } };

	// Acquire memory for image acquisition
	auto images = std::vector<std::vector<std::byte>>(positions.size());
	for (auto& image : images) {
		image.resize(m_cameraSettings.roi.bytesPerFrame);
	}

	m_camera->startAcquisition(m_cameraSettings);

	auto iteration{ 0 };
	emit(s_scaleCalibrationAcquisitionProgress(iteration));
	for (const auto& position : positions) {
		// Abort if requested
		if (m_abort) {
			this->abortMode();
			return;
		}

		/*
		 * Set the new stage position
		 */
		auto positionAbsolute = m_startPosition + POINT3{ position.x, position.y, 0 };
		// To prevent problems with the hysteresis of the stage, we always move to the desired point coming from lower values.
		m_scanControl->setPosition(positionAbsolute - POINT3{ hysteresisCompensation, hysteresisCompensation, 0 });
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		m_scanControl->setPosition(positionAbsolute);
		std::this_thread::sleep_for(std::chrono::milliseconds(200));

		/*
		 * Acquire the camera image
		 */

		 // acquire images
		m_camera->getImageForAcquisition(&(images[iteration])[0], false);

		// Sometimes the uEye camera returns a black image (only zeros), we try to catch this here by
		// repeating the acquisition a maximum of 5 times
		// cast the vector type T
		auto images_ = (std::vector<T> *) &(images[iteration]);

		auto sum = simplemath::sum(*images_);
		auto tryCount{ 0 };
		while (sum == 0 && 5 > tryCount++) {
			m_camera->getImageForAcquisition(&(images[iteration])[0], false);

			images_ = (std::vector<T> *) &(images[iteration]);
			sum = simplemath::sum(*images_);
		}

		++iteration;
		emit(s_scaleCalibrationAcquisitionProgress(iteration * 100.0 / images.size()));
	}

	// Stop the camera acquisition
	m_camera->stopAcquisition();

	// Create input matrices for OpenCV from camera images. Previously always wrapped as
	// CV_8UC1 regardless of the camera's actual readout type - correct only for 8-bit; wrong
	// for PointGrey (see PointGrey.cpp) set to Mono12/Mono16, which arrives here as
	// T = unsigned short. readAsMat8U() (also used by the FOV-offset measurement - see
	// computeFovOffsetShiftUm()) wraps the buffer at its real depth and downscales 16-bit
	// sources to 8-bit for the matching pipeline below, which has only ever run against
	// 8-bit data.
	auto imagesCV = std::vector<cv::Mat>(images.size());
	for (gsl::index i{ 0 }; i < images.size(); i++) {
		imagesCV[i] = readAsMat8U(images[i], m_cameraSettings.roi.height_binned, m_cameraSettings.roi.width_binned, m_cameraSettings.readout.dataType);
	}

	/*
	 * Determine the shift in pixels
	 */

#ifdef _DEBUG
	 // Show the input images for debugging
	cv::imshow("Origin", imagesCV[0]);
	cv::imshow("Dx", imagesCV[1]);
	cv::imshow("Dy", imagesCV[2]);
#endif

	// We use template matching, so we have to create a template from the origin image
	auto padding = 200.0;
	auto size = imagesCV[0].size();
	auto templateROI = cv::Rect(padding, padding, size.width - 2 * padding, size.height - 2 * padding);
	auto templ = imagesCV[0](templateROI);

#ifdef _DEBUG
	cv::imshow("Template", templ);
#endif

	auto outputX = cv::Mat{};
	cv::matchTemplate(imagesCV[1], templ, outputX, cv::TemplateMatchModes::TM_SQDIFF);
	auto outputY = cv::Mat{};
	cv::matchTemplate(imagesCV[2], templ, outputY, cv::TemplateMatchModes::TM_SQDIFF);

	cv::normalize(outputX, outputX, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());
	cv::normalize(outputY, outputY, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());

	auto minValX = double{};
	auto maxValX = double{};
	auto minLocX = cv::Point{};
	auto maxLocX = cv::Point{};
	cv::minMaxLoc(outputX, &minValX, &maxValX, &minLocX, &maxLocX, cv::Mat());
	auto shiftDx = minLocX - cv::Point(padding, padding);

	auto minValY = double{};
	auto maxValY = double{};
	auto minLocY = cv::Point{};
	auto maxLocY = cv::Point{};
	cv::minMaxLoc(outputY, &minValY, &maxValY, &minLocY, &maxLocY, cv::Mat());
	auto shiftDy = minLocY - cv::Point(padding, padding);

	/*
	 * Construct the scale calibration
	 */
	m_scaleCalibration.micrometerToPixX = { -1.0 * shiftDx.x / m_Ds.x, shiftDx.y / m_Ds.x };
	m_scaleCalibration.micrometerToPixY = { -1.0 * shiftDy.x / m_Ds.y, shiftDy.y / m_Ds.y };
	try {
		// Can throw an exception (if the provided calibration is invalid):
		ScaleCalibrationHelper::initializeCalibrationFromMicrometer(&m_scaleCalibration);

		// Store the calibration in a file

		auto images_ = (std::vector<std::vector<T>> *) &images;
		save((*images_), positions);

		// Read by startScaleCalibrationCycle() right after this call returns, to decide whether
		// this cycle's m_scaleCalibration is safe to average in.
		m_lastAcquireSucceeded = true;

		emit(s_scaleCalibrationChanged(m_scaleCalibration));

	} catch (std::exception& e) {
		emit(s_scaleCalibrationStatus("Scale calibration failed", "Please make sure there are distinct structures visible."));
	}

	/*
	 * Cleanup the acquisition mode
	 */
	m_scanControl->setPosition(m_startPosition);
	QMetaObject::invokeMethod(
		m_scanControl,
		[scanControl = m_scanControl]() { scanControl->startAnnouncing(); },
		Qt::AutoConnection
	);

	setAcquisitionStatus(ACQUISITION_STATUS::FINISHED);
}

/*
 * Private slots
 */

void ScaleCalibration::acquire(std::unique_ptr <StorageWrapper>& storage) {}

void ScaleCalibration::acquire() {
	if (m_cameraSettings.readout.dataType == "unsigned short") {
		__acquire<unsigned short>();
	} else if (m_cameraSettings.readout.dataType == "unsigned char") {
		__acquire<unsigned char>();
	}
}

void ScaleCalibration::initialize() {
	// Get the current scale calibration from the scanControl. Only the base
	// ScaleCalibrationData portion - m_scaleCalibration is now the derived
	// ObjectiveCalibrationData, and getScaleCalibration() only returns the base type, so a
	// plain assignment no longer compiles. The objective-identity/FOV-offset fields are
	// populated separately just below, from whatever is already registered for the active
	// slot (if any), so the dialog opens showing the right existing values for whichever
	// objective is actually in the beam path right now.
	static_cast<ScaleCalibrationData&>(m_scaleCalibration) = m_scanControl->getScaleCalibration();
	auto activeCalibration = m_scanControl->getActiveObjectiveCalibration();
	m_scaleCalibration.objectiveName = activeCalibration.objectiveName;
	m_scaleCalibration.magnification = activeCalibration.magnification;
	m_scaleCalibration.hasFovOffset = activeCalibration.hasFovOffset;
	m_scaleCalibration.fovOffsetUm = activeCalibration.fovOffsetUm;
	m_scaleCalibration.fovOffsetSigmaUm = activeCalibration.fovOffsetSigmaUm;
	m_scaleCalibration.referenceObjectiveName = activeCalibration.referenceObjectiveName;
	m_scaleCalibration.calibrationDate = activeCalibration.calibrationDate;
	m_scaleCalibration.scaleCalibrationSigmaUm = activeCalibration.scaleCalibrationSigmaUm;

	// Emit it to the main GUI thread
	emit(s_scaleCalibrationAcquisitionProgress(0.0));
	emit(s_scaleCalibrationChanged(m_scaleCalibration));
	emit(s_objectiveCalibrationChanged(m_scaleCalibration));
	emit(s_Ds_changed(m_Ds));
}

void ScaleCalibration::setTranslationDistanceX(double dx) {
	m_Ds.x = dx;
	emit(s_Ds_changed(m_Ds));
}

void ScaleCalibration::setTranslationDistanceY(double dy) {
	m_Ds.y = dy;
	emit(s_Ds_changed(m_Ds));
}

void ScaleCalibration::setMicrometerToPixX_x(double value) {
	m_scaleCalibration.micrometerToPixX.x = value;
	try {
		ScaleCalibrationHelper::initializeCalibrationFromMicrometer(&m_scaleCalibration);
		emit(s_scaleCalibrationChanged(m_scaleCalibration));
	} catch (std::exception& e) {
	}
}

void ScaleCalibration::setMicrometerToPixX_y(double value) {
	m_scaleCalibration.micrometerToPixX.y = value;
	try {
		ScaleCalibrationHelper::initializeCalibrationFromMicrometer(&m_scaleCalibration);
		emit(s_scaleCalibrationChanged(m_scaleCalibration));
	} catch (std::exception& e) {
	}
}

void ScaleCalibration::setMicrometerToPixY_x(double value) {
	m_scaleCalibration.micrometerToPixY.x = value;
	try {
		ScaleCalibrationHelper::initializeCalibrationFromMicrometer(&m_scaleCalibration);
		emit(s_scaleCalibrationChanged(m_scaleCalibration));
	} catch (std::exception& e) {
	}
}

void ScaleCalibration::setMicrometerToPixY_y(double value) {
	m_scaleCalibration.micrometerToPixY.y = value;
	try {
		ScaleCalibrationHelper::initializeCalibrationFromMicrometer(&m_scaleCalibration);
		emit(s_scaleCalibrationChanged(m_scaleCalibration));
	} catch (std::exception& e) {
	}
}

void ScaleCalibration::setPixToMicrometerX_x(double value) {
	m_scaleCalibration.pixToMicrometerX.x = value;
	try {
		ScaleCalibrationHelper::initializeCalibrationFromPixel(&m_scaleCalibration);
		emit(s_scaleCalibrationChanged(m_scaleCalibration));
	} catch (std::exception& e) {
	}
}

void ScaleCalibration::setPixToMicrometerX_y(double value) {
	m_scaleCalibration.pixToMicrometerX.y = value;
	try {
		ScaleCalibrationHelper::initializeCalibrationFromPixel(&m_scaleCalibration);
		emit(s_scaleCalibrationChanged(m_scaleCalibration));
	} catch (std::exception& e) {
	}
}

void ScaleCalibration::setPixToMicrometerY_x(double value) {
	m_scaleCalibration.pixToMicrometerY.x = value;
	try {
		ScaleCalibrationHelper::initializeCalibrationFromPixel(&m_scaleCalibration);
		emit(s_scaleCalibrationChanged(m_scaleCalibration));
	} catch (std::exception& e) {
	}
}

void ScaleCalibration::setPixToMicrometerY_y(double value) {
	m_scaleCalibration.pixToMicrometerY.y = value;
	try {
		ScaleCalibrationHelper::initializeCalibrationFromPixel(&m_scaleCalibration);
		emit(s_scaleCalibrationChanged(m_scaleCalibration));
	} catch (std::exception& e) {
	}
}

void ScaleCalibration::setObjectiveName(QString name) {
	m_scaleCalibration.objectiveName = name.toStdString();
	emit(s_objectiveCalibrationChanged(m_scaleCalibration));
}

void ScaleCalibration::setMagnification(double value) {
	m_scaleCalibration.magnification = value;
	emit(s_objectiveCalibrationChanged(m_scaleCalibration));
}

void ScaleCalibration::setHasFovOffset(bool hasFovOffset) {
	m_scaleCalibration.hasFovOffset = hasFovOffset;
	emit(s_objectiveCalibrationChanged(m_scaleCalibration));
}

void ScaleCalibration::setFovOffsetX(double value) {
	m_scaleCalibration.fovOffsetUm.x = value;
	emit(s_objectiveCalibrationChanged(m_scaleCalibration));
}

void ScaleCalibration::setFovOffsetY(double value) {
	m_scaleCalibration.fovOffsetUm.y = value;
	emit(s_objectiveCalibrationChanged(m_scaleCalibration));
}

void ScaleCalibration::setFovOffsetSigma(double value) {
	m_scaleCalibration.fovOffsetSigmaUm = value;
	emit(s_objectiveCalibrationChanged(m_scaleCalibration));
}

/*
 * FOV-offset auto-measurement.
 *
 * startObjectiveCycleCalibration()/continueObjectiveCycle()/abortObjectiveCycle() are the only
 * way this measurement runs now (the earlier manual, two-button "Set as FOV-offset reference"/
 * "Measure FOV offset" flow was removed from the GUI once this automated flow existed - the
 * underlying captureFovOffsetReferenceImage()/measureFovOffset() functions are unchanged, just
 * called by the cycle instead of by a button). Unlike every other capture path in this class,
 * this flow *does* drive the nosepiece itself, via switchToObjectiveSlotAndVerify() -
 * a deliberate, confirmed exception to "the operator always drives the switch": doing this by
 * hand for M repeated reference<->target cycles is exactly the tedious, error-prone repetition
 * this automation exists to remove. A Z-retract safety step precedes every commanded switch
 * (see beginCycleReferencePhase()/beginCycleTargetPhase()) as the mitigation for the collision
 * risk that hand-driving the changer avoided entirely before. Each cycle pauses once per
 * objective (not just once, at the target) for the operator to refocus, since the two
 * objectives generally are not perfectly parfocal with each other.
 */

void ScaleCalibration::configureCalibrationCameraRoi() {
	// This used to end with m_camera->setSettings(m_cameraSettings) right here, applying the
	// (no-longer-ROI-changing, but still trigger-mode-changing) settings immediately. That is
	// itself unsafe while a live preview is running: Camera::applySettings() is not
	// mutex-guarded, and switching triggerMode away from whatever the running preview loop
	// (Camera::getImageForPreview(), see Camera.cpp) was capturing with breaks that loop too -
	// getImageForPreview()/acquireImage() never fires a software trigger the way
	// getImageForAcquisition() does, so once triggerMode flips to "Software" out from under it,
	// the *preview's own* next frame-grab blocks forever on RetrieveBuffer() as well, while
	// holding Camera::m_mutex - which is why the live image froze, the Stop button stopped
	// responding (it can't interrupt a call the preview loop never returns from to notice the
	// stop flag), and beam-path/preset switching stopped working too (ScanControl shares
	// m_acquisitionThread with this class - see the startWorker() calls in
	// BrillouinAcquisition.cpp - and that thread's own next call into the camera,
	// startAcquisition() below, then blocks right behind the preview waiting on the same mutex).
	//
	// Fix: this function only prepares the desired settings now and does not apply them.
	// Camera::startAcquisition() (called by both callers of this function, right after) already
	// does the right thing atomically under Camera::m_mutex: stop the preview first (safely,
	// letting its current call return), *then* apply the new settings, then start capture. So
	// the actual apply is left to that one call, instead of happening twice - once here,
	// unsafely, and again (redundantly) inside startAcquisition().
	m_cameraSettings = m_camera->getSettings();

	// Deliberately no ROI override - always capture at whatever size the camera is already
	// configured for (full sensor, or whatever preview is currently using), same as
	// Fluorescence::configureCamera(). Previously cropped to a fixed left=1000/top=800/
	// 1000x1000 region regardless of the camera's actual sensor size, which is a separate bug
	// this class no longer has.
	//
	// Trigger mode: mirrors Fluorescence::configureCamera() too. This class never touched
	// triggerMode before, so a capture just inherited whatever the camera was last left in;
	// PointGrey::getImageForAcquisition() only fires a software trigger when
	// triggerMode == "Software", so anything else (e.g. "External", waiting on a hardware
	// trigger line nothing here supplies) blocked this capture's own RetrieveBuffer() forever.
	// Forcing "Software" here guarantees a trigger is actually sent for *this* capture; See the
	// comment above for why applying it must wait for startAcquisition().
	auto cameraType = (std::string)typeid(*m_camera).name();
	if (cameraType == "class uEyeCam" || cameraType == "class PointGrey") {
		m_cameraSettings.readout.triggerMode = L"Software";
	}
#ifdef _DEBUG
	else if (cameraType == "class MockCamera") {
		m_cameraSettings.readout.triggerMode = L"Software";
	}
#endif
	m_cameraSettings.readout.cycleMode = L"Fixed";
	m_cameraSettings.frameCount = 1;
}

std::vector<std::byte> ScaleCalibration::captureBrightfieldImageForFovOffset() {
	if (!m_camera) {
		return {};
	}

	configureCalibrationCameraRoi();

	auto image = std::vector<std::byte>(m_cameraSettings.roi.bytesPerFrame);

	m_camera->startAcquisition(m_cameraSettings);
	m_camera->getImageForAcquisition(&image[0], false);

	// Retry on an all-black frame, same as __acquire()'s scale-calibration capture - std::byte
	// has no arithmetic, so simplemath::sum() needs the same reinterpret-as-unsigned-char
	// view __acquire() already uses for exactly this reason.
	auto image_ = (std::vector<unsigned char>*)&image;
	auto sum = simplemath::sum(*image_);
	auto tryCount{ 0 };
	while (sum == 0 && 5 > tryCount++) {
		m_camera->getImageForAcquisition(&image[0], false);
		sum = simplemath::sum(*image_);
	}
	m_camera->stopAcquisition();

	return image;
}

cv::Mat ScaleCalibration::readAsMat8U(const std::vector<std::byte>& image, int rows, int cols, const std::string& dataType) const {
	if (dataType == "unsigned short") {
		cv::Mat nativeMat(rows, cols, CV_16UC1, (void*)image.data());
		cv::Mat mat8U;
		// matchTemplate() below has only ever run against 8-bit data - downscaling here keeps
		// that pipeline unchanged instead of routing an untested bit depth through it. Full
		// 16-bit precision isn't needed just to align structures.
		nativeMat.convertTo(mat8U, CV_8U, 1.0 / 256.0);
		return mat8U;
	}
	// Direct, copy-free view onto image's own memory - matches the convention __acquire()'s
	// image-matrix construction already uses for the 8-bit case.
	return cv::Mat(rows, cols, CV_8UC1, (void*)image.data());
}

std::string ScaleCalibration::debugCalibrationFolder() const {
	auto folder = std::string{};
	if (!m_linkedCalibrationFilePath.empty()) {
		folder = path(m_linkedCalibrationFilePath).parent_path().string();
	} else if (m_acquisition) {
		folder = m_acquisition->getCurrentFolder();
	}
	if (folder.empty()) {
		return {};
	}
	try {
		create_directories(folder);
	} catch (const filesystem_error&) {
		return {};
	}
	return folder;
}

void ScaleCalibration::saveDebugCalibrationMat(const cv::Mat& mat, const std::string& label) {
	if (mat.empty()) {
		return;
	}
	auto folder = debugCalibrationFolder();
	if (folder.empty()) {
		return;
	}
	auto timestamp = QDateTime::currentDateTime().toString("yyyy-MM-ddTHHmmss.zzz").toStdString();
	auto filepath = folder + "/" + label + "_" + timestamp + ".tif";
	try {
		cv::imwrite(filepath, mat);
	} catch (const cv::Exception&) {
	}
}

void ScaleCalibration::saveDebugCalibrationImage(const std::vector<std::byte>& image, const CAMERA_ROI& roi, const std::string& dataType, const std::string& label) {
	if (image.empty()) {
		return;
	}
	auto mat = readAsMat8U(image, roi.height_binned, roi.width_binned, dataType);
	saveDebugCalibrationMat(mat, label);
}

bool ScaleCalibration::computeFovOffsetShiftUm(
	const std::vector<std::byte>& referenceImage, const CAMERA_ROI& referenceRoi, const ScaleCalibrationData& referenceScale, const std::string& referenceDataType,
	const std::vector<std::byte>& targetImage, const CAMERA_ROI& targetRoi, const ScaleCalibrationData& targetScale, const std::string& targetDataType,
	POINT2* shiftUm, double* estimatedMagnificationChange, std::string* failureReason
) {
	if (referenceImage.empty() || targetImage.empty()) {
		*failureReason = referenceImage.empty() && targetImage.empty()
			? "Neither the reference nor the target image was captured (camera failure)."
			: (referenceImage.empty()
				? "The reference image was not captured (camera failure)."
				: "The target image was not captured (camera failure).");
		return false;
	}

	cv::Mat refMat = readAsMat8U(referenceImage, referenceRoi.height_binned, referenceRoi.width_binned, referenceDataType);
	cv::Mat tgtMat = readAsMat8U(targetImage, targetRoi.height_binned, targetRoi.width_binned, targetDataType);

	// Approximate, isotropic pixel pitch [um/pix] - only to bring the two images to a roughly
	// comparable scale before template matching. The final um result below uses the target's
	// full, non-approximated calibration instead.
	//
	// ScaleCalibrationHelper::isotropicPixelPitchUm() (sqrt(|determinant|) of the pix->um
	// matrix), NOT an average of its diagonal terms: the determinant is the matrix's area-scale
	// factor (um^2/pixel^2), which is correct regardless of any rotation between the camera's
	// pixel axes and the stage axes - averaging only the diagonal (X.x, Y.y) terms silently
	// assumes zero rotation, and is wrong (by a large, rotation-dependent factor - up to totally
	// collapsing to ~0 at 90 degrees) whenever the two objectives' optical paths actually
	// introduce different image rotations, which is exactly what previously produced a grossly
	// inflated "estimated magnification change" (and the resulting nonsense multi-hundred-um
	// shift, since the two images were then rescaled to the wrong relative size before matching)
	// even with two independently-verified-correct per-objective pixel-scale calibrations.
	auto referencePixelSizeUm = ScaleCalibrationHelper::isotropicPixelPitchUm(referenceScale);
	auto targetPixelSizeUm = ScaleCalibrationHelper::isotropicPixelPitchUm(targetScale);
	if (referencePixelSizeUm <= 0.0 || targetPixelSizeUm <= 0.0) {
		// Not an image-content problem at all - this objective has no (non-zero)
		// pixToMicrometer pixel-scale calibration registered yet, i.e. its own "Acquire"
		// (translation between images) has never successfully completed and been applied/
		// saved. The FOV-offset measurement needs that first, for both objectives, purely to
		// know how much to rescale one image before comparing it to the other.
		*failureReason = (referencePixelSizeUm <= 0.0 && targetPixelSizeUm <= 0.0)
			? "Neither the reference nor the target objective has a saved pixel-scale (\"Acquire\"/translation-between-images) calibration yet - run and apply/save that for both objectives first."
			: (referencePixelSizeUm <= 0.0
				? "The reference objective has no saved pixel-scale (\"Acquire\"/translation-between-images) calibration yet - run and apply/save that for it first."
				: "The target objective has no saved pixel-scale (\"Acquire\"/translation-between-images) calibration yet - run and apply/save that for it first.");
		return false;
	}

	auto rescaleFactor = referencePixelSizeUm / targetPixelSizeUm;
	// This is exactly the reference->target magnification ratio the matching below assumes,
	// derived purely from each objective's own stored pixToMicrometer calibration (not from
	// the image content) - the caller compares it against the nominal ratio of the two
	// objectives' typed-in "Magnification" values as a sanity check that the matching is
	// operating at a sane scale.
	*estimatedMagnificationChange = rescaleFactor;
	cv::Mat refMatRescaled;
	cv::resize(refMat, refMatRescaled, cv::Size(), rescaleFactor, rescaleFactor, cv::INTER_LINEAR);

	// matchTemplate() requires the template to fit inside the search image - whichever of the
	// two (now comparably-scaled) images is smaller becomes the template.
	cv::Mat searchMat;
	cv::Mat templateMat;
	bool referenceIsSearch;
	if (refMatRescaled.rows >= tgtMat.rows && refMatRescaled.cols >= tgtMat.cols) {
		searchMat = refMatRescaled;
		templateMat = tgtMat;
		referenceIsSearch = true;
	} else if (tgtMat.rows >= refMatRescaled.rows && tgtMat.cols >= refMatRescaled.cols) {
		searchMat = tgtMat;
		templateMat = refMatRescaled;
		referenceIsSearch = false;
	} else {
		// Neither fits inside the other - very different aspect ratios after rescaling, or a
		// degenerate ROI. Give up rather than guess. This means the two objectives' pixel-scale
		// calibrations imply a very different aspect ratio between the two captured frames -
		// most likely one of the two pixel-scale calibrations is wrong (e.g. x/y swapped),
		// not that the images lack matchable content.
		*failureReason = "The reference and target images have too different an aspect ratio after "
			"rescaling to a common pixel scale - check both objectives' pixel-scale calibrations "
			"(the estimated magnification change reported after a successful measurement is the "
			"sanity check for this).";
		return false;
	}

	// The smaller (already rescaled-to-a-common-pixel-pitch) image is used as the template in
	// full - no additional crop/margin. An earlier version of this code cropped a padded,
	// centered region out of it first, copying the trick __acquire() uses for the (same-size-
	// image) scale-calibration match, where cropping is what creates room to slide at all. Here
	// searchMat and templateMat are already different sizes (that is the whole point - the
	// reference objective's real field of view is physically bigger), so that size difference
	// alone already provides all the room matchTemplate needs; cropping on top of it only threw
	// away real image content for no benefit, which is the actual/main cause of the large,
	// genuinely-different cycle-to-cycle results you saw - a smaller, less distinctive template
	// window is more prone to a locally-similar-but-wrong best match. Using the full image
	// gives the match the most real content to work with.
	cv::Mat templ = templateMat;
	if (templ.rows < 1 || templ.cols < 1 || searchMat.rows < templ.rows || searchMat.cols < templ.cols) {
		*failureReason = "The smaller of the two captured images is empty, or no longer fits inside "
			"the larger one after rescaling to a common pixel scale - capture at a larger ROI, or "
			"check both objectives' pixel-scale calibrations for a gross error.";
		return false;
	}

	// TM_CCOEFF_NORMED (mean-subtracted, normalized cross-correlation) rather than TM_SQDIFF -
	// unlike the same-objective scale-calibration match in __acquire(), reference and target here
	// are captured through two different objectives, whose illumination/brightness (and contrast)
	// commonly differ with magnification. TM_SQDIFF's score is sensitive to absolute intensity
	// offsets/scale between the two images, which can bias or outright break the match; the
	// normalized-correlation coefficient is invariant to a per-image additive/multiplicative
	// brightness difference, so it keeps matching on structure regardless.
	cv::Mat matchResult;
	cv::matchTemplate(searchMat, templ, matchResult, cv::TemplateMatchModes::TM_CCOEFF_NORMED);
	cv::normalize(matchResult, matchResult, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());

	auto minVal = double{};
	auto maxVal = double{};
	auto minLoc = cv::Point{};
	auto maxLoc = cv::Point{};
	cv::minMaxLoc(matchResult, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat());

	// maxLoc (not minLoc - TM_CCOEFF_NORMED's best match is the highest score, the opposite
	// convention from TM_SQDIFF) is where templ's top-left corner best matches inside searchMat.
	// searchMat and templ are NOT the same size here (unlike the __acquire() scale-calibration
	// match this pipeline was originally adapted from) - after rescaling to a common pixel pitch,
	// the reference's real field of view is still a different physical size than the target's
	// (e.g. a lower-mag objective genuinely sees more of the sample), so searchMat (the bigger
	// one) and templ (the full smaller one) differ in size. The correct "zero shift" expected
	// location is therefore where templ's own center would coincide with searchMat's own center -
	// (searchMat.cols - templ.cols) / 2 horizontally, (searchMat.rows - templ.rows) / 2
	// vertically. An earlier version of this code got this reference point wrong (assumed the
	// two images were the same size) and then additionally cropped a margin off templ for no
	// real benefit, discarding image content in a way that made repeated measurements
	// noticeably inconsistent - both are fixed now.
	auto expectedLoc = cv::Point((searchMat.cols - templ.cols) / 2, (searchMat.rows - templ.rows) / 2);
	auto pixelShift = maxLoc - expectedLoc;
	if (!referenceIsSearch) {
		// templ came from the (rescaled) reference and searchMat is the target - the above
		// then measures "reference relative to target", the opposite of "target relative to
		// reference" (searchMat = reference case), so flip it to keep one consistent meaning
		// regardless of which image happened to be larger.
		pixelShift = -pixelShift;
	}

	// Visual sanity check for the operator: the (common-pixel-scale) target image blended at
	// 50% opacity onto the (common-pixel-scale) reference image, positioned exactly where the
	// match above placed it - so misalignment is visible directly, independent of trusting the
	// numeric shift computed below. targetTopLeftInRef = centeredExpectedLocInRef + pixelShift
	// holds regardless of which of refMatRescaled/tgtMat ended up as searchMat/templateMat
	// above: expectedLoc (used to derive pixelShift) is symmetric in the two images' sizes, and
	// the referenceIsSearch sign-flip already applied to pixelShift exactly cancels the sign
	// flip in expectedLoc's own definition between the two branches.
	{
		auto centeredExpectedLocInRef = cv::Point(
			(refMatRescaled.cols - tgtMat.cols) / 2,
			(refMatRescaled.rows - tgtMat.rows) / 2
		);
		auto targetTopLeftInRef = centeredExpectedLocInRef + pixelShift;

		auto refRect = cv::Rect(0, 0, refMatRescaled.cols, refMatRescaled.rows);
		auto targetRectInRef = cv::Rect(targetTopLeftInRef, tgtMat.size());
		auto visibleRect = refRect & targetRectInRef;
		if (visibleRect.area() > 0) {
			auto overlay = refMatRescaled.clone();
			auto sourceRect = cv::Rect(visibleRect.tl() - targetTopLeftInRef, visibleRect.size());
			cv::addWeighted(overlay(visibleRect), 0.5, tgtMat(sourceRect), 0.5, 0.0, overlay(visibleRect));
			saveDebugCalibrationMat(overlay, "fovOverlay");
		}
	}

	// Convert to um using the target's own exact calibration (both images are now at
	// approximately the target's pixel scale after the rescale step above). Matches the same
	// [[pixToMicrometerX.x, pixToMicrometerY.x], [pixToMicrometerX.y, pixToMicrometerY.y]]
	// convention ScaleCalibrationHelper::initializeCalibrationFromPixel() builds its Matrix2 from.
	auto shiftXUm = targetScale.pixToMicrometerX.x * pixelShift.x + targetScale.pixToMicrometerY.x * pixelShift.y;
	auto shiftYUm = targetScale.pixToMicrometerX.y * pixelShift.x + targetScale.pixToMicrometerY.y * pixelShift.y;

	// (shiftXUm, shiftYUm) is "target optical center minus reference optical center", both
	// observed at the same, unmoved stage position - i.e. how far the view apparently jumped
	// when switching objectives. fovOffsetUm is defined as the *compensating stage move*
	// needed to undo that jump (see ScaleCalibrationHelper.h and Brillouin::resolvedGridOriginUm())
	// - the negative of the observed jump. This sign is the single highest-risk-of-being-
	// backwards line in this function; verify empirically before trusting it (capture at the
	// reference objective, note a feature's position, switch to the target, jog the stage by
	// exactly the reported (fovOffsetUm.x, fovOffsetUm.y) and confirm the feature re-centers -
	// if it moves twice as far off instead, negate this).
	shiftUm->x = -shiftXUm;
	shiftUm->y = -shiftYUm;

	return true;
}

void ScaleCalibration::captureFovOffsetReferenceImage(bool resetAccumulatedSamples) {
	if (!m_camera || !m_scanControl) {
		return;
	}

	m_fovReferenceImage = captureBrightfieldImageForFovOffset();
	if (m_fovReferenceImage.empty()) {
		emit(s_scaleCalibrationStatus("Could not capture FOV-offset reference", "Please make sure a brightfield camera is connected."));
		return;
	}
	m_fovReferenceRoi = m_cameraSettings.roi;
	m_fovReferenceDataType = m_cameraSettings.readout.dataType;
	saveDebugCalibrationImage(m_fovReferenceImage, m_fovReferenceRoi, m_fovReferenceDataType,
		"fovReference_slot" + std::to_string(m_scanControl->getActiveObjectiveSlot()));
	// Read directly from this slot's own stored calibration (ScanControl::m_objectiveCalibrations),
	// not the "currently active" mirror (getScaleCalibration()) - the mirror is only guaranteed
	// in sync immediately after a slot *change* observed via handleObjectiveSlotObserved(); reading
	// it here made this measurement depend on that timing/backend-specific signal path instead of
	// on the actually-registered per-slot calibration, which is what previously caused a spurious
	// "no pixel-scale calibration" failure even when one had genuinely been loaded for this slot.
	auto activeCalibration = m_scanControl->getObjectiveCalibration(m_scanControl->getActiveObjectiveSlot());
	m_fovReferenceScaleCalibration = activeCalibration;
	m_fovReferenceObjectiveName = activeCalibration.objectiveName;
	m_fovReferenceObjectiveSlot = m_scanControl->getActiveObjectiveSlot();
	m_fovReferenceMagnification = activeCalibration.magnification;

	if (resetAccumulatedSamples) {
		// A new reference invalidates any samples collected against the previous one. Skipped
		// mid-automated-run (cycles 2..M) so re-capturing a fresh reference image each cycle
		// does not also wipe the mean/sigma being built up across all M cycles.
		m_fovOffsetTargetSlot = -1;
		m_fovOffsetSamplesUm.clear();
	}
}

void ScaleCalibration::measureFovOffset() {
	if (!m_camera || !m_scanControl) {
		return;
	}
	if (m_fovReferenceImage.empty()) {
		emit(s_scaleCalibrationStatus("No FOV-offset reference set", "Start an automated calibration run to capture a reference image first."));
		return;
	}

	auto targetSlot = m_scanControl->getActiveObjectiveSlot();
	if (targetSlot == m_fovReferenceObjectiveSlot) {
		emit(s_scaleCalibrationStatus("Same objective as reference", "Switch to the objective you want to calibrate before measuring."));
		return;
	}
	if (targetSlot != m_fovOffsetTargetSlot) {
		// A different target objective than the previous measurement was accumulating for -
		// those samples don't apply here.
		m_fovOffsetTargetSlot = targetSlot;
		m_fovOffsetSamplesUm.clear();
	}

	auto targetImage = captureBrightfieldImageForFovOffset();
	if (targetImage.empty()) {
		emit(s_scaleCalibrationStatus("Could not capture target image", "Please make sure a brightfield camera is connected."));
		return;
	}
	// Read directly from this slot's own stored calibration, not the "currently active" mirror -
	// see the identical comment in captureFovOffsetReferenceImage() for why.
	auto targetScaleCalibration = m_scanControl->getObjectiveCalibration(targetSlot);
	auto targetDataType = m_cameraSettings.readout.dataType;
	saveDebugCalibrationImage(targetImage, m_cameraSettings.roi, targetDataType, "fovTarget_slot" + std::to_string(targetSlot));

	auto shiftUm = POINT2{};
	auto estimatedMagnificationChange = 0.0;
	auto failureReason = std::string{};
	auto ok = computeFovOffsetShiftUm(
		m_fovReferenceImage, m_fovReferenceRoi, m_fovReferenceScaleCalibration, m_fovReferenceDataType,
		targetImage, m_cameraSettings.roi, targetScaleCalibration, targetDataType,
		&shiftUm, &estimatedMagnificationChange, &failureReason
	);
	if (!ok) {
		emit(s_scaleCalibrationStatus("FOV-offset measurement failed", failureReason));
		return;
	}

	m_fovOffsetSamplesUm.push_back(shiftUm);

	auto meanUm = POINT2{ 0, 0 };
	for (const auto& sample : m_fovOffsetSamplesUm) {
		meanUm.x += sample.x;
		meanUm.y += sample.y;
	}
	meanUm.x /= m_fovOffsetSamplesUm.size();
	meanUm.y /= m_fovOffsetSamplesUm.size();

	auto sigma = 0.0;
	if (m_fovOffsetSamplesUm.size() > 1) {
		auto sumSq = 0.0;
		for (const auto& sample : m_fovOffsetSamplesUm) {
			auto dx = sample.x - meanUm.x;
			auto dy = sample.y - meanUm.y;
			sumSq += dx * dx + dy * dy;
		}
		sigma = std::sqrt(sumSq / (m_fovOffsetSamplesUm.size() - 1));
	}

	// Compose through the reference objective's own already-stored, baseline-relative offset
	// (see this function's doc comment in the header) - so it does not matter which already-
	// calibrated objective was used as the reference for this particular measurement, the
	// result stored here is always this target's offset relative to the same shared baseline
	// every other objective's fovOffsetUm is relative to. If the reference itself has no stored
	// offset (hasFovOffset false - typically true for the baseline objective itself, which has
	// nothing to be offset from), it contributes {0,0}/0, so measuring directly against the
	// baseline still works exactly as before.
	auto referenceCalibration = m_scanControl->getObjectiveCalibration(m_fovReferenceObjectiveSlot);
	auto referenceOffsetUm = referenceCalibration.hasFovOffset ? referenceCalibration.fovOffsetUm : POINT2{ 0, 0 };
	auto referenceSigmaUm = referenceCalibration.hasFovOffset ? referenceCalibration.fovOffsetSigmaUm : 0.0;
	auto composedOffsetUm = POINT2{ referenceOffsetUm.x + meanUm.x, referenceOffsetUm.y + meanUm.y };
	auto composedSigmaUm = std::sqrt(sigma * sigma + referenceSigmaUm * referenceSigmaUm);

	m_scaleCalibration.hasFovOffset = true;
	m_scaleCalibration.fovOffsetUm = composedOffsetUm;
	m_scaleCalibration.fovOffsetSigmaUm = composedSigmaUm;
	m_scaleCalibration.referenceObjectiveName = m_fovReferenceObjectiveName;

	emit(s_objectiveCalibrationChanged(m_scaleCalibration));

	auto sampleCount = std::to_string(m_fovOffsetSamplesUm.size());
	auto message = "Sample " + sampleCount + ": measured shift vs. reference (" + std::to_string(meanUm.x) + ", " + std::to_string(meanUm.y)
		+ ") um, sigma " + std::to_string(sigma) + " um.\nFOV-center offset relative to baseline: ("
		+ std::to_string(composedOffsetUm.x) + ", " + std::to_string(composedOffsetUm.y) + ") um, sigma "
		+ std::to_string(composedSigmaUm) + " um. Repeat (more cycles) for a better sigma, then Save.";

	// Cross-check: estimatedMagnificationChange came purely from each objective's stored
	// pixToMicrometer calibration (what the template matching actually assumed); the nominal
	// ratio below comes from the two objectives' own registered "Magnification" values (read
	// from ScanControl's saved per-slot calibration, not this dialog's edit buffer, so it's
	// correct even if the dialog's Objective name/Magnification fields are still showing
	// whatever objective was active when the dialog was opened). If matching is working
	// correctly, these two should agree to within a few percent - a large disagreement means
	// either the pixel-scale calibration for one of the two objectives is off, or the wrong
	// objective ended up as reference/target for this measurement. Shown to 2 decimal places -
	// full double precision here was noise, not signal, for a ratio this coarse.
	auto targetRegisteredMagnification = m_scanControl->getActiveObjectiveCalibration().magnification;
	message += "\nEstimated magnification change used for image matching (target/reference, from calibrated pixel scale): "
		+ QString::number(estimatedMagnificationChange, 'f', 2).toStdString() + "x";
	if (targetRegisteredMagnification > 0.0 && m_fovReferenceMagnification > 0.0) {
		auto nominalMagnificationChange = targetRegisteredMagnification / m_fovReferenceMagnification;
		message += " (nominal from saved objective magnifications: " + QString::number(nominalMagnificationChange, 'f', 2).toStdString()
			+ "x - large disagreement suggests a bad pixel-scale calibration for one of the two objectives, not this measurement).";
	} else {
		message += " (nominal magnification ratio unavailable - one of the two objectives has no saved \"Magnification\" value).";
	}

	emit(s_scaleCalibrationStatus("FOV offset measured", message));
}

bool ScaleCalibration::findObjectiveElement(DeviceElement* out) const {
	if (!m_scanControl) {
		return false;
	}
	for (const auto& element : m_scanControl->m_deviceElements) {
		if (element.name == "Objective") {
			*out = element;
			return true;
		}
	}
	return false;
}

bool ScaleCalibration::switchToObjectiveSlotAndVerify(int slot) {
	DeviceElement objectiveElement;
	if (!findObjectiveElement(&objectiveElement)) {
		emit(s_scaleCalibrationStatus("No objective changer", "This device has no motorized \"Objective\" element."));
		return false;
	}
	// A plain, direct, same-thread synchronous call - ScaleCalibration and ScanControl already
	// share m_acquisitionThread (see BrillouinAcquisition's startWorker() calls), exactly like
	// ScanControl::setPreset() already calls setElement() the same way internally. Each
	// backend's setElement() (e.g. ZeissMTB::setElement(), ~500ms-scaled) blocks until the
	// physical switch completes and, on the way out, synchronously emits
	// elementPositionChanged() - ScanControl's self-connection to that signal (default
	// same-thread DirectConnection) updates m_activeObjectiveSlot before this call returns. So
	// checking getActiveObjectiveSlot() immediately below is a real, already-settled check, not
	// a race against the 100ms-polled elementPositionsChanged() path (that path only matters
	// for a switch made at the microscope's own panel, not one commanded here).
	m_scanControl->setElement(objectiveElement, (double)slot);
	if (m_scanControl->getActiveObjectiveSlot() != slot) {
		emit(s_scaleCalibrationStatus("Objective switch failed",
			"Expected slot " + std::to_string(slot) + " but the changer reports a different position. Check the nosepiece."));
		return false;
	}
	return true;
}

void ScaleCalibration::startObjectiveCycleCalibration(int referenceSlot, int targetSlot, int cycles, double zRetractDistanceUm) {
	if (m_objectiveCycleState != ObjectiveCycleState::Idle) {
		emit(s_scaleCalibrationStatus("Automated calibration already running", "Abort the current run before starting a new one."));
		return;
	}
	if (!m_camera || !m_scanControl) {
		return;
	}
	if (referenceSlot == targetSlot || cycles < 1) {
		return;
	}

	m_objectiveCycleReferenceSlot = referenceSlot;
	m_objectiveCycleTargetSlot = targetSlot;
	m_objectiveCycleCount = cycles;
	m_objectiveCycleIndex = 1;
	m_objectiveCycleRetractUm = zRetractDistanceUm;

	beginCycleReferencePhase();
}

void ScaleCalibration::continueObjectiveCycle() {
	if (m_objectiveCycleState == ObjectiveCycleState::WaitingForReferenceFocus) {
		// The operator has refocused at the reference objective - capture it now, at (hopefully)
		// good focus, then move on to the target phase.
		captureFovOffsetReferenceImage(m_objectiveCycleIndex == 1);
		if (m_fovReferenceImage.empty()) {
			emit(s_scaleCalibrationStatus("Objective cycle aborted", "Could not capture the reference image."));
			finishObjectiveCycle(true);
			return;
		}
		beginCycleTargetPhase();
		return;
	}
	if (m_objectiveCycleState == ObjectiveCycleState::WaitingForTargetFocus) {
		// The operator has refocused at the target objective - measure now, then start the next
		// cycle's reference phase or finish. Unmodified from before this two-pause split:
		// m_objectiveCycleTargetSlot is the same slot across every cycle in this run, so
		// measureFovOffset()'s own "different target than last time, reset samples" guard never
		// fires mid-run, and repeated calls accumulate into m_fovOffsetSamplesUm exactly as
		// repeated manual clicks already do (see its class-level doc comment).
		measureFovOffset();

		if (m_objectiveCycleIndex >= m_objectiveCycleCount) {
			finishObjectiveCycle(false);
			return;
		}
		m_objectiveCycleIndex++;
		beginCycleReferencePhase();
		return;
	}
}

void ScaleCalibration::abortObjectiveCycle() {
	if (m_objectiveCycleState == ObjectiveCycleState::Idle) {
		return;
	}
	finishObjectiveCycle(true);
}

void ScaleCalibration::beginCycleReferencePhase() {
	m_objectiveCycleState = ObjectiveCycleState::Running;
	emit(s_objectiveCycleProgress(m_objectiveCycleIndex, m_objectiveCycleCount, false));

	// Retract Z before every commanded switch, as a safety margin against a collision between
	// objectives of different parfocal length/working distance - see the class-level comment
	// above startObjectiveCycleCalibration(). Relative move, sign/magnitude as entered by the
	// operator (Automated calibration: FOV > "Z retract [um]").
	m_scanControl->movePosition(POINT3{ 0, 0, m_objectiveCycleRetractUm });
	// Switch to the reference objective and verify it landed.
	if (!switchToObjectiveSlotAndVerify(m_objectiveCycleReferenceSlot)) {
		finishObjectiveCycle(true);
		return;
	}

	// Pause here for the operator to refocus at the REFERENCE objective before it is captured -
	// an earlier version of this code captured the reference image immediately after switching,
	// with no refocus step at all, relying purely on the Z-retract compensation. That is fine
	// for cycle 1 (Z was wherever the operator had it focused at the reference objective before
	// clicking Start), but from cycle 2 onward Z is wherever the operator last refocused for the
	// TARGET objective (see beginCycleTargetPhase()) - not restored to the reference objective's
	// own focus in between (deliberately: "restoring" a fixed pre-retract Z here could
	// reintroduce the very collision risk the retract exists to avoid, since the two objectives'
	// parfocal planes generally differ). Without this pause, every cycle after the first
	// measured a defocused (and therefore less reliable, more scattered) reference image against
	// a properly focused target image - consistent with reported results starting small and
	// growing/becoming noisier over the course of a run. This is a real return to the caller/
	// event loop, not a blocking wait - ScaleCalibration shares m_acquisitionThread with
	// ScanControl and every other acquisition mode, so blocking here would freeze all of them,
	// exactly like the earlier trigger-mode/ROI bugs in this class did. continueObjectiveCycle()
	// (invoked from a GUI button click, via QMetaObject::invokeMethod like every other GUI ->
	// ScaleCalibration call) is what resumes from here.
	m_objectiveCycleState = ObjectiveCycleState::WaitingForReferenceFocus;
	emit(s_objectiveCycleProgress(m_objectiveCycleIndex, m_objectiveCycleCount, true));
	emit(s_scaleCalibrationStatus("Refocus and continue",
		"Refocus at the reference objective, then click \"Continue\" (cycle " + std::to_string(m_objectiveCycleIndex)
		+ " of " + std::to_string(m_objectiveCycleCount) + ")."));
}

void ScaleCalibration::beginCycleTargetPhase() {
	m_objectiveCycleState = ObjectiveCycleState::Running;

	// Retract again before the second switch of this cycle, for the same reason as in
	// beginCycleReferencePhase().
	m_scanControl->movePosition(POINT3{ 0, 0, m_objectiveCycleRetractUm });
	// Switch to the target objective and verify it landed.
	if (!switchToObjectiveSlotAndVerify(m_objectiveCycleTargetSlot)) {
		finishObjectiveCycle(true);
		return;
	}

	// Pause for the operator to refocus at the TARGET objective before measureFovOffset() runs -
	// see beginCycleReferencePhase() for why this pause (and the matching one there) exist, and
	// why Z is never auto-restored between them.
	m_objectiveCycleState = ObjectiveCycleState::WaitingForTargetFocus;
	emit(s_objectiveCycleProgress(m_objectiveCycleIndex, m_objectiveCycleCount, true));
	emit(s_scaleCalibrationStatus("Refocus and continue",
		"Refocus at the target objective, then click \"Continue\" (cycle " + std::to_string(m_objectiveCycleIndex)
		+ " of " + std::to_string(m_objectiveCycleCount) + ")."));
}

void ScaleCalibration::finishObjectiveCycle(bool aborted) {
	m_objectiveCycleState = ObjectiveCycleState::Idle;
	emit(s_objectiveCycleProgress(0, m_objectiveCycleCount, false));
	if (aborted) {
		emit(s_scaleCalibrationStatus("Automated calibration aborted",
			"Stopped after cycle " + std::to_string(m_objectiveCycleIndex) + " of " + std::to_string(m_objectiveCycleCount)
			+ ". Whatever samples were already accumulated are still available below - Save if usable, or start a new run."));
	} else {
		emit(s_scaleCalibrationStatus("Automated calibration finished",
			"Completed " + std::to_string(m_objectiveCycleCount) + " cycles. Review the mean offset/sigma and the "
			"estimated-magnification-change sanity check above, then Save if it looks right."));
	}
}

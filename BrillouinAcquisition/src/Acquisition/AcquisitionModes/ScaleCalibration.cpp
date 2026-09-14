#include "stdafx.h"
#include "filesystem"
#include "ScaleCalibration.h"

#include "src/helper/h5_helper.h"

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>
#include <type_traits>

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

void ScaleCalibration::load(std::string filepath) {

	using namespace std::filesystem;

	if (exists(filepath)) {
		try {
			readCalibrationFile(filepath, &m_scaleCalibration);

			// Apply the scale calibration
			apply();
		} catch (H5::Exception& exception) {
			emit(s_scaleCalibrationStatus("Could not load the scale calibration", "Please select a valid scale calibration file."));
		}
	}
}

void ScaleCalibration::autoLoadCalibrationsFromFolder(std::string folder) {
	using namespace std::filesystem;

	if (folder.empty() || !exists(folder) || !is_directory(folder)) {
		return;
	}

	struct Candidate {
		std::string filepath;
		ObjectiveCalibrationData data;
	};
	// objectiveName -> every file in the folder that declares that name.
	std::map<std::string, std::vector<Candidate>> byName;
	// filepath -> why it was rejected before even getting a name group (parse failure,
	// degenerate matrix, or no objectiveName at all).
	std::vector<std::pair<std::string, std::string>> rejected;

	for (const auto& entry : directory_iterator(folder)) {
		if (!entry.is_regular_file() || entry.path().extension() != ".h5") {
			continue;
		}
		auto filepath = entry.path().string();
		ObjectiveCalibrationData data{};
		try {
			readCalibrationFile(filepath, &data);
			// Derive the inverse matrix, same as apply() does - also serves as a validity
			// check (throws on a degenerate/non-basis calibration). Caught with a bare "..."
			// rather than "H5::Exception&"/"std::exception&" specifically - readCalibrationFile()
			// can throw either (H5::Exception from a non-HDF5/corrupt file, std::exception from
			// this call on a degenerate matrix) and both are equally "not usable", so there is
			// no different handling to justify telling them apart here.
			ScaleCalibrationHelper::initializeCalibrationFromPixel(&data);
		} catch (...) {
			rejected.emplace_back(filepath, "not a valid scale calibration file");
			continue;
		}
		if (data.objectiveName.empty()) {
			rejected.emplace_back(filepath, "no objective name set in the file");
			continue;
		}
		byName[data.objectiveName].push_back(Candidate{ filepath, data });
	}

	std::string appliedText;
	std::string warningText;
	for (const auto& reason : rejected) {
		warningText += reason.first + ": " + reason.second + "\n";
	}

	// First pass: for every objectiveName with exactly one file, work out which slot it wants
	// and group by slot - this has to happen before anything is applied, so a slot claimed by
	// two differently-named files can be caught and rejected for BOTH of them, rather than the
	// first one processed winning silently and the second being told (incorrectly) that it
	// lost to an already-applied calibration.
	std::map<int, std::vector<std::string>> slotClaims;			// slot -> objectiveNames claiming it
	std::map<int, const Candidate*> slotCandidate;					// slot -> its (so far unique) candidate

	for (const auto& [name, candidates] : byName) {
		if (candidates.size() > 1) {
			warningText += "Multiple calibration files found for objective \"" + name + "\" - none applied automatically:\n";
			for (const auto& candidate : candidates) {
				warningText += "  " + candidate.filepath + "\n";
			}
			continue;
		}

		const auto& candidate = candidates.front();
		auto slot = candidate.data.objectiveSlot;
		if (slot < 0 || !m_scanControl->isValidObjectiveSlot(slot)) {
			warningText += candidate.filepath + ": objective \"" + name + "\" has no valid nosepiece slot recorded (calibrate/save it again while this objective is active) - not applied automatically.\n";
			continue;
		}
		slotClaims[slot].push_back(name);
		slotCandidate[slot] = &candidate;
	}

	// Second pass: only apply where exactly one objectiveName claimed the slot.
	for (const auto& [slot, names] : slotClaims) {
		if (names.size() > 1) {
			std::string nameList;
			for (const auto& name : names) {
				nameList += "\"" + name + "\" ";
			}
			warningText += "Slot " + std::to_string(slot) + " is claimed by more than one objective (" + nameList + ") - none applied automatically for that slot.\n";
			continue;
		}

		const auto& candidate = *slotCandidate[slot];
		m_scanControl->setObjectiveCalibration(slot, candidate.data);
		appliedText += names.front() + " -> slot " + std::to_string(slot) + " (" + candidate.filepath + ")\n";
	}

	emit(s_calibrationAutoLoadSummary(appliedText, warningText));
}

void ScaleCalibration::loadCalibrationForSlot(int slot, std::string filepath) {
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
		// Same validity check autoLoadCalibrationsFromFolder() uses - also throws on a
		// degenerate/non-basis calibration, not just an unreadable file.
		ScaleCalibrationHelper::initializeCalibrationFromPixel(&data);
	} catch (...) {
		emit(s_scaleCalibrationStatus("Could not link calibration file",
			"\"" + filepath + "\" is not a valid scale calibration file."));
		return;
	}

	// Explicit link, unlike autoLoadCalibrationsFromFolder()'s name/slot-matching - register
	// against the requested slot regardless of what the file's own objectiveName/objectiveSlot
	// say. setObjectiveCalibration() applies it live immediately if slot is already active, and
	// unconditionally on every future switch to it either way (ScanControl::
	// handleObjectiveSlotObserved()).
	m_scanControl->setObjectiveCalibration(slot, data);
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

	// Objective-identity/FOV-offset/slot fields - absent on a calibration file saved before
	// they existed, in which case whatever *out already had (its defaults, if freshly
	// constructed) is left as-is.
	readAttributeOptional(root, "objectiveName", &out->objectiveName);
	readAttributeOptional(root, "magnification", &out->magnification);
	readAttributeOptional(root, "referenceObjectiveName", &out->referenceObjectiveName);
	readAttributeOptional(root, "calibrationDate", &out->calibrationDate);
	auto hasFovOffsetValue = out->hasFovOffset ? 1.0 : 0.0;
	readAttributeOptional(root, "hasFovOffset", &hasFovOffsetValue);
	out->hasFovOffset = hasFovOffsetValue != 0.0;
	readAttributeOptional(root, "fovOffsetX", &out->fovOffsetUm.x);
	readAttributeOptional(root, "fovOffsetY", &out->fovOffsetUm.y);
	readAttributeOptional(root, "fovOffsetSigma", &out->fovOffsetSigmaUm);
	auto objectiveSlotValue = (double)out->objectiveSlot;
	readAttributeOptional(root, "objectiveSlot", &objectiveSlotValue);
	out->objectiveSlot = (int)objectiveSlotValue;
}

void ScaleCalibration::writeCalibrationMetadata(H5::Group& root) {
	// Record which slot this calibration is being saved from - apply() (called right after by
	// both save() and saveCalibration()'s callers) already assumes the operator is standing at
	// the objective being calibrated, so the currently active slot is exactly the right value
	// to persist for the calibrations-folder auto-load to key off of later (see
	// autoLoadCalibrationsFromFolder()).
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
	writeAttribute(root, "objectiveSlot", (double)m_scaleCalibration.objectiveSlot);
}

void ScaleCalibration::saveCalibration() {
	auto filepath = newCalibrationFilePath();
	try {
		auto file = H5::H5File(&filepath[0], H5F_ACC_TRUNC);
		auto root = file.openGroup("/");
		writeCalibrationMetadata(root);
	} catch (H5::Exception& exception) {
		emit(s_scaleCalibrationStatus("Could not save the scale calibration", "Please select a writable working directory."));
	}
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

void ScaleCalibration::readAttributeOptional(const H5::H5Object& parent, std::string name, double* value) {
	// Missing on a calibration file saved before the objective fields existed - leave the
	// caller's pre-set default untouched rather than throwing and aborting the whole load()
	// (which would otherwise also lose the still-valid scale calibration fields read before
	// this point).
	if (!parent.attrExists(name.c_str())) {
		return;
	}
	readAttribute(parent, name, value);
}

void ScaleCalibration::readAttributeOptional(const H5::H5Object& parent, std::string name, std::string* value) {
	if (!parent.attrExists(name.c_str())) {
		return;
	}
	try {
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
	} catch (H5::Exception&) {
	}
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

	// Emit it to the main GUI thread
	emit(s_scaleCalibrationAcquisitionProgress(0.0));
	emit(s_scaleCalibrationChanged(m_scaleCalibration));
	emit(s_objectiveCalibrationChanged(m_scaleCalibration));
	emit(s_Ds_changed(m_Ds));
}

void ScaleCalibration::apply() {
	try {
		ScaleCalibrationHelper::initializeCalibrationFromMicrometer(&m_scaleCalibration);
		ScaleCalibrationHelper::initializeCalibrationFromPixel(&m_scaleCalibration);
		// Registers against whichever objective slot is currently active (see
		// ScanControl::setObjectiveCalibration()) - the operator is expected to have already
		// switched to the objective being calibrated, exactly like the existing
		// acquire-based procedure already implicitly assumes. This also applies the scale
		// calibration to the live scanControl immediately (setObjectiveCalibration() does
		// that itself when the slot matches the active one), so a separate
		// setScaleCalibration() call is no longer needed here.
		m_scanControl->setObjectiveCalibration(m_scanControl->getActiveObjectiveSlot(), m_scaleCalibration);
		emit(s_closeScaleCalibrationDialog());
	} catch (std::exception& e) {
		emit(s_scaleCalibrationStatus("Cannot apply scale calibration", "The provided scale calibration is invalid."));
	}
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

void ScaleCalibration::setReferenceObjectiveName(QString name) {
	m_scaleCalibration.referenceObjectiveName = name.toStdString();
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
 * setFovOffsetReference()/measureFovOffset() (the manual, two-button flow) still never command
 * an objective switch themselves - only setElement()-driven (via the beampath buttons) or
 * physically-at-the-microscope switches move the nosepiece for that flow. The operator drives
 * the actual switch there; those two functions only automate the error-prone part (measuring
 * the resulting pixel shift by eye).
 *
 * startObjectiveCycleCalibration()/continueObjectiveCycle()/abortObjectiveCycle() (the
 * automated multi-cycle flow, added later) are a deliberate, confirmed exception to that: they
 * do drive the nosepiece themselves, via switchToObjectiveSlotAndVerify(). Doing this by hand
 * for M repeated reference<->target cycles is exactly the tedious, error-prone repetition this
 * automation exists to remove - a Z-retract safety step precedes every commanded switch (see
 * runObjectiveCycleStep()) as the mitigation for the collision risk that hand-driving the
 * changer avoided entirely before.
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

	// Approximate, isotropic pixel pitch [um/pix] - only to bring the two images to a
	// roughly comparable scale before template matching. The final um result below uses the
	// target's full, non-approximated calibration instead.
	auto referencePixelSizeUm = 0.5 * (std::abs(referenceScale.pixToMicrometerX.x) + std::abs(referenceScale.pixToMicrometerY.y));
	auto targetPixelSizeUm = 0.5 * (std::abs(targetScale.pixToMicrometerX.x) + std::abs(targetScale.pixToMicrometerY.y));
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

	// Crop a padded, centered region out of the template so the search has room to find a
	// shift in any direction (same trick __acquire() uses for the scale-calibration match).
	auto padding = std::min(templateMat.rows, templateMat.cols) / 5;
	if (padding < 1 || templateMat.rows <= 2 * padding || templateMat.cols <= 2 * padding) {
		*failureReason = "The smaller of the two captured images is too small to search for a match "
			"in (after rescaling to a common pixel scale) - capture at a larger ROI, or check both "
			"objectives' pixel-scale calibrations for a gross error.";
		return false;
	}
	cv::Rect templateROI(padding, padding, templateMat.cols - 2 * padding, templateMat.rows - 2 * padding);
	cv::Mat templ = templateMat(templateROI);

	if (searchMat.rows < templ.rows || searchMat.cols < templ.cols) {
		*failureReason = "The search region ended up smaller than the template after cropping - "
			"this should not happen given the size check above; please report this.";
		return false;
	}

	cv::Mat matchResult;
	cv::matchTemplate(searchMat, templ, matchResult, cv::TemplateMatchModes::TM_SQDIFF);
	cv::normalize(matchResult, matchResult, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());

	auto minVal = double{};
	auto maxVal = double{};
	auto minLoc = cv::Point{};
	auto maxLoc = cv::Point{};
	cv::minMaxLoc(matchResult, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat());

	// minLoc is where templ's top-left corner best matches inside searchMat. templ's content
	// was itself cropped from templateMat's own center with `padding` margin, and
	// searchMat/templateMat now share the same (rescaled) pixel scale and were both captured
	// at the same, deliberately-unmoved stage position - so if the two objectives were
	// perfectly co-centered, minLoc would land exactly at (padding, padding) (templ's
	// un-cropped position). Any deviation from that is the pixel shift between the two
	// objectives' optical centers, as observed in whichever image ended up as the search image.
	auto pixelShift = minLoc - cv::Point(padding, padding);
	if (!referenceIsSearch) {
		// templ came from the (rescaled) reference and searchMat is the target - the above
		// then measures "reference relative to target", the opposite of "target relative to
		// reference" (searchMat = reference case), so flip it to keep one consistent meaning
		// regardless of which image happened to be larger.
		pixelShift = -pixelShift;
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

void ScaleCalibration::setFovOffsetReference() {
	captureFovOffsetReferenceImage(/*resetAccumulatedSamples=*/true);
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
	m_fovReferenceScaleCalibration = m_scanControl->getScaleCalibration();
	auto activeCalibration = m_scanControl->getActiveObjectiveCalibration();
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

	emit(s_scaleCalibrationStatus("FOV-offset reference captured",
		"Without moving the stage, switch to the objective you want to calibrate and click \"Measure FOV offset\"."));
}

void ScaleCalibration::measureFovOffset() {
	if (!m_camera || !m_scanControl) {
		return;
	}
	if (m_fovReferenceImage.empty()) {
		emit(s_scaleCalibrationStatus("No FOV-offset reference set", "Click \"Set as FOV-offset reference\" at the reference objective first."));
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
	auto targetScaleCalibration = m_scanControl->getScaleCalibration();
	auto targetDataType = m_cameraSettings.readout.dataType;

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

	m_scaleCalibration.hasFovOffset = true;
	m_scaleCalibration.fovOffsetUm = meanUm;
	m_scaleCalibration.fovOffsetSigmaUm = sigma;
	m_scaleCalibration.referenceObjectiveName = m_fovReferenceObjectiveName;

	emit(s_objectiveCalibrationChanged(m_scaleCalibration));

	auto sampleCount = std::to_string(m_fovOffsetSamplesUm.size());
	auto message = "Sample " + sampleCount + ": mean offset (" + std::to_string(meanUm.x) + ", " + std::to_string(meanUm.y)
		+ ") um, sigma " + std::to_string(sigma) + " um. Repeat (switch away and back) for a better sigma, then Apply/Save.";

	// Cross-check: estimatedMagnificationChange came purely from each objective's stored
	// pixToMicrometer calibration (what the template matching actually assumed); the nominal
	// ratio below comes from the two objectives' own registered "Magnification" values (read
	// from ScanControl's saved per-slot calibration, not this dialog's edit buffer, so it's
	// correct even if the dialog's Objective name/Magnification fields are still showing
	// whatever objective was active when the dialog was opened). If matching is working
	// correctly, these two should agree to within a few percent - a large disagreement means
	// either the pixel-scale calibration for one of the two objectives is off, or the wrong
	// objective ended up as reference/target for this measurement.
	auto targetRegisteredMagnification = m_scanControl->getActiveObjectiveCalibration().magnification;
	message += "\nEstimated magnification change used for image matching (target/reference, from calibrated pixel scale): "
		+ std::to_string(estimatedMagnificationChange) + "x";
	if (targetRegisteredMagnification > 0.0 && m_fovReferenceMagnification > 0.0) {
		auto nominalMagnificationChange = targetRegisteredMagnification / m_fovReferenceMagnification;
		message += " (nominal from saved objective magnifications: " + std::to_string(nominalMagnificationChange)
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

	runObjectiveCycleStep();
}

void ScaleCalibration::continueObjectiveCycle() {
	if (m_objectiveCycleState != ObjectiveCycleState::WaitingForContinue) {
		return;
	}
	// Unmodified - m_objectiveCycleTargetSlot is the same slot across every cycle in this run,
	// so measureFovOffset()'s own "different target than last time, reset samples" guard never
	// fires mid-run, and repeated calls accumulate into m_fovOffsetSamplesUm exactly as
	// repeated manual clicks already do (see its class-level doc comment).
	measureFovOffset();

	if (m_objectiveCycleIndex >= m_objectiveCycleCount) {
		finishObjectiveCycle(false);
		return;
	}
	m_objectiveCycleIndex++;
	runObjectiveCycleStep();
}

void ScaleCalibration::abortObjectiveCycle() {
	if (m_objectiveCycleState == ObjectiveCycleState::Idle) {
		return;
	}
	finishObjectiveCycle(true);
}

void ScaleCalibration::runObjectiveCycleStep() {
	m_objectiveCycleState = ObjectiveCycleState::Running;
	emit(s_objectiveCycleProgress(m_objectiveCycleIndex, m_objectiveCycleCount, false));

	// (a) Retract Z before every commanded switch, as a safety margin against a collision
	// between objectives of different parfocal length/working distance - see the class-level
	// comment above startObjectiveCycleCalibration(). Relative move, sign/magnitude as entered
	// by the operator (Automated calibration: run > "Z retract [um]").
	m_scanControl->movePosition(POINT3{ 0, 0, m_objectiveCycleRetractUm });
	// (b)(c) Switch to the reference objective and verify it landed.
	if (!switchToObjectiveSlotAndVerify(m_objectiveCycleReferenceSlot)) {
		finishObjectiveCycle(true);
		return;
	}
	// (d) Capture a fresh reference image every cycle - only reset the accumulated samples on
	// the very first cycle (see captureFovOffsetReferenceImage()'s own comment).
	captureFovOffsetReferenceImage(m_objectiveCycleIndex == 1);
	if (m_fovReferenceImage.empty()) {
		emit(s_scaleCalibrationStatus("Objective cycle aborted", "Could not capture the reference image."));
		finishObjectiveCycle(true);
		return;
	}

	// (e) Retract again before the second switch of this cycle, for the same reason as (a).
	m_scanControl->movePosition(POINT3{ 0, 0, m_objectiveCycleRetractUm });
	// (f) Switch to the target objective and verify it landed.
	if (!switchToObjectiveSlotAndVerify(m_objectiveCycleTargetSlot)) {
		finishObjectiveCycle(true);
		return;
	}

	// (g) Pause here for the operator to refocus - Z was just retracted twice and neither
	// switch restores it (deliberately: this objective's parfocal plane is not the previous
	// one's, so "restoring" the pre-retract Z here could reintroduce the very collision risk
	// the retract exists to avoid - see continueObjectiveCycle()/finishObjectiveCycle(), Z is
	// never auto-restored even once the whole run ends). This is a real return to the caller/
	// event loop, not a blocking wait - ScaleCalibration shares m_acquisitionThread with
	// ScanControl and every other acquisition mode, so blocking here would freeze all of them,
	// exactly like the earlier trigger-mode/ROI bugs in this class did. continueObjectiveCycle()
	// (invoked from a GUI button click, via QMetaObject::invokeMethod like every other GUI ->
	// ScaleCalibration call) is what resumes from here.
	m_objectiveCycleState = ObjectiveCycleState::WaitingForContinue;
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
			+ ". Whatever samples were already accumulated are still available below - Apply/Save if usable, or start a new run."));
	} else {
		emit(s_scaleCalibrationStatus("Automated calibration finished",
			"Completed " + std::to_string(m_objectiveCycleCount) + " cycles. Review the mean offset/sigma and the "
			"estimated-magnification-change sanity check above, then Apply/Save if it looks right."));
	}
}

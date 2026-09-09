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
 * Deliberately never commands an objective switch itself - only setElement()-driven or
 * physically-at-the-microscope switches exist elsewhere in this codebase, and having this
 * class blindly drive the nosepiece back and forth (needing to know the reference slot,
 * wait for settle, handle a failed switch, ...) is a different, larger, and untested piece
 * of hardware automation than "capture an image and run OpenCV on it", which this class
 * already does safely. The operator drives the actual switch; this only automates the
 * error-prone part (measuring the resulting pixel shift by eye).
 */

void ScaleCalibration::configureCalibrationCameraRoi() {
	m_cameraSettings = m_camera->getSettings();
	m_cameraSettings.roi.left = 1000;
	m_cameraSettings.roi.top = 800;
	m_cameraSettings.roi.width_physical = 1000;
	m_cameraSettings.roi.height_physical = 1000;
	m_cameraSettings.frameCount = 1;
	m_camera->setSettings(m_cameraSettings);
	m_cameraSettings = m_camera->getSettings();
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
	POINT2* shiftUm
) {
	if (referenceImage.empty() || targetImage.empty()) {
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
		return false;
	}

	auto rescaleFactor = referencePixelSizeUm / targetPixelSizeUm;
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
		// degenerate ROI. Give up rather than guess.
		return false;
	}

	// Crop a padded, centered region out of the template so the search has room to find a
	// shift in any direction (same trick __acquire() uses for the scale-calibration match).
	auto padding = std::min(templateMat.rows, templateMat.cols) / 5;
	if (padding < 1 || templateMat.rows <= 2 * padding || templateMat.cols <= 2 * padding) {
		return false;
	}
	cv::Rect templateROI(padding, padding, templateMat.cols - 2 * padding, templateMat.rows - 2 * padding);
	cv::Mat templ = templateMat(templateROI);

	if (searchMat.rows < templ.rows || searchMat.cols < templ.cols) {
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

	// A new reference invalidates any samples collected against the previous one.
	m_fovOffsetTargetSlot = -1;
	m_fovOffsetSamplesUm.clear();

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
	auto ok = computeFovOffsetShiftUm(
		m_fovReferenceImage, m_fovReferenceRoi, m_fovReferenceScaleCalibration, m_fovReferenceDataType,
		targetImage, m_cameraSettings.roi, targetScaleCalibration, targetDataType,
		&shiftUm
	);
	if (!ok) {
		emit(s_scaleCalibrationStatus("FOV-offset measurement failed", "Please make sure there are distinct structures visible in both objectives' field of view."));
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
	emit(s_scaleCalibrationStatus("FOV offset measured", message));
}

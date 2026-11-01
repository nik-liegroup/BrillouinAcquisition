#include "stdafx.h"
#include "Fluorescence.h"
#include "Brillouin.h"

#include <cmath>

/*
 * Public definitions
 */

Fluorescence::Fluorescence(QObject* parent, Acquisition* acquisition, Camera*& camera,
	ScanControl*& scanControl, Brillouin* brillouinMode)
	: AcquisitionMode(parent, acquisition, scanControl), m_camera(camera),
	m_brillouinMode(brillouinMode) {}

Fluorescence::~Fluorescence() {
}

/*
 * Public slots
 */

void Fluorescence::startRepetitions() {
	startRepetitions({});
}

void Fluorescence::startRepetitions(std::vector<FLUORESCENCE_MODE> modes) {
	std::vector<ChannelSettings*> channels;
	// If the provided mode vector is empty, acquire the enabled channels.
	if (!modes.size()) {
		channels = getEnabledChannels();
		// If no channels are enabled, return.
		if (!channels.size()) {
			return;
		}
		// Else, find the channel settings for the given modes.
	}
	else {
		for (auto const& mode : modes) {
			auto channel = getChannelSettings(mode);
			channels.push_back(channel);
		}
	}

	bool allowed = m_acquisition->enableMode(ACQUISITION_MODE::FLUORESCENCE);
	if (!allowed) {
		return;
	}

	// Save current preview channel to re-enable it after acquisition
	m_previousPreviewChannel = m_currentPreviewChannel;
	// stop preview if necessary
	if (m_currentPreviewChannel != FLUORESCENCE_MODE::NONE) {
		m_currentPreviewChannel = FLUORESCENCE_MODE::NONE;
		if (m_camera) {
			m_camera->stopPreview();
		}
	}

	// reset abort flag
	m_abort = false;

	configureCamera();

	m_acquisition->newRepetition(ACQUISITION_MODE::FLUORESCENCE);

	// start repetition
	acquire(m_acquisition->m_storage, channels);

	m_acquisition->disableMode(ACQUISITION_MODE::FLUORESCENCE);

	if (m_previousPreviewChannel != FLUORESCENCE_MODE::NONE) {
		startStopPreview(m_previousPreviewChannel);
	}
}

void Fluorescence::initialize() {
	m_settings.camera.gain = 10;
	emit(s_acqSettingsChanged(m_settings));
}

void Fluorescence::setChannel(FLUORESCENCE_MODE mode, bool enabled) {
	ChannelSettings* channel = getChannelSettings(mode);
	channel->enabled = enabled;
	emit(s_acqSettingsChanged(m_settings));
}

void Fluorescence::setExposure(FLUORESCENCE_MODE mode, int exposure) {
	ChannelSettings* channel = getChannelSettings(mode);
	channel->exposure = exposure;
	// Apply the settings immediately if the mode preview is running
	if (m_currentPreviewChannel == mode) {
		m_settings.camera.exposureTime = 1e-3 * channel->exposure;
		if (m_camera) {
			QMetaObject::invokeMethod(
				m_camera,
				[camera = m_camera, channel]() { camera->setSetting(CAMERA_SETTING::EXPOSURE, 1e-3 * channel->exposure); },
				Qt::AutoConnection
			);
		}
	}
	emit(s_acqSettingsChanged(m_settings));
}

void Fluorescence::setGain(FLUORESCENCE_MODE mode, int gain) {
	ChannelSettings* channel = getChannelSettings(mode);
	channel->gain = gain;
	// Apply the settings immediately if the mode preview is running
	if (m_currentPreviewChannel == mode) {
		m_settings.camera.gain = channel->gain;
		if (m_camera) {
			QMetaObject::invokeMethod(
				m_camera,
				[camera = m_camera, channel]() { camera->setSetting(CAMERA_SETTING::GAIN, channel->gain); },
				Qt::AutoConnection
			);
		}
	}
	emit(s_acqSettingsChanged(m_settings));
}

void Fluorescence::startStopPreview(FLUORESCENCE_MODE mode) {
	if (mode == m_currentPreviewChannel || mode == FLUORESCENCE_MODE::NONE) {
		// stop preview
		m_currentPreviewChannel = FLUORESCENCE_MODE::NONE;
		if (m_camera) {
			QMetaObject::invokeMethod(
				m_camera,
				[camera = m_camera]() { camera->stopPreview(); },
				Qt::AutoConnection
			);
		}
	} else {
		// if preview is already running, stop it first
		if (m_camera && m_camera->m_isPreviewRunning) {
			m_camera->stopPreview();
		}
		m_currentPreviewChannel = mode;
		ChannelSettings* channel = getChannelSettings(mode);

		// move to Fluorescence configuration
		if (m_scanControl) {
			m_scanControl->setPreset(channel->preset);
		}

		// start image acquisition
		m_settings.camera.exposureTime = 1e-3 * channel->exposure;
		m_settings.camera.gain = channel->gain;
		if (m_camera) {
			m_camera->setSettings(m_settings.camera);
			QMetaObject::invokeMethod(
				m_camera,
				[camera = m_camera]() { camera->startPreview(); },
				Qt::AutoConnection
			);
		}
	}
	emit(s_previewRunning(m_currentPreviewChannel));
}

/*
 * Private definitions
 */

void Fluorescence::abortMode(std::unique_ptr <StorageWrapper>& storage) {
	m_acquisition->disableMode(ACQUISITION_MODE::FLUORESCENCE);

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
}

ChannelSettings* Fluorescence::getChannelSettings(FLUORESCENCE_MODE mode) {
	if (mode == FLUORESCENCE_MODE::BLUE) {
		return &m_settings.blue;
	} else if (mode == FLUORESCENCE_MODE::GREEN) {
		return &m_settings.green;
	} else if (mode == FLUORESCENCE_MODE::RED) {
		return &m_settings.red;
	} else if (mode == FLUORESCENCE_MODE::BRIGHTFIELD) {
		return &m_settings.brightfield;
	} else {
		return nullptr;
	}
}

std::vector<ChannelSettings*> Fluorescence::getEnabledChannels() {
	std::vector<ChannelSettings*> enabledChannels;
	for (gsl::index i{ 0 }; i < (int)FLUORESCENCE_MODE::MODE_COUNT; i++) {

		ChannelSettings* channel = getChannelSettings((FLUORESCENCE_MODE)i);

		if (channel == nullptr) {
			continue;
		}
		// Don't acquire this mode if it is not enabled
		if (!channel->enabled) {
			continue;
		} else {
			enabledChannels.push_back(channel);
		}
	}
	return enabledChannels;
}

void Fluorescence::configureCamera() {

	if (!m_camera) {
		return;
	}

	auto cameraType = (std::string)typeid(*m_camera).name();

	m_settings.camera = m_camera->getSettings();

	// Deliberately no ROI override - always capture at full sensor, same as the overview
	// brightfield path, rather than cropping to a smaller measurement region.
	if (cameraType == "class uEyeCam" || cameraType == "class PointGrey") {
		m_settings.camera.readout.triggerMode = L"Software";
	}
#ifdef _DEBUG
	else if (cameraType == "class MockCamera") {
		m_settings.camera.readout.triggerMode = L"Software";
	}
#endif
	m_settings.camera.readout.cycleMode = L"Fixed";
	m_settings.camera.frameCount = 1;
}

template <typename T>
void Fluorescence::__acquire(std::unique_ptr <StorageWrapper>& storage, std::vector<ChannelSettings*> channels) {
	setAcquisitionStatus(ACQUISITION_STATUS::STARTED);

	// If the provided channel settings vector is empty, acquire the enabled channels.
	if (!channels.size()) {
		channels = getEnabledChannels();
		// If no channels are enabled, return.
		if (!channels.size()) {
			return;
		}
	}

	writeScaleCalibration(storage, ACQUISITION_MODE::FLUORESCENCE);

	QMetaObject::invokeMethod(
		storage.get(),
		[&storage = storage]() { storage.get()->startWritingQueues(); },
		Qt::AutoConnection
	);

	QElapsedTimer measurementTimer;
	measurementTimer.start();

	int rank_data{ 3 };
	// Loop through the different modes
	int imageNumber{ 0 };
	for (auto const& channel : channels) {
		// Abort if requested
		if (m_abort) {
			this->abortMode(storage);
			return;
		}

		// move to Fluorescence configuration
		if (m_scanControl) {
			m_scanControl->setPreset(channel->preset);
		}

		/*
		 * We have to check if the exposure time or gain settings will change.
		 * If they will, then we have to trash the first image after the change,
		 * as the settings might not be applied already (valid for trigger mode).
		 * Might be worse for free running mode.
		 * See https://www.ptgrey.com/KB/10086
		 */
		auto changed{ false };
		if (((int)(1e3 * m_settings.camera.exposureTime) != channel->exposure) || (m_settings.camera.gain != channel->gain)) {
			changed = true;
		}

		m_settings.camera.exposureTime = 1e-3 * channel->exposure;
		m_settings.camera.gain = channel->gain;

		if (changed) {
			m_settings.camera.frameCount = 2;
			if (m_camera) {
				m_camera->startAcquisition(m_settings.camera);
				// Settings might change after acquisition start (e.g. binning size and bytes per frame)
				m_settings.camera = m_camera->getSettings();
				m_camera->getImageForAcquisition(nullptr, false);
				m_camera->getImageForAcquisition(nullptr, false);
				m_camera->stopAcquisition();
			}
		}

		// start image acquisition
		m_settings.camera.frameCount = 1;
		if (m_camera) {
			m_camera->startAcquisition(m_settings.camera);
		}

		// Settings might change after acquisition start (e.g. binning size and bytes per frame)
		if (m_camera) {
			m_settings.camera = m_camera->getSettings();
		}
		hsize_t dims_data[3] = { 1, (hsize_t)m_settings.camera.roi.height_binned, (hsize_t)m_settings.camera.roi.width_binned };

		// read images from camera
		std::vector<std::byte> images(m_settings.camera.roi.bytesPerFrame);

		// acquire images
		if (m_camera) {
			m_camera->getImageForAcquisition(&images[0], true);
		}

		// cast the vector to unsigned short
		auto images_ = (std::vector<T> *) & images;

		// Sometimes the uEye camera returns a black image (only zeros), we try to catch this here by
		// repeating the acquisition a maximum of 5 times
		auto sum = simplemath::sum(*images_);
		int i{ 0 };
		while (sum == 0 && 5 > i++) {
			if (m_camera) {
				m_camera->getImageForAcquisition(&images[0], true);
			}

			// cast the vector type T
			images_ = (std::vector<T> *) & images;

			sum = simplemath::sum(*images_);
		}

		// store images
		// asynchronously write image to disk
		// the datetime has to be set here, otherwise it would be determined by the time the queue is processed
		std::string date = QDateTime::currentDateTime().toOffsetFromUtc(QDateTime::currentDateTime().offsetFromUtc())
			.toString(Qt::ISODateWithMs).toStdString();
		// Fluorescence mode has no grid/origin concept of its own (single-point capture, not
		// a scan) - the raw absolute stage position at capture time is both "the position the
		// scanner would normally image at" and the actual position. Still converted through
		// rawPositionToGridFrame() so this position lands in the same frame as the Brillouin
		// measurement grid's own positions-x/y/z - in absolute-grid mode that frame is offset
		// from raw absolute stage um by the grid's saved origin.
		const auto rawPosition = m_scanControl ? m_scanControl->getPosition() : POINT3{ 0, 0, 0 };
		const auto targetPosition = m_brillouinMode
			? m_brillouinMode->rawPositionToGridFrame(rawPosition)
			: rawPosition;
		// That grid frame is the one in force NOW: an image taken before a Brillouin run is converted with the origin set before Start
		// (absolute mode: whatever origin was saved then, relative mode: none), which need not be the frame of the run that follows. So
		// the image also carries what the overview images carry (see Brillouin::captureOverviewEvent()): the raw position and the origin
		// that was subtracted, the objective and its scale calibration, and the shift between the laser target and the middle of the
		// image - with these a reader places the image in any run's frame: middle of the image = stage_position_raw - image_centre_shift.
		std::vector<std::pair<std::string, double>> numbers;
		std::vector<std::pair<std::string, std::string>> strings;
		if (m_scanControl) {
			const auto stage = m_scanControl->getPosition(PositionType::STAGE);
			const auto scanner = m_scanControl->getPosition(PositionType::SCANNER);
			const auto scale = m_scanControl->getScaleCalibration();
			const auto objective = m_scanControl->getActiveObjectiveCalibration();
			const auto fov = m_scanControl->getActiveObjectiveFovOffsetUm();
			// A target puts the laser on a point (stage = target - scanner) and pixel p shows target - scanner + A * (p - origin) - FOV
			// offset; so the middle of the image is the target minus this shift (the same expression as the overview's centreShift
			// together with its fovCompensation).
			const auto dx = 0.5 * (double)m_settings.camera.roi.width_binned - scale.originPix.x;
			const auto dy = 0.5 * (double)m_settings.camera.roi.height_binned - scale.originPix.y;
			auto centreShift = POINT2{
				scanner.x - (dx * scale.pixToMicrometerX.x + dy * scale.pixToMicrometerY.x) + fov.x,
				scanner.y - (dx * scale.pixToMicrometerX.y + dy * scale.pixToMicrometerY.y) + fov.y
			};
			if (!std::isfinite(centreShift.x) || !std::isfinite(centreShift.y)) {
				centreShift = POINT2{ 0.0, 0.0 };
			}
			numbers = {
				{ "stage_position_raw_x_um", rawPosition.x }, { "stage_position_raw_y_um", rawPosition.y }, { "stage_position_raw_z_um", rawPosition.z },
				{ "stage_only_position_x_um", stage.x }, { "stage_only_position_y_um", stage.y },
				{ "scanner_position_x_um", scanner.x }, { "scanner_position_y_um", scanner.y },
				{ "grid_frame_origin_x_um", rawPosition.x - targetPosition.x }, { "grid_frame_origin_y_um", rawPosition.y - targetPosition.y },
				{ "grid_frame_origin_z_um", rawPosition.z - targetPosition.z },
				{ "fov_compensation_x_um", 0.0 }, { "fov_compensation_y_um", 0.0 },
				{ "image_centre_shift_x_um", centreShift.x }, { "image_centre_shift_y_um", centreShift.y },
				{ "objective_slot", (double)m_scanControl->getActiveObjectiveSlot() },
				{ "objective_magnification", objective.magnification },
				{ "objective_fov_offset_x_um", fov.x }, { "objective_fov_offset_y_um", fov.y },
				{ "scale_micrometer_to_pix_x_col", scale.micrometerToPixX.x }, { "scale_micrometer_to_pix_x_row", scale.micrometerToPixX.y },
				{ "scale_micrometer_to_pix_y_col", scale.micrometerToPixY.x }, { "scale_micrometer_to_pix_y_row", scale.micrometerToPixY.y },
				{ "scale_origin_col", scale.originPix.x }, { "scale_origin_row", scale.originPix.y }
			};
			strings = { { "objective_name", objective.objectiveName } };
		}
		auto img = new FLUOIMAGE<T>(
			imageNumber,
			rank_data,
			dims_data,
			date,
			channel->name,
			*images_,
			m_settings.camera.exposureTime,
			m_settings.camera.gain,
			m_settings.camera.roi,
			targetPosition,
			false, POINT3{ 0, 0, 0 },
			false,
			false, -1,
			numbers,
			strings
		);

		QMetaObject::invokeMethod(
			storage.get(),
			[&storage = storage, img]() { storage.get()->s_enqueuePayload(img); },
			Qt::AutoConnection
		);

		// configure camera for preview
		if (m_camera) {
			m_camera->stopAcquisition();
		}
		imageNumber++;
		double percentage = 100 * (double)imageNumber / channels.size();
		int remaining = 1e-3 * measurementTimer.elapsed() / imageNumber * ((int64_t)channels.size() - imageNumber);
		emit(s_repetitionProgress(percentage, remaining));
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

	setAcquisitionStatus(ACQUISITION_STATUS::FINISHED);
}

/*
 * Private slots
 */

void Fluorescence::acquire(std::unique_ptr <StorageWrapper>& storage) {
	acquire(storage, {});
}

void Fluorescence::acquire(std::unique_ptr<StorageWrapper>& storage, const std::vector<ChannelSettings*>& channels) {
	if (m_settings.camera.readout.dataType == "unsigned short") {
		__acquire<unsigned short>(storage, channels);
	} else if (m_settings.camera.readout.dataType == "unsigned char") {
		__acquire<unsigned char>(storage, channels);
	}
}
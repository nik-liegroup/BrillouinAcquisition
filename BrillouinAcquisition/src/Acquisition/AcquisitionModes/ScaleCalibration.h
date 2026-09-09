#ifndef SCALECALIBRATION_H
#define SCALECALIBRATION_H

#include <QtCore>
#include <gsl/gsl>
#include "H5Cpp.h"

#include "opencv2/imgproc.hpp"

#include "AcquisitionMode.h"
#include "ScaleCalibrationHelper.h"
#include "../../Devices/Cameras/Camera.h"
#include "../../Devices/ScanControls/ScanControl.h"

#include "ui_ScaleCalibrationDialog.h"

class ScaleCalibration : public AcquisitionMode {
	Q_OBJECT

public:
	ScaleCalibration(QObject* parent, Acquisition* acquisition, Camera*& camera, ScanControl*& scanControl);
	~ScaleCalibration();

public slots:
	void startRepetitions() override;

	void load(std::string filepath);

	// Scans `folder` (non-recursive) for "*.h5" calibration files, matches each one to a
	// nosepiece slot via its stored objectiveName/objectiveSlot fields (see
	// writeCalibrationMetadata()), and registers the unambiguous matches with the scanControl
	// (ScanControl::setObjectiveCalibration()) exactly as if the operator had stood at each
	// objective and clicked Load+Apply. Called at startup (see
	// BrillouinAcquisition::autoLoadObjectiveCalibrations()) once a calibrations folder has
	// been configured. A file is left unregistered - and reported back via
	// s_calibrationAutoLoadSummary()'s warning text - if: it does not parse as a calibration
	// file at all (readCalibrationFile() throws), it has no objectiveName, its objectiveSlot is
	// unset or is not a physically-possible slot on this backend (ScanControl::
	// isValidObjectiveSlot()), its objectiveName is shared by more than one file in the folder,
	// or its objectiveSlot is claimed by a different objectiveName's file. Does not touch
	// m_scaleCalibration or the dialog (unlike load()) - this is a background batch operation,
	// not something the operator is watching in the Scale Calibration dialog.
	void autoLoadCalibrationsFromFolder(std::string folder);

	// Writes the current calibration (scale + whatever objective-identity/FOV-offset fields
	// are set) to a new file, the same way the acquire-based procedure's save() does - but
	// without requiring an acquire to have run first (no images to write, no dependency on
	// m_cameraSettings ever having been populated this session). For entering a known
	// FOV-center offset by hand without re-running the translation measurement.
	void saveCalibration();

	void acquire(std::unique_ptr <StorageWrapper>& storage) override;
	void acquire();

	void initialize();

	void apply();

	void setTranslationDistanceX(double dx);
	void setTranslationDistanceY(double dy);

	void setMicrometerToPixX_x(double value);
	void setMicrometerToPixX_y(double value);
	void setMicrometerToPixY_x(double value);
	void setMicrometerToPixY_y(double value);

	void setPixToMicrometerX_x(double value);
	void setPixToMicrometerX_y(double value);
	void setPixToMicrometerY_x(double value);
	void setPixToMicrometerY_y(double value);

	// The objective-identity/FOV-offset fields on top of the existing scale calibration
	// (see ObjectiveCalibrationData) - set from the dialog's new "Objective" group, saved/
	// loaded alongside the rest of this calibration file, and registered against whichever
	// objective slot is active when apply() runs (not a separately-typed slot number - the
	// operator is expected to physically/software-switch to the objective being calibrated
	// first, exactly like the existing acquire-based procedure already implicitly assumes).
	void setObjectiveName(QString name);
	void setMagnification(double value);
	void setReferenceObjectiveName(QString name);
	void setHasFovOffset(bool hasFovOffset);
	void setFovOffsetX(double value);
	void setFovOffsetY(double value);
	void setFovOffsetSigma(double value);

	// FOV-center offset auto-measurement. Two-step, operator-driven (this never commands an
	// objective switch itself - see the class-level comment on measureFovOffset() for why):
	// 1) At the reference objective, call setFovOffsetReference() to capture and cache an
	//    image, without moving the stage afterward.
	// 2) Switch to the objective being calibrated (any means - GUI button or the microscope's
	//    own panel), without moving the stage, and call measureFovOffset(). Repeating step 2
	//    (switch away to any other objective and back, then measure again) accumulates more
	//    samples and tightens fovOffsetSigmaUm; switching to a different target objective
	//    between calls discards the previous target's samples automatically.
	void setFovOffsetReference();
	void measureFovOffset();

private:
	void abortMode(std::unique_ptr <StorageWrapper>& storage) override;
	void abortMode();

	template <typename T>
	void save(std::vector<std::vector<T>> images, std::vector<POINT2> positions);

	// Shared by save() and saveCalibration() so both write exactly the same calibration
	// fields (date, scale calibration, objective-identity/FOV-offset) - keeping this in one
	// place instead of two independent copies is what avoids the two ever drifting apart.
	void writeCalibrationMetadata(H5::Group& root);
	// Folder (from m_acquisition->getCurrentFolder()) + timestamped filename, shared by
	// save() and saveCalibration().
	std::string newCalibrationFilePath() const;

	// Reads one calibration file's fields into *out (scale calibration + objective-identity/
	// FOV-offset + objectiveSlot), without touching m_scaleCalibration or emitting any of the
	// dialog-refresh signals load() does - shared by load() (which then does both) and
	// autoLoadCalibrationsFromFolder() (which does neither, since no single file is "the one
	// being edited"). Throws H5::Exception if filepath is not a readable HDF5 file or lacks the
	// required scale-calibration datasets (origin/pixToMicrometerX/Y/micrometerToPixX/Y) - the
	// caller decides what "invalid calibration file" means for its own context.
	void readCalibrationFile(const std::string& filepath, ObjectiveCalibrationData* out);

	void writePoint(H5::Group group, std::string name, POINT2 point);
	POINT2 readPoint(H5::Group group, const std::string& name);

	void writeAttribute(H5::H5Object& parent, std::string name, double value);
	void writeAttribute(H5::H5Object& parent, std::string name, std::string value);
	void readAttribute(const H5::H5Object& parent, std::string name, double* value);
	// Missing attribute (older calibration file, saved before the objective fields existed)
	// leaves *value untouched rather than throwing - callers pre-set it to the desired
	// default before calling.
	void readAttributeOptional(const H5::H5Object& parent, std::string name, double* value);
	void readAttributeOptional(const H5::H5Object& parent, std::string name, std::string* value);

	template <typename T>
	void __acquire();

	// Sets m_cameraSettings to the standard calibration crop (1000x1000 at (1000,800), one
	// frame) and applies it to m_camera. Shared by startRepetitions() (scale-calibration
	// acquire) and captureBrightfieldImageForFovOffset(), which previously each had their own
	// copy of this block.
	void configureCalibrationCameraRoi();

	// Captures a single brightfield frame at the standard calibration ROI (same crop
	// startRepetitions() configures for the scale-calibration procedure). Buffer type matches
	// Camera::getImageForAcquisition()'s std::byte* requirement. Pixel depth is whatever
	// m_cameraSettings.readout.dataType reports after the capture (the caller reads that
	// right after calling this, same way __acquire() already tracks it) -
	// computeFovOffsetShiftUm() below interprets the buffer accordingly, it is not assumed
	// to be 8-bit. Empty on failure (no camera).
	std::vector<std::byte> captureBrightfieldImageForFovOffset();

	// The actual rescale-then-template-match: brings referenceImage/targetImage to a common
	// approximate pixel scale (using each side's own calibration's approximate isotropic
	// pixel pitch - shear/rotation are not corrected for, only used to get the two images'
	// scale roughly aligned so matchTemplate has a chance), locates the best match, and
	// converts the resulting pixel shift to micrometers using the target's full (exact,
	// non-approximated) calibration. referenceDataType/targetDataType are each image's
	// CAMERA_SETTINGS.readout.dataType ("unsigned char" or "unsigned short") at capture time -
	// a 16-bit source is read at its real depth then downscaled to 8-bit before matching (see
	// the .cpp definition), matching the same fix applied to __acquire(). Returns false
	// (leaves *shiftUm untouched) if either image is empty, either calibration is degenerate,
	// or no image fits inside the other after rescaling.
	bool computeFovOffsetShiftUm(
		const std::vector<std::byte>& referenceImage, const CAMERA_ROI& referenceRoi, const ScaleCalibrationData& referenceScale, const std::string& referenceDataType,
		const std::vector<std::byte>& targetImage, const CAMERA_ROI& targetRoi, const ScaleCalibrationData& targetScale, const std::string& targetDataType,
		POINT2* shiftUm
	);

	// Wraps a captured buffer at its real depth (dataType: "unsigned short" -> 16-bit,
	// anything else -> 8-bit, matching CAMERA_SETTINGS.readout.dataType's convention
	// elsewhere in this class) and returns an 8-bit view of it - a 16-bit source is
	// downscaled via convertTo(), an 8-bit source is returned as a direct (copy-free) view
	// onto `image`'s own memory, same as the pre-existing code this mirrors (see
	// __acquire()'s image-matrix construction).
	cv::Mat readAsMat8U(const std::vector<std::byte>& image, int rows, int cols, const std::string& dataType) const;

	CAMERA_SETTINGS m_cameraSettings;
	Camera*& m_camera;
	POINT3 m_startPosition{ 0, 0, 0 };

	// FOV-offset reference/measurement state (see setFovOffsetReference()/measureFovOffset()).
	std::vector<std::byte> m_fovReferenceImage;
	CAMERA_ROI m_fovReferenceRoi{};
	std::string m_fovReferenceDataType;
	ScaleCalibrationData m_fovReferenceScaleCalibration{};
	std::string m_fovReferenceObjectiveName;
	int m_fovReferenceObjectiveSlot{ -1 };

	// Which objective slot the currently-accumulating m_fovOffsetSamplesUm belong to - reset
	// (cleared) automatically whenever measureFovOffset() sees a different active slot than
	// this, so samples from two different target objectives never get averaged together.
	int m_fovOffsetTargetSlot{ -1 };
	std::vector<POINT2> m_fovOffsetSamplesUm;

	// ObjectiveCalibrationData IS-A ScaleCalibrationData, so every existing use of
	// m_scaleCalibration.<scale calibration field> is unaffected by this - only the objective-
	// identity/FOV-offset fields declared on top of it are new.
	ObjectiveCalibrationData m_scaleCalibration;
	POINT2 m_Ds{ 10.0, 10.0 };	// [µm]	shift in x- and y-direction

signals:
	void s_Ds_changed(POINT2);
	void s_scaleCalibrationChanged(ScaleCalibrationData);
	// Carries the objective-identity/FOV-offset fields s_scaleCalibrationChanged's
	// ScaleCalibrationData parameter slices away - the dialog's new "Objective" group binds
	// to this one instead.
	void s_objectiveCalibrationChanged(ObjectiveCalibrationData);
	void s_scaleCalibrationAcquisitionProgress(double);
	void s_scaleCalibrationStatus(std::string title, std::string message);
	void s_closeScaleCalibrationDialog();
	// Result of autoLoadCalibrationsFromFolder(). appliedText lists what got registered
	// (objective name -> slot -> file), one per line, empty if nothing matched cleanly.
	// warningText lists everything that did NOT get auto-applied and why (ambiguous name,
	// slot conflict, invalid/non-objective file), empty if there was nothing to flag - the
	// receiver only needs to show a blocking warning when this is non-empty.
	void s_calibrationAutoLoadSummary(std::string appliedText, std::string warningText);
};

#endif //SCALECALIBRATION_H

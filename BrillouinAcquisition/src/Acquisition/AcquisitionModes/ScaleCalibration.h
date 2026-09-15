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

	// Reads one calibration file and registers it directly against `slot`, regardless of what
	// the file's own saved objectiveName/objectiveSlot say - this is an explicit, operator-
	// chosen link (see BrillouinAcquisition's "Objective Setup" dialog, which is what calls
	// this - once per named-and-linked slot, both at startup and right after the operator
	// picks/changes a file there). Backfills objectiveName/magnification from Objective Setup's
	// own name for this slot (unconditionally, not just when the file lacks them) - Objective
	// Setup is the single authoritative source of a slot's identity now, so this is also how a
	// legacy calibration file (saved before these fields existed, or under a name that has since
	// changed) gets "extended"/kept in sync automatically, with no separate migration step.
	// Registers straight into ScanControl::setObjectiveCalibration(), which itself applies it
	// live only if `slot` happens to already be the active one, and unconditionally re-applies
	// it on every future switch to that slot (ScanControl::handleObjectiveSlotObserved()). Emits
	// s_scaleCalibrationStatus() on failure (unreadable/invalid file, or slot not physically
	// possible on this backend) rather than throwing, since the caller is a startup/background
	// path, not something with its own try/catch around every call.
	void loadCalibrationForSlot(int slot, std::string filepath, std::string objectiveName, double magnification);

	// Writes a brand-new, blank calibration file (identity/zero scale calibration, no FOV
	// offset) for a slot that has no calibration file yet, and registers it exactly like
	// loadCalibrationForSlot() would - so "New" in Objective Setup behaves like picking a real
	// (if not yet actually calibrated) file from that point on: Save later overwrites the
	// same path once a real scale calibration/FOV offset has been measured for it.
	void createEmptyCalibrationFile(int slot, std::string objectiveName, double magnification, std::string filepath);

	// Writes `data` verbatim to `slot`'s live registration and, if `filepath` is non-empty, to
	// that file on disk (H5F_ACC_TRUNC, the same schema writeCalibrationMetadata() uses) - a
	// slot- and file-explicit primitive, unlike the rest of this class's save path (persistPartial()/
	// saveScaleCalibration()/saveFovOffsetCalibration()), which always targets whichever slot/file
	// the dialog currently has open (the physically active objective). Used by
	// BrillouinAcquisition's "Reference" checkbox handling (Objective Setup dialog) to promote/
	// demote a slot's isReferenceObjective flag without requiring the operator to physically
	// switch to it first - both the newly-promoted and any newly-demoted slot need writing, and
	// at most one of the two can be the physically active one. Emits s_scaleCalibrationStatus()
	// (does not throw) on a write failure; a blank filepath is tolerated silently (live
	// registration still happens - same "not linked to a file yet" tolerance
	// writeLinkedCalibrationFile() has for the active slot).
	void writeCalibrationToSlot(int slot, std::string filepath, ObjectiveCalibrationData data);

	// The active slot's linked calibration file path (BrillouinAcquisition::
	// m_objectiveSlotCalibrationPaths, pushed in by refreshScaleCalibrationObjectiveDisplay()
	// whenever the dialog opens or the active slot changes) - what saveScaleCalibration()/
	// saveFovOffsetCalibration() write into, so editing/measuring a calibration actually
	// persists into the same file Objective Setup links for this slot, instead of only updating
	// the in-memory registration. "" if this slot has no linked file yet.
	void setLinkedCalibrationFilePath(std::string path);

	// The two "Save" buttons (one in each box) are deliberately independent - each commits only
	// its own half of the calibration, starting from whatever is currently stored for the active
	// slot (already registered in ScanControl / on disk) and overwriting only the fields it owns.
	// This matters because m_scaleCalibration is one shared live edit buffer for the whole
	// dialog: without this split, clicking either button would persist BOTH halves together,
	// silently committing an in-progress, not-yet-decided edit sitting in the *other* box (e.g.
	// clicking "Save calibration" in the FOV box right after running an automated Scale cycle you
	// hadn't decided to keep yet would have saved that new scale calibration too). Both also
	// re-read the merged, actually-saved result back into m_scaleCalibration and emit change
	// signals, so the dialog visually reverts any abandoned edit in the box that was NOT saved.

	// Persists only the scale-calibration (pix<->um) fields into the active slot's linked file
	// and live registration - leaves whatever FOV-offset fields are currently stored untouched.
	void saveScaleCalibration();
	// Persists only the FOV-offset fields into the active slot's linked file and live
	// registration - leaves whatever scale-calibration fields are currently stored untouched.
	// Tolerant of a still-degenerate (not yet "Acquire"d) scale calibration, since it never
	// touches that half at all.
	void saveFovOffsetCalibration();

	void acquire(std::unique_ptr <StorageWrapper>& storage) override;
	void acquire();

	void initialize();

	// Runs startRepetitions() (the existing single-shot "translation between images" procedure)
	// `cycles` times back-to-back, then averages every cycle's resulting pix<->um matrix
	// (component-wise mean) into m_scaleCalibration and stores the repeatability (std dev of the
	// isotropic pixel pitch across cycles, see ScaleCalibrationHelper::isotropicPixelPitchUm())
	// as scaleCalibrationSigmaUm. Unlike the FOV-offset automated cycle, this needs no objective
	// switch/operator refocus between repetitions - it is fully autonomous, cycles run in one
	// call with no pause. A cycle whose match fails is skipped (not averaged in); if every cycle
	// fails, m_scaleCalibration is left untouched and s_scaleCalibrationStatus() explains why.
	// Aborted early (m_abort, same flag/convention as every other acquisition mode) still
	// averages whatever cycles completed before the abort.
	void startScaleCalibrationCycle(int cycles);

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
	// (see ObjectiveCalibrationData) - name/magnification are now set programmatically (the
	// dialog shows them read-only, driven by Objective Setup's own name for the active slot,
	// not free-text editable here), and referenceObjectiveName is set internally by
	// measureFovOffset() - neither has a dedicated GUI input anymore. Saved/loaded alongside the
	// rest of this calibration file, and registered against whichever objective slot is active
	// when saveFovOffsetCalibration()/saveScaleCalibration() run.
	void setObjectiveName(QString name);
	void setMagnification(double value);
	void setHasFovOffset(bool hasFovOffset);
	void setFovOffsetX(double value);
	void setFovOffsetY(double value);
	void setFovOffsetSigma(double value);

	// FOV-center offset auto-measurement, driven only by the automated multi-cycle run now (see
	// startObjectiveCycleCalibration() below) - captureFovOffsetReferenceImage() (private) takes
	// the reference image, measureFovOffset() (below) computes the shift once the target
	// objective is active, composing it through the reference objective's own already-stored,
	// baseline-relative offset (see the .cpp definition) so results stay correct no matter which
	// already-calibrated objective was used as the reference for this particular run. Repeated
	// calls (successive cycles of the same run) accumulate more samples and tighten
	// fovOffsetSigmaUm; a different target objective than the previous call discards the
	// previous target's samples automatically.
	void measureFovOffset();

	// Runs the flow above, between two already-named, distinct nosepiece slots (see
	// BrillouinAcquisition's "Objective Setup" dialog/m_objectiveSlotNames for where names come
	// from - this class only ever deals in slot numbers). Unlike every other capture path in
	// this class, this one *does* command objective switches itself
	// (switchToObjectiveSlotAndVerify(), via ScanControl::setElement()) - a deliberate, confirmed
	// exception to "the operator drives every switch", made because doing this by hand for M
	// repeated cycles is exactly the kind of tedious, error-prone repetition automation is for.
	// Each cycle pauses TWICE (returns to the caller/event loop - see beginCycleReferencePhase()/
	// beginCycleTargetPhase()), once per objective, since the two objectives generally are not
	// perfectly parfocal with each other and Z is deliberately never auto-restored (see the .cpp
	// comment on beginCycleReferencePhase() for why): retract Z, switch to referenceSlot, PAUSE
	// for the operator to refocus at the reference objective; on Continue, capture a fresh
	// reference image, retract Z again, switch to targetSlot, PAUSE again for the operator to
	// refocus at the target objective; on Continue, measure (via measureFovOffset(),
	// accumulating into the same running mean/sigma exactly as repeated manual clicks would),
	// and either start the next cycle's reference phase or finish. No-op (emits a status and
	// returns) if a run is already in progress.
	void startObjectiveCycleCalibration(int referenceSlot, int targetSlot, int cycles, double zRetractDistanceUm);
	// Resumes from whichever of the two pauses above is currently active - after the operator has
	// refocused at the reference objective (captures the reference image, then proceeds to the
	// target phase), or after they have refocused at the target objective (measures, then starts
	// the next cycle or finishes). No-op if not currently paused.
	void continueObjectiveCycle();
	// Aborts a run in progress or a paused run. Safe to call at any time (no-op if idle).
	void abortObjectiveCycle();

private:
	void abortMode(std::unique_ptr <StorageWrapper>& storage) override;
	void abortMode();

	template <typename T>
	void save(std::vector<std::vector<T>> images, std::vector<POINT2> positions);

	// Shared by save() and writeLinkedCalibrationFile() so both write exactly the same
	// calibration fields (date, scale calibration, objective-identity/FOV-offset) - keeping this
	// in one place instead of two independent copies is what avoids the two ever drifting apart.
	void writeCalibrationMetadata(H5::Group& root);
	// Folder (from m_acquisition->getCurrentFolder()) + timestamped filename, used by save()
	// for the raw-image diagnostic dump the acquire-based procedure writes.
	std::string newCalibrationFilePath() const;
	// Overwrites m_linkedCalibrationFilePath with the current m_scaleCalibration (via
	// writeCalibrationMetadata()) - called by persistPartial() once it has merged in only the
	// half of the calibration the caller (saveScaleCalibration()/saveFovOffsetCalibration())
	// actually owns. Emits s_scaleCalibrationStatus() instead of writing if no file is linked
	// for this slot, or if the linked file is not writable.
	void writeLinkedCalibrationFile();
	// Shared by saveScaleCalibration()/saveFovOffsetCalibration(): reads the active slot's
	// currently-stored calibration (m_scanControl->getObjectiveCalibration()) as the baseline,
	// overlays only the scale-calibration fields (includeScale) and/or FOV-offset fields
	// (includeFov) from the live edit buffer (m_scaleCalibration) on top of it, registers the
	// merged result live, writes it to the linked file, and re-reads it back into
	// m_scaleCalibration (emitting the usual change signals) so the dialog reflects exactly what
	// was actually saved - including reverting any abandoned edit in the half that was NOT
	// included. objectiveName/magnification are always carried over from the live edit buffer
	// (identity fields, not "owned" by either half - always kept correct by
	// refreshScaleCalibrationObjectiveDisplay(), so always safe to include).
	void persistPartial(bool includeScale, bool includeFov);

	// Reads one calibration file's fields into *out (scale calibration + objective-identity/
	// FOV-offset + objectiveSlot), without touching m_scaleCalibration or emitting any dialog-
	// refresh signals - used by loadCalibrationForSlot(). Throws H5::Exception if filepath is
	// not a readable HDF5 file, lacks the scale-calibration datasets (origin/pixToMicrometerX/Y/
	// micrometerToPixX/Y), or lacks any of the objective-identity/FOV-offset attributes
	// (objectiveName/magnification/hasFovOffset/fovOffsetX/Y/Sigma/referenceObjectiveName/
	// calibrationDate/objectiveSlot) - only "new"-format files (written by writeCalibrationMetadata()/
	// createEmptyCalibrationFile(), i.e. by this version of the software) are accepted; an older
	// legacy file missing these is rejected rather than silently backfilled with defaults. The
	// caller decides what "invalid calibration file" means for its own context.
	void readCalibrationFile(const std::string& filepath, ObjectiveCalibrationData* out);

	void writePoint(H5::Group group, std::string name, POINT2 point);
	POINT2 readPoint(H5::Group group, const std::string& name);

	void writeAttribute(H5::H5Object& parent, std::string name, double value);
	void writeAttribute(H5::H5Object& parent, std::string name, std::string value);
	// Both throw (H5::Exception) if the attribute is missing - readCalibrationFile() relies on
	// this to reject a file missing any of the "new"-format objective-identity/FOV-offset
	// attributes.
	void readAttribute(const H5::H5Object& parent, std::string name, double* value);
	void readAttribute(const H5::H5Object& parent, std::string name, std::string* value);

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
	// approximate pixel scale (using each side's own calibration's isotropic-equivalent pixel
	// pitch - sqrt(|determinant|) of its pix->um matrix, which is correct regardless of any
	// rotation between the camera's pixel axes and the stage axes, unlike averaging the
	// matrix's diagonal terms; shear beyond a pure rotation is still not corrected for, this is
	// only used to get the two images' scale roughly aligned so matchTemplate has a chance),
	// locates the best match, and
	// converts the resulting pixel shift to micrometers using the target's full (exact,
	// non-approximated) calibration. referenceDataType/targetDataType are each image's
	// CAMERA_SETTINGS.readout.dataType ("unsigned char" or "unsigned short") at capture time -
	// a 16-bit source is read at its real depth then downscaled to 8-bit before matching (see
	// the .cpp definition), matching the same fix applied to __acquire(). Returns false
	// (leaves *shiftUm and *estimatedMagnificationChange untouched) if either image is empty,
	// either calibration is degenerate, or no image fits inside the other after rescaling -
	// *failureReason is set to a specific, human-readable explanation of which of those it was
	// (none of them are actually about image content/matching - this function never rejects a
	// match based on how good it looks, see the .cpp definition - so do not describe any of
	// them to the operator as "no distinct structures", which was misleading every previous
	// caller of this).
	// *estimatedMagnificationChange is the reference->target pixel-scale rescale factor this
	// function actually used to bring the two images to a common scale before matching -
	// exposed so the caller can sanity-check it against the nominal magnification ratio typed
	// into the two objectives' "Magnification" fields (see measureFovOffset()). A large
	// disagreement between the two means the pixel-scale calibration for one of the objectives
	// (not the FOV-offset measurement itself) is the thing to re-check.
	bool computeFovOffsetShiftUm(
		const std::vector<std::byte>& referenceImage, const CAMERA_ROI& referenceRoi, const ScaleCalibrationData& referenceScale, const std::string& referenceDataType,
		const std::vector<std::byte>& targetImage, const CAMERA_ROI& targetRoi, const ScaleCalibrationData& targetScale, const std::string& targetDataType,
		POINT2* shiftUm, double* estimatedMagnificationChange, std::string* failureReason
	);

	// Wraps a captured buffer at its real depth (dataType: "unsigned short" -> 16-bit,
	// anything else -> 8-bit, matching CAMERA_SETTINGS.readout.dataType's convention
	// elsewhere in this class) and returns an 8-bit view of it - a 16-bit source is
	// downscaled via convertTo(), an 8-bit source is returned as a direct (copy-free) view
	// onto `image`'s own memory, same as the pre-existing code this mirrors (see
	// __acquire()'s image-matrix construction).
	cv::Mat readAsMat8U(const std::vector<std::byte>& image, int rows, int cols, const std::string& dataType) const;

	// Writes a plain, human-viewable .tif copy of a just-captured FOV-offset reference/target
	// image (same 8-bit conversion readAsMat8U() gives the actual matching code, so this is
	// exactly what the algorithm saw) next to the active slot's linked calibration file
	// (m_linkedCalibrationFilePath's own folder), or m_acquisition->getCurrentFolder() if no
	// file is linked yet. Filenames are "<label>_<timestamp>.tif", timestamped so repeated
	// cycles/measurements never overwrite each other. Best-effort/silent on failure (missing
	// folder, not writable, etc.) - purely a debugging aid, never blocks the actual measurement.
	void saveDebugCalibrationImage(const std::vector<std::byte>& image, const CAMERA_ROI& roi, const std::string& dataType, const std::string& label);

	// Shared folder-resolution logic behind saveDebugCalibrationImage()/saveDebugCalibrationMat()
	// - see saveDebugCalibrationImage()'s own doc comment for what folder this resolves to.
	// Returns an empty string (nothing should be written) if no folder is available yet, or it
	// could not be created.
	std::string debugCalibrationFolder() const;

	// Same as saveDebugCalibrationImage(), but for a cv::Mat already in memory (e.g. a
	// constructed debug visualization) rather than a just-captured raw image buffer.
	void saveDebugCalibrationMat(const cv::Mat& mat, const std::string& label);

	// Captures and caches the reference image (see measureFovOffset()'s doc comment above) -
	// called by continueObjectiveCycle() with resetAccumulatedSamples=(cycle == 1), so a fresh
	// reference image is captured every cycle without also wiping m_fovOffsetSamplesUm each
	// time, which would defeat "M cycles average into one mean/sigma".
	void captureFovOffsetReferenceImage(bool resetAccumulatedSamples);

	// Automated multi-cycle FOV-offset calibration state (see startObjectiveCycleCalibration()
	// in the public section above for the overall flow). WaitingForReferenceFocus/
	// WaitingForTargetFocus are the two pause points per cycle - kept distinct (rather than one
	// shared "WaitingForContinue") so continueObjectiveCycle() knows which half of the cycle to
	// resume: capture the reference image and move on to the target phase, or measure and move
	// on to the next cycle/finish.
	enum class ObjectiveCycleState { Idle, Running, WaitingForReferenceFocus, WaitingForTargetFocus };
	ObjectiveCycleState m_objectiveCycleState{ ObjectiveCycleState::Idle };
	int m_objectiveCycleReferenceSlot{ -1 };
	int m_objectiveCycleTargetSlot{ -1 };
	int m_objectiveCycleCount{ 0 };	// requested M
	int m_objectiveCycleIndex{ 0 };	// 1-based current cycle
	double m_objectiveCycleRetractUm{ 0.0 };

	// Retracts Z, switches to the reference objective, then pauses (sets state to
	// WaitingForReferenceFocus and returns to the caller/event loop - see the .cpp definition for
	// why this return is the only pause point, never a blocking wait on this object's own thread)
	// for the operator to refocus there before the reference image is captured. Called for cycle
	// 1 from startObjectiveCycleCalibration(), and for cycles 2..M from continueObjectiveCycle()
	// once the previous cycle's measurement is done.
	void beginCycleReferencePhase();
	// Captures the reference image (called from continueObjectiveCycle() once the operator has
	// refocused at the reference objective - see beginCycleReferencePhase()), then retracts Z,
	// switches to the target objective, and pauses again (WaitingForTargetFocus) for the operator
	// to refocus there before measureFovOffset() runs.
	void beginCycleTargetPhase();
	// Finds the "Objective" DeviceElement the same way the beampath buttons do
	// (m_scanControl->m_deviceElements, matched by name). Returns false (leaves *out untouched)
	// if this backend has none.
	bool findObjectiveElement(DeviceElement* out) const;
	// Commands the switch (m_scanControl->setElement(), a plain same-thread synchronous call -
	// see the .cpp definition for why this is safe/non-racy) and verifies
	// getActiveObjectiveSlot() == slot afterward. Emits a failure status and returns false on a
	// missing "Objective" element or a mismatch (the changer not actually landing on the
	// requested slot).
	bool switchToObjectiveSlotAndVerify(int slot);
	// Ends a run (cycles exhausted, or aborted): resets state to Idle, emits a final status and
	// s_objectiveCycleProgress(0, ...). Deliberately does not call saveFovOffsetCalibration()
	// itself - persisting the result stays an explicit, operator-clicked step (the dialog's
	// existing Save button already reads the up-to-date m_scaleCalibration this leaves behind,
	// exactly as it does after a manual measureFovOffset() click).
	void finishObjectiveCycle(bool aborted);

	CAMERA_SETTINGS m_cameraSettings;
	Camera*& m_camera;
	POINT3 m_startPosition{ 0, 0, 0 };

	// FOV-offset reference/measurement state (see captureFovOffsetReferenceImage()/measureFovOffset()).
	std::vector<std::byte> m_fovReferenceImage;
	CAMERA_ROI m_fovReferenceRoi{};
	std::string m_fovReferenceDataType;
	ScaleCalibrationData m_fovReferenceScaleCalibration{};
	std::string m_fovReferenceObjectiveName;
	int m_fovReferenceObjectiveSlot{ -1 };
	// Magnification registered for the reference objective (ScanControl's saved calibration,
	// not the dialog's possibly-since-edited-and-unapplied buffer) at the moment
	// captureFovOffsetReferenceImage() was called - used only for the nominal-magnification-ratio
	// sanity check in measureFovOffset(). 0 if unset.
	double m_fovReferenceMagnification{ 0.0 };

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

	// Set by __acquire() - false at the start of every startRepetitions() call, true only once
	// its match has actually succeeded and m_scaleCalibration's pix<->um fields have been
	// updated for that call. startScaleCalibrationCycle() reads this right after each
	// startRepetitions() call to know whether that cycle's result is safe to average in.
	bool m_lastAcquireSucceeded{ false };

	// See setLinkedCalibrationFilePath()/writeLinkedCalibrationFile().
	std::string m_linkedCalibrationFilePath;

signals:
	void s_Ds_changed(POINT2);
	void s_scaleCalibrationChanged(ScaleCalibrationData);
	// Carries the objective-identity/FOV-offset fields s_scaleCalibrationChanged's
	// ScaleCalibrationData parameter slices away - the dialog's new "Objective" group binds
	// to this one instead.
	void s_objectiveCalibrationChanged(ObjectiveCalibrationData);
	void s_scaleCalibrationAcquisitionProgress(double);
	void s_scaleCalibrationStatus(std::string title, std::string message);
	// Automated multi-cycle FOV-offset calibration progress. currentCycle is 1-based, 0 while
	// idle/just finished (see finishObjectiveCycle()). waitingForContinue is true for EITHER of
	// the two pauses per cycle (WaitingForReferenceFocus or WaitingForTargetFocus) - it only
	// drives the dialog's Start/Continue/Abort button enablement, which of the two pauses is
	// currently active does not need extra plumbing here since the operator sees the specific
	// "refocus at reference/target objective" instruction via s_scaleCalibrationStatus instead.
	void s_objectiveCycleProgress(int currentCycle, int totalCycles, bool waitingForContinue);
	// Automated multi-cycle scale-calibration progress (startScaleCalibrationCycle()).
	// currentCycle is 1-based, 0 while idle/just finished - no "waiting for continue" state,
	// this run is fully autonomous.
	void s_scaleCalibrationCycleProgress(int currentCycle, int totalCycles);
	// Emitted by persistPartial() whenever saveFovOffsetCalibration() (or saveScaleCalibration()
	// if the FOV half happened to be included too - not currently possible from the GUI, but
	// persistPartial() doesn't otherwise know that) commits new FOV-offset fields for the active
	// slot. `slot` is always ScanControl::getActiveObjectiveSlot() at save time - this dialog only
	// ever edits/saves against whichever objective is currently active, never a different one.
	// old*/new* let a listener (BrillouinAcquisition::onFovOffsetSaved()) apply the exact same
	// "shift the not-yet-visited relative-mode grid by the delta, force an absolute-mode redraw"
	// treatment ScanControl::s_objectiveSwitched() already gets on an actual objective switch -
	// a plain Save while staying on the same objective triggers neither on its own.
	void s_fovOffsetSaved(int slot, POINT2 oldOffsetUm, bool oldHasFovOffset, POINT2 newOffsetUm, bool newHasFovOffset);
};

#endif //SCALECALIBRATION_H

#ifndef SCANCONTROL_H
#define SCANCONTROL_H

#include "../Device.h"
#include "../../lib/TypesafeBitmask.h"
#include "../../lib/math/points.h"
#include "../../Acquisition/AcquisitionModes/ScaleCalibrationHelper.h"

#include <map>

enum class ScanPreset {
	SCAN_NULL			= 0x0,
	SCAN_BRIGHTFIELD	= 0x2,
	SCAN_CALIBRATION	= 0x4,
	SCAN_BRILLOUIN		= 0x8,
	SCAN_EYEPIECE		= 0x10,
	SCAN_ODT			= 0x20,
	SCAN_EPIFLUOOFF		= 0x40,
	SCAN_EPIFLUOBLUE	= 0x80,
	SCAN_EPIFLUOGREEN	= 0x100,
	SCAN_EPIFLUORED		= 0x200,
	SCAN_LASEROFF		= 0x800,
	SCAN_CONFOCAL		= 0x1000
};
ENABLE_BITMASK_OPERATORS(ScanPreset)

struct BOUNDS {
	double xMin{ -1e3 };	// [um] minimal x-value
	double xMax{ 1e3 };		// [um] maximal x-value
	double yMin{ -1e3 };	// [um] minimal y-value
	double yMax{ 1e3 };		// [um] maximal y-value
	double zMin{ -1e3 };	// [um] minimal z-value
	double zMax{ 1e3 };		// [um] maximal z-value
};

enum class Capabilities {
	ODT,
	TranslationStage,
	ScaleCalibration,
	VoltageCalibration,
	LaserScanner,
	COUNT
};

enum class PositionType {
	BOTH,
	STAGE,
	SCANNER
};

typedef enum class enDeviceInput {
	PUSHBUTTON,
	INTBOX,
	DOUBLEBOX,
	SLIDER
} DEVICE_INPUT_TYPE;

class DeviceElement {
public:
	DeviceElement() {};
	explicit DeviceElement(const std::string& name, int maxOptions, int index) :
		name(name), maxOptions(maxOptions), index(index), optionNames(checkNames(maxOptions)) {};
	DeviceElement(const std::string& name, int maxOptions, int index, DEVICE_INPUT_TYPE inputType) :
		name(name), maxOptions(maxOptions), index(index), optionNames(checkNames(maxOptions)), inputType(inputType) {};
	DeviceElement(const std::string& name, int maxOptions, int index, const std::vector<std::string>& optionNames) :
		name(name), maxOptions(maxOptions), index(index), optionNames(checkNames(maxOptions, optionNames)) {};
	DeviceElement(const std::string& name, int maxOptions, int index, const std::vector<std::string>& optionNames, DEVICE_INPUT_TYPE inputType) :
		name(name), maxOptions(maxOptions), index(index), optionNames(checkNames(maxOptions, optionNames)), inputType(inputType) {};

	std::string name{ "" };
	int maxOptions{ 0 };
	int index{ 0 };
	std::vector<std::string> optionNames;
	DEVICE_INPUT_TYPE inputType{ enDeviceInput::PUSHBUTTON }; // possible types are "pushButtons", "intBox" or "doubleBox"

	std::vector<std::string> checkNames(int count, std::vector<std::string> names = {}) {
		//check that the number of names fits the number of options
		while (names.size() < count) {
			names.push_back(std::to_string((int)names.size() + 1));
		}
		return names;
	}
};

class DeviceElements {

public:
	DeviceElements() {};
	explicit DeviceElements(const std::vector<DeviceElement>& deviceElements) {
		m_deviceElements = deviceElements;
	};
	void operator=(const std::vector<DeviceElement>& deviceElements) {
		m_deviceElements = deviceElements;
	}

	std::vector<DeviceElement> getDeviceElements() {
		return m_deviceElements;
	};

	int count() {
		return m_deviceElements.size();
	};

private:
	std::vector<DeviceElement> m_deviceElements;
};

class Preset {
public:
	Preset(const std::string& name, ScanPreset index, const std::vector<std::vector<double>>& positions) :
		name(name), index(index), elementPositions(positions) {};

	std::string name{ "" };
	ScanPreset index{ ScanPreset::SCAN_NULL };
	std::vector<std::vector<double>> elementPositions;
};

class ScanControl: public Device {
	Q_OBJECT

public:
	ScanControl() noexcept;
	virtual ~ScanControl() {};

	bool getConnectionStatus();

	virtual void setPosition(POINT2 position) = 0;
	virtual void setPosition(POINT3 position) = 0;
	// moves the position relative to current position
	virtual void movePosition(POINT2 distance);
	virtual void movePosition(const POINT3& distance);
	virtual POINT3 getPosition(PositionType positionType = PositionType::BOTH);
	// Moves to the target position, always approaching from lower x/y values.
	// Mechanical translation stages exhibit backlash/hysteresis, so approaching
	// a target from an inconsistent direction leads to inconsistent positioning
	// (e.g. a grid point is not reached reproducibly). This mirrors the approach
	// already used for scale calibration moves.
	void setPositionCompensated(POINT3 position);
	// Same idea as setPositionCompensated(), but for a relative move (used for
	// click-to-move navigation in the live view).
	void movePositionCompensated(POINT2 distance);

	typedef enum class enScanDevice {
		ZEISSECU = 0,
		NIDAQ = 1,
		ZEISSMTB = 2,
		ZEISSMTBERLANGEN = 3,
		ZEISSMTBERLANGEN2 = 4
	} SCAN_DEVICE;
	inline static std::vector<std::string> SCAN_DEVICE_NAMES = { "Zeiss ECU", "NI-DAQmx", "Zeiss MTB", "Zeiss MTB Erlangen", "Zeiss MTB Erlangen 2" };

	std::vector<DeviceElement> m_deviceElements;
	std::vector<double> m_elementPositions;
	std::vector<double> m_elementPositionsTmp = m_elementPositions;

	std::vector<Preset> m_presets;
	ScanPreset m_activePresets = ScanPreset::SCAN_NULL;

public slots:
	virtual void setElement(DeviceElement, double) = 0;
	virtual int getElement(const DeviceElement&) = 0;
	virtual void getElements() = 0;
	// sets the position relative to the home position m_homePosition
	void setPositionRelativeX(double position);
	void setPositionRelativeY(double position);
	void setPositionRelativeZ(double position);

	// In case no laser scanner is present, the laser position needs to be provided by the user
	// (or searched automatically by the software later on).
	void locatePositionScanner(POINT2 positionLaserPix);

	bool supportsCapability(Capabilities);

	void setPositionInPix(POINT2);

	void enableMeasurementMode(bool enabled);

	void setPreset(ScanPreset presetType);
	Preset getPreset(ScanPreset);
	void checkPresets();
	bool isPresetActive(ScanPreset);
	// The "RL Shutter" device element (if this backend has one) is deliberately excluded
	// from setPreset()'s automatic per-element forcing (see setPreset()'s own comment) -
	// outside of an acquisition it is purely user/manual-controlled (via the beampath
	// buttons), and acquisition code that actually needs a specific state calls this
	// instead of relying on whatever a preset's table happens to say. No-op if this
	// backend has no element named "RL Shutter".
	void setRLShutterOpen(bool open);
	void announcePosition();
	void startAnnouncing();
	void stopAnnouncing();
	void startAnnouncingPosition();
	void stopAnnouncingPosition();
	void startAnnouncingElementPosition();
	void stopAnnouncingElementPosition();
	void setHome();
	POINT3 getHomePosition() const;
	void moveHome();
	void savePosition();
	void moveToSavedPosition(int index);
	void deleteSavedPosition(int index);

	std::vector<POINT3> getSavedPositionsNormalized();
	void announceSavedPositionsNormalized();
	
	void setScaleCalibration(const ScaleCalibrationData& scaleCalibration);
	ScaleCalibrationData getScaleCalibration();

	// Objective calibration profiles, keyed by the numeric position of the "Objective"
	// device element (the nosepiece slot). Registering/looking these up is matched generically
	// by DeviceElement::name == "Objective" (see onElementPositionChanged() below) rather than
	// any backend's own DEVICE_ELEMENT::OBJECTIVE - each scan-control backend (ZeissMTB.h,
	// ZeissECU.h, ...) defines that as a separate, backend-local enum for its own m_deviceElements
	// indexing, so it cannot be used here. This works uniformly across all backends without
	// touching any of their files.
	void setObjectiveCalibration(int slot, const ObjectiveCalibrationData& calibration);
	bool hasObjectiveCalibration(int slot) const;
	ObjectiveCalibrationData getObjectiveCalibration(int slot) const;
	int getActiveObjectiveSlot() const;
	// Empty/default (hasFovOffset == false) if the active slot has no registered calibration.
	ObjectiveCalibrationData getActiveObjectiveCalibration() const;
	// This objective's FOV-center offset relative to the reference objective, or {0,0} if
	// none is calibrated - always safe to add unconditionally to a grid origin (see
	// Brillouin::resolvedGridOriginUm()), since "no calibration" and "calibrated zero
	// offset" must resolve to the same harmless no-op.
	POINT2 getActiveObjectiveFovOffsetUm() const;

	// Whether `slot` is a physically-possible position of this backend's "Objective" device
	// element (1..maxOptions) - false if this backend has no "Objective" element at all (e.g.
	// plain NIDAQ). Used by the calibrations-folder auto-load (see
	// BrillouinAcquisition::autoLoadObjectiveCalibrations()) to tell a calibration file that
	// names a real, currently-possible slot apart from one saved on different hardware (or
	// never actually saved against a live objective changer, objectiveSlot == -1) - the latter
	// is flagged for the operator instead of being silently registered against a slot that may
	// not mean what the file thinks it means.
	bool isValidObjectiveSlot(int slot) const;

	// Called by the GUI once the operator has explicitly accepted running an objective
	// switch with no calibrated FOV-center offset (see s_objectiveSwitched()). Reset back to
	// false on the next objective switch - acceptance does not carry over.
	void acceptMissingObjectiveOffset();
	bool isMissingObjectiveOffsetAccepted() const;

	std::vector<POINT2> getPositionsPix(const std::vector<POINT3>& positionsMicrometer);
	std::vector<POINT2> getPositionsPix(const std::vector<POINT3>& positionsMicrometer, bool positionsAreAbsolute);
	// Same conversion as getPositionsPix(), but without caching the position for
	// re-emission on scale calibration change - use for ad-hoc overlay geometry
	// (e.g. mosaic tile outlines) that must not clobber the cached AOI markers.
	POINT2 getPositionPix(POINT3 positionMicrometer, bool positionIsAbsolute);

	// The micrometer offset getPositionPix()/convertPositionsToPix() add to a stored
	// (grid-offset or absolute) position before projecting it to pixels. Exposed so other
	// overlays that are stored in the same grid-offset frame (e.g. the ROI mask polygon)
	// can be projected with the exact same convention - re-deriving it independently is
	// what let those overlays drift apart from the AOI markers whenever the scanner
	// position was non-zero.
	POINT2 getPositionOffset(bool positionIsAbsolute);

	virtual POINT2 pixToMicroMeter(POINT2 positionPix);
	virtual POINT2 microMeterToPix(POINT2 positionMicrometer);
	POINT2 microMeterToPix(POINT3 positionMicrometer);

protected:
	virtual void setPresetAfter(ScanPreset presetType);

	virtual void calculateBounds();
	void calculateHomePositionBounds();
	void calculateCurrentPositionBounds();
	void calculateCurrentPositionBounds(POINT3 currentPosition);
	void announcePositions();
	void announcePositionScanner();
	void registerCapability(Capabilities);

	std::vector<Capabilities> m_capabilities;

	double m_positionFocus{ 0 };			// [um]	position of the focus (z-position)
	POINT2 m_positionStage{ 0, 0 };			// [um]	position of the stage (x-y-position)
	POINT2 m_positionScanner{ 0, 0 };		// [um]	position of the scanner (x-y-position)

	bool m_isCompatible{ false };
	POINT3 m_homePosition{ 0, 0, 0 };
	POINT2 m_startPosition{ 0, 0 };		// [um]	start position

	ScaleCalibrationData m_scaleCalibration;

	std::vector<POINT3> m_savedPositions;

	QTimer* m_positionTimer{ nullptr };
	QTimer* m_elementPositionTimer{ nullptr };

	BOUNDS m_absoluteBounds;
	BOUNDS m_homePositionBounds;
	BOUNDS m_currentPositionBounds;

	bool m_measurementMode{ false };
	bool m_AOI_positionsAbsolute{ false };
	POINT2 m_positionStageOld{ 0, 0 };
	POINT2 m_positionScannerOld{ 0, 0 };

private:
	std::vector<POINT2> convertPositionsToPix();

	std::vector<POINT3> m_AOI_positions;

	// -1 if no element named "Objective" exists on this backend at all (e.g. plain NIDAQ
	// setups without a motorized changer) - callers must check for that before indexing.
	int objectiveElementIndex() const;
	// Shared by onElementPositionChanged() and onElementPositionsChanged() - the actual
	// switch-detection/calibration-lookup/signal-emission logic, common to both the
	// software-commanded and the physically-polled change path.
	void handleObjectiveSlotObserved(int newSlot);

	std::map<int, ObjectiveCalibrationData> m_objectiveCalibrations;
	// -1 until the first "Objective" element position is observed (see
	// onElementPositionChanged()) - that first observation is the initial hardware read on
	// startup, not an operator-driven switch, and must not fire s_objectiveSwitched()/a warning.
	int m_activeObjectiveSlot{ -1 };
	bool m_objectiveOffsetWarningAccepted{ false };

private slots:
	// elementPositionChanged(DeviceElement, double) only fires for a software-commanded
	// change (see setElement() in each backend); a change made physically at the microscope's
	// own control panel is only ever reported through the polled, plural
	// elementPositionsChanged(vector<double>) (see each backend's getElements(), driven by
	// m_elementPositionTimer every 100 ms) - both must be handled, or a manual objective
	// switch at the stand would silently skip calibration lookup/the missing-offset warning.
	void onElementPositionChanged(DeviceElement element, double position);
	void onElementPositionsChanged(std::vector<double> positions);

signals:
	void elementPositionsChanged(std::vector<double>);
	void elementPositionChanged(DeviceElement, double);
	void currentPosition(POINT3);
	void savedPositionsChanged(std::vector<POINT3>);
	void homePositionBoundsChanged(BOUNDS);
	void currentPositionBoundsChanged(BOUNDS);
	void s_scaleCalibrationChanged(std::vector<POINT2>);
	void s_positionScannerChanged(POINT2);
	// Emitted alongside s_scaleCalibrationChanged, from the same computation, so a receiver
	// on another thread can cache the exact offset the just-emitted AOI pixel positions were
	// built from - see getPositionOffset()/announcePositions() for why re-fetching this live
	// afterward (rather than using this snapshot) is racy whenever something on this thread
	// (e.g. enableMeasurementMode(false) at the end of an acquisition) changes what
	// getPositionOffset() would return before the receiver gets around to processing the
	// queued signal.
	void s_gridOffsetChanged(POINT2 offsetUm, bool positionIsAbsolute);

	// Emitted whenever the "Objective" device element's position changes and is recognized as
	// an actual switch (not the initial startup read - previousSlot == -1 marks that case and
	// no receiver should warn on it). hasCalibration false means no scale calibration at all
	// was registered for the new slot (getScaleCalibration()/pixel<->um conversions for this
	// objective are simply wrong until one is loaded). hasCalibration true but hasFovOffset
	// false means a scale calibration exists but no FOV-center offset relative to the
	// reference objective was ever measured for this specific pair - grids/ROIs/overview tiles
	// will not be translated to compensate (see Brillouin::resolvedGridOriginUm()), and the
	// GUI must warn and require acceptMissingObjectiveOffset() before treating the switch as
	// resolved.
	void s_objectiveSwitched(int previousSlot, int newSlot, bool hasCalibration, bool hasFovOffset, POINT2 offsetUm, double offsetSigmaUm);
};

#endif // SCANCONTROL_H

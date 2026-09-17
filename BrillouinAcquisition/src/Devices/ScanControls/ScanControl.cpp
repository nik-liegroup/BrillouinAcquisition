#include "stdafx.h"
#include "ScanControl.h"
#include "src/helper/logger.h"

#include <chrono>
#include <thread>

ScanControl::ScanControl() noexcept {
	// Self-connected so objective-switch detection works uniformly for every backend
	// (ZeissECU, ZeissMTB and its Erlangen variants, NIDAQ) without touching any of their
	// files. Both signals are needed: elementPositionChanged is only emitted for a
	// software-commanded change (setElement(), e.g. a GUI button click), while a change made
	// physically at the microscope's own control panel is only ever reported through the
	// polled, plural elementPositionsChanged (see each backend's getElements(), driven by
	// m_elementPositionTimer every 100 ms) - relying on only one would miss the other's case.
	connect(this, &ScanControl::elementPositionChanged, this, &ScanControl::onElementPositionChanged);
	connect(this, &ScanControl::elementPositionsChanged, this, &ScanControl::onElementPositionsChanged);
}

/*
 * Public definitions
 */

bool ScanControl::getConnectionStatus() {
	return m_isConnected && m_isCompatible;
}

void ScanControl::movePosition(POINT2 distance) {
	auto position = getPosition();
	auto newPosition = POINT2{ position.x, position.y } + distance;
	setPosition(newPosition);
}

void ScanControl::movePosition(const POINT3& distance) {
	auto position = getPosition() + distance;
	setPosition(position);
}

void ScanControl::movePositionCompensated(POINT2 distance) {
	auto position = getPosition();
	auto newPosition = POINT2{ position.x, position.y } + distance;
	setPositionCompensated(POINT3{ newPosition.x, newPosition.y, position.z });
}

void ScanControl::setPositionCompensated(POINT3 position) {
	// Only mechanical translation stages exhibit backlash; galvo/voltage-driven
	// scanners do not need (and would just lose time to) this compensation.
	if (supportsCapability(Capabilities::TranslationStage)) {
		constexpr auto hysteresisCompensation{ 10.0 };	// [µm] distance for compensation of the stage hysteresis
		// getPosition() below re-queries the real hardware controller, not a cached software
		// value, so its readback carries genuine encoder/COM round-trip noise - a picometer-
		// scale epsilon never matches and would spuriously re-approach on every call, even
		// when the xy target is identical to the previous one (e.g. repeated z-only moves
		// within a surface-scan column). 0.5 µm is comfortably above that noise floor and
		// comfortably below any real intended xy step.
		constexpr auto epsilon{ 0.5 };					// [µm] axes closer than this are considered unchanged
		const auto current = getPosition();
		// To prevent problems with the hysteresis of the stage, we always approach
		// the desired point coming from lower x/y values, just like the scale calibration does.
		// Only axes that are actually moving are pre-approached, so e.g. a pure
		// z/focus move does not also nudge x/y back and forth for no reason.
		auto approach = position;
		auto needsApproach = false;
		if (std::abs(position.x - current.x) > epsilon) {
			approach.x = position.x - hysteresisCompensation;
			needsApproach = true;
		}
		if (std::abs(position.y - current.y) > epsilon) {
			approach.y = position.y - hysteresisCompensation;
			needsApproach = true;
		}
		if (needsApproach) {
			setPosition(approach);
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}
	}
	setPosition(position);
}

POINT3 ScanControl::getPosition(PositionType positionType) {
	auto pos = POINT2{};
	switch (positionType) {
		case PositionType::BOTH:
			// We return the absolute position, including the position of the stage and the scanner
			pos = m_positionStage + m_positionScanner;
			break;
		case PositionType::STAGE:
			pos = m_positionStage;
			break;
		case PositionType::SCANNER:
			pos = m_positionScanner;
			break;
		default:
			break;
	}

	return POINT3{ pos.x, pos.y, m_positionFocus };
}

/*
 * Public slots
 */

void ScanControl::setPositionRelativeX(double positionX) {
	// We use the base implementation of getPosition, so that
	// the hardware is not queried.
	auto position = ScanControl::getPosition();
	position.x = m_homePosition.x + positionX;

	// setPositionCompensated() only pre-approaches axes that actually change,
	// so this does not disturb y/z when only x is being set here.
	setPositionCompensated(position);
	announcePosition();
}

void ScanControl::setPositionRelativeY(double positionY) {
	// We use the base implementation of getPosition, so that
	// the hardware is not queried.
	auto position = ScanControl::getPosition();
	position.y = m_homePosition.y + positionY;

	setPositionCompensated(position);
	announcePosition();
}

void ScanControl::setPositionRelativeZ(double positionZ) {
	// We use the base implementation of getPosition, so that
	// the hardware is not queried.
	auto position = ScanControl::getPosition();
	position.z = m_homePosition.z + positionZ;

	// No-op for hysteresis compensation (only x/y are compensated), kept for
	// consistency so all manual position entry goes through the same path.
	setPositionCompensated(position);
	announcePosition();
}

void ScanControl::locatePositionScanner(POINT2 positionLaserPix) {
	// Don't allow to locate the laser position manually if the scanControl supports capability LaserScanner
	if (supportsCapability(Capabilities::LaserScanner)) {
		return;
	}

	// Stored with the active objective's own FOV-center offset subtracted back out, so
	// m_positionScanner itself stays an objective-invariant quantity (the real beam alignment
	// relative to the stage, not tied to whichever objective happened to be active when this
	// was last located) - see announcePositionScanner()/getPositionOffset() for the matching
	// "+ fovOffsetUm(active)" used everywhere this gets converted back to a pixel/raw-frame value.
	m_positionScanner = pixToMicroMeter(positionLaserPix) - getActiveObjectiveFovOffsetUm();

	announcePositionScanner();
	announcePositions();
}

void ScanControl::setPendingRestoredMarker(POINT2 positionPix, int objectiveSlot) {
	m_pendingRestoredMarkerPix = positionPix;
	m_pendingRestoredMarkerObjectiveSlot = objectiveSlot;
	m_hasPendingRestoredMarker = true;
	tryApplyPendingRestoredMarker();
}

void ScanControl::tryApplyPendingRestoredMarker() {
	if (!m_hasPendingRestoredMarker) {
		return;
	}
	if (m_activeObjectiveSlot != m_pendingRestoredMarkerObjectiveSlot) {
		// Not this objective (yet, or ever, this session) - stay pending rather than guessing.
		// If the operator switches back to the objective this was saved under, it will apply
		// then; if not, it's correctly left unset instead of reinterpreted under the wrong
		// calibration.
		return;
	}
	if (!hasObjectiveCalibration(m_activeObjectiveSlot)) {
		// Slot matches, but its calibration hasn't finished loading yet (loadCalibrationForSlot()
		// is asynchronous) - setObjectiveCalibration() retries this once it has.
		return;
	}
	m_hasPendingRestoredMarker = false;
	locatePositionScanner(m_pendingRestoredMarkerPix);
}

bool ScanControl::supportsCapability(Capabilities capability) {
	return std::find(m_capabilities.begin(), m_capabilities.end(), capability) != m_capabilities.end();
}

void ScanControl::setPositionInPix(POINT2 positionPix) {
	// This is the wanted position of the laser focus
	auto positionMicrometer = pixToMicroMeter(positionPix);
	// We have to subtract the position of the scanner to get to the relative movement
	positionMicrometer -= m_positionScanner;
	// ...and the active objective's own FOV-center offset - m_positionScanner is stored
	// objective-invariant (see locatePositionScanner()), so recovering "distance from the
	// marker's own drawn pixel, in this objective's raw pixel frame" needs the same
	// "+ fovOffsetUm(active)" announcePositionScanner() adds back when drawing that pixel in
	// the first place. Clicking exactly on the marker's own drawn pixel is the sanity check
	// this must satisfy (must always be a true no-op, for any fovOffsetUm) - it only holds
	// once announcePositionScanner() and locatePositionScanner() apply the matching
	// correction too, which they now do.
	positionMicrometer -= getActiveObjectiveFovOffsetUm();
	/**
	 * Prevent moving more than 1 cm at a time
	 *
	 * We don't want the stage to crash the objective into the sample holder.
	 * This is just for safety in case *something* goes wrong, and should not limit the normal operation.
	 * The field of view is way smaller, so this check should never trigger normally.
	 */
	if (abs(positionMicrometer) > 1e4) {
		return;
	}
	// Click-to-move in the live view: approach from a consistent direction so
	// repeatedly clicking back on the same spot lands there reproducibly.
	movePositionCompensated(positionMicrometer);
}

void ScanControl::enableMeasurementMode(bool enabled, POINT3 startPosition) {
	// The caller already reads the stage+scanner position once for its own start anchor
	// (Brillouin::m_startPosition) - reuse that exact reading here instead of querying
	// hardware a second time, so the two anchors can never diverge (stage jitter, or a
	// transient readback error on only one of the two calls).
	//
	// getActiveObjectiveFovOffsetUm() is deliberately not folded in here - a measurement must
	// target exactly the real coordinates verified while idle, on any objective. This
	// m_startPosition is ScanControl's own display-only copy (feeds getPositionOffset()'s
	// measurementMode branch), kept numerically in step with Brillouin::m_startPosition.
	if (enabled) {
		m_startPosition = POINT2{ startPosition.x, startPosition.y };
	}
	m_measurementMode = enabled;
}

void ScanControl::setPreset(ScanPreset presetType) {
	auto preset = getPreset(presetType);
	getElements();

	for (gsl::index ii{ 0 }; ii < m_deviceElements.size(); ii++) {
		// "RL Shutter" is deliberately excluded from this automatic per-preset forcing, so that
		// switching optical presets (e.g. for a brightfield preview, calibration, or scale
		// calibration) never clobbers whatever the user had it manually set to outside of an
		// actual acquisition. It's either left exactly as the user set it (manual beampath
		// button), or explicitly driven by acquisition code that needs a specific state - see
		// setRLShutterOpen().
		if (m_deviceElements[ii].name == "RL Shutter") {
			continue;
		}
		// check if element position needs to be changed
		if (!preset.elementPositions[ii].empty() && !simplemath::contains(preset.elementPositions[ii], m_elementPositions[ii])) {
			setElement(m_deviceElements[ii], preset.elementPositions[ii][0]);
			m_elementPositions[ii] = preset.elementPositions[ii][0];
		}
	}
	checkPresets();
	emit(elementPositionsChanged(m_elementPositions));

	setPresetAfter(presetType);
}

void ScanControl::setRLShutterOpen(bool open) {
	for (gsl::index ii{ 0 }; ii < m_deviceElements.size(); ii++) {
		if (m_deviceElements[ii].name == "RL Shutter") {
			// Position convention is consistent across every backend that has this
			// element: optionNames = { "Close", "Open" }, i.e. position 1 = Close, 2 = Open.
			const double position = open ? 2.0 : 1.0;
			setElement(m_deviceElements[ii], position);
			m_elementPositions[ii] = position;
			emit(elementPositionsChanged(m_elementPositions));
			return;
		}
	}
}

bool ScanControl::setBeamBlockOpen(bool open) {
	for (gsl::index ii{ 0 }; ii < m_deviceElements.size(); ii++) {
		if (m_deviceElements[ii].name == "Beam Block") {
			// Same "Close" = 1, "Open" = 2 convention setRLShutterOpen() relies on - every
			// backend with a Beam Block element declares it with optionNames = { "Close",
			// "Open" } too, and translates this generic 1/2 into whatever its own hardware
			// actually needs (a Thorlabs flip mount position, a raw DAQ TTL level, etc.) inside
			// its own setElement() dispatch.
			const double position = open ? 2.0 : 1.0;
			setElement(m_deviceElements[ii], position);
			m_elementPositions[ii] = position;
			emit(elementPositionsChanged(m_elementPositions));
			return true;
		}
	}
	return false;
}

Preset ScanControl::getPreset(ScanPreset presetType) {
	for (gsl::index ii{ 0 }; ii < m_presets.size(); ii++) {
		if (m_presets[ii].index == presetType) {
			return m_presets[ii];
		}
	}
	return m_presets[0];
}

void ScanControl::checkPresets() {
	// checks all presets if they are currenty active
	for (gsl::index ii{ 0 }; ii < m_presets.size(); ii++) {
		auto preset = m_presets[ii];
		auto active{ true };
		// check if an element position does not match the valid positions of a preset
		for (gsl::index jj{ 0 }; jj < preset.elementPositions.size(); jj++) {
			if (!preset.elementPositions[jj].empty() && !simplemath::contains(preset.elementPositions[jj], m_elementPositions[jj])) {
				m_activePresets &= ~preset.index;
				active = false;
				break;
			}
		}
		// set the preset active
		if (active) {
			m_activePresets |= preset.index;
		}
	}
}

bool ScanControl::isPresetActive(ScanPreset presetType) {
	return ScanPreset::SCAN_NULL != (presetType & m_activePresets);
}

void ScanControl::announcePosition() {
	auto point = getPosition();
	emit(currentPosition(point - m_homePosition));
	announcePositions();
}

void ScanControl::startAnnouncing() {
	startAnnouncingPosition();
	startAnnouncingElementPosition();
}

void ScanControl::stopAnnouncing() {
	stopAnnouncingPosition();
	stopAnnouncingElementPosition();
}

void ScanControl::startAnnouncingPosition() {
	if (m_positionTimer) {
		m_positionTimer->start(100);
	}
}

void ScanControl::stopAnnouncingPosition() {
	if (m_positionTimer) {
		m_positionTimer->stop();
	}
}

void ScanControl::startAnnouncingElementPosition() {
	if (m_elementPositionTimer) {
		m_elementPositionTimer->start(100);
	}
}

void ScanControl::stopAnnouncingElementPosition() {
	if (m_elementPositionTimer) {
		m_elementPositionTimer->stop();
	}
}

void ScanControl::setHome() {
	m_homePosition = getPosition();
	announceSavedPositionsNormalized();
	announcePosition();
	calculateHomePositionBounds();
}

POINT3 ScanControl::getHomePosition() const {
	return m_homePosition;
}

void ScanControl::moveHome() {
	// Approach from a consistent direction so returning home lands reproducibly.
	setPositionCompensated(m_homePosition);
}

void ScanControl::savePosition() {
	auto position = getPosition();
	m_savedPositions.push_back(position);
	announceSavedPositionsNormalized();
}

void ScanControl::moveToSavedPosition(int index) {
	if (m_savedPositions.size() > index) {
		// Approach from a consistent direction so the saved point is reached reproducibly.
		setPositionCompensated(m_savedPositions[index]);
	}
}

void ScanControl::deleteSavedPosition(int index) {
	if (m_savedPositions.size() > index) {
		m_savedPositions.erase(m_savedPositions.begin() + index);
		announceSavedPositionsNormalized();
	}
}

std::vector<POINT3> ScanControl::getSavedPositionsNormalized() {
	auto savedPositionsNormalized = m_savedPositions;
	std::transform(savedPositionsNormalized.begin(), savedPositionsNormalized.end(), savedPositionsNormalized.begin(),
		[this](POINT3 point) {
			return point - this->m_homePosition;
		}
	);
	return savedPositionsNormalized;
}

void ScanControl::announceSavedPositionsNormalized() {
	auto savedPositionsNormalized = getSavedPositionsNormalized();
	emit(savedPositionsChanged(savedPositionsNormalized));
}

void ScanControl::setScaleCalibration(const ScaleCalibrationData& scaleCalibration) {
	// m_positionScanner is deliberately left as-is (same [um] value) across a scale-calibration
	// change - it marks a real, physical, sample-relative location (e.g. where the beam actually
	// lands on this particular dish, which can be off-axis for sample-dependent optical reasons -
	// refraction/meniscus/mounting, not a fixed camera pixel), not a pixel on the sensor.
	// Reprojecting it through a pixel round-trip on every calibration change would keep its
	// ON-SCREEN pixel position fixed across an objective switch, but for an off-axis marker that
	// silently rescales its true [um] distance from the optical axis by the two objectives'
	// magnification ratio, corrupting every relative-mode grid point anchored to it (see
	// getPositionOffset()). Leaving the [um] value untouched means the marker (and everything
	// anchored to it) transforms exactly like any other physical location - through
	// microMeterToPix() under whichever calibration is active.
	m_scaleCalibration = scaleCalibration;

	calculateBounds();
	calculateHomePositionBounds();
	// A pure objective/calibration switch changes neither m_positionStage nor m_positionScanner,
	// so announcePositions() (the usual path to this pair of emissions) never runs on its own -
	// emit the same pair here too, in the same order (offset before pixel positions, so a queued
	// receiver processes the offset snapshot the pixel positions were computed from before the
	// positions themselves - see BrillouinAcquisition::on_gridOffsetChanged()).
	emit(s_gridOffsetChanged(getPositionOffset(m_AOI_positionsAbsolute), m_AOI_positionsAbsolute));
	emit(s_scaleCalibrationChanged(convertPositionsToPix()));
	// The marker's own drawn pixel (announcePositionScanner()'s microMeterToPix(m_positionScanner))
	// is calibration-dependent too, exactly like the AOI/grid positions convertPositionsToPix()
	// just re-emitted above - re-announce it here so it doesn't stay at its pre-switch screen
	// position until something else happens to call announcePositionScanner() again.
	announcePositionScanner();
}

ScaleCalibrationData ScanControl::getScaleCalibration() {
	return m_scaleCalibration;
}

void ScanControl::setObjectiveCalibration(int slot, const ObjectiveCalibrationData& calibration) {
	m_objectiveCalibrations[slot] = calibration;
	// If this is the objective currently in the beam path, apply its scale calibration
	// immediately rather than waiting for the next physical switch.
	if (slot == m_activeObjectiveSlot) {
		setScaleCalibration(calibration);
	}
	tryApplyPendingRestoredMarker();
}

bool ScanControl::hasObjectiveCalibration(int slot) const {
	return m_objectiveCalibrations.find(slot) != m_objectiveCalibrations.end();
}

ObjectiveCalibrationData ScanControl::getObjectiveCalibration(int slot) const {
	auto it = m_objectiveCalibrations.find(slot);
	if (it == m_objectiveCalibrations.end()) {
		return ObjectiveCalibrationData{};
	}
	return it->second;
}

int ScanControl::getActiveObjectiveSlot() const {
	return m_activeObjectiveSlot;
}

ObjectiveCalibrationData ScanControl::getActiveObjectiveCalibration() const {
	return getObjectiveCalibration(m_activeObjectiveSlot);
}

POINT2 ScanControl::getActiveObjectiveFovOffsetUm() const {
	auto calibration = getActiveObjectiveCalibration();
	if (!calibration.hasFovOffset) {
		return POINT2{ 0, 0 };
	}
	return calibration.fovOffsetUm;
}

void ScanControl::acceptMissingObjectiveOffset() {
	m_objectiveOffsetWarningAccepted = true;
}

bool ScanControl::isMissingObjectiveOffsetAccepted() const {
	return m_objectiveOffsetWarningAccepted;
}

bool ScanControl::isValidObjectiveSlot(int slot) const {
	for (const auto& element : m_deviceElements) {
		if (element.name == "Objective") {
			return slot >= 1 && slot <= element.maxOptions;
		}
	}
	return false;
}

void ScanControl::setObjectiveOptionNames(const std::vector<std::string>& names) {
	for (auto& element : m_deviceElements) {
		if (element.name == "Objective") {
			if ((int)names.size() != element.maxOptions) {
				return;
			}
			element.optionNames = names;
			return;
		}
	}
}

int ScanControl::objectiveElementIndex() const {
	for (const auto& element : m_deviceElements) {
		if (element.name == "Objective") {
			return element.index;
		}
	}
	return -1;
}

void ScanControl::onElementPositionChanged(DeviceElement element, double position) {
	if (element.name != "Objective") {
		return;
	}
	handleObjectiveSlotObserved((int)position);
}

void ScanControl::onElementPositionsChanged(std::vector<double> positions) {
	auto index = objectiveElementIndex();
	if (index < 0 || (size_t)index >= positions.size()) {
		return;
	}
	handleObjectiveSlotObserved((int)positions[index]);
}

void ScanControl::handleObjectiveSlotObserved(int newSlot) {
	if (newSlot == m_activeObjectiveSlot) {
		return;
	}
	// The turret can report a transient, invalid slot (observed: 0) while still mechanically
	// settling after a real switch - confirmed from a two-day log capture: 110+ occurrences,
	// every single one a "0" sandwiched between the real previous slot and the real new slot,
	// corrected again within under a second, never once persisting. Treating that dip as a
	// real switch used to set m_activeObjectiveSlot to 0, trigger "no calibration for slot 0"
	// (silently, via the early-return branch below, which is why this never showed up in a
	// plain qInfo search - it's a QMessageBox, not a log line), and then leave the very next,
	// genuine re-detection of the real slot logged with a bogus previousSlot of 0 instead of
	// the actual one. Ignoring invalid readings here - the same way the initial startup read
	// (previousSlot == -1) is already ignored below - stops the dip from ever being treated as
	// a real switch in the first place. If a real objective is ever actually mounted at slot 0,
	// this needs revisiting - but no observation of "0" in that capture ever lasted longer than
	// one poll tick, which a deliberately-selected slot would.
	if (!isValidObjectiveSlot(newSlot)) {
		return;
	}
	auto previousSlot = m_activeObjectiveSlot;
	m_activeObjectiveSlot = newSlot;
	// A fresh switch always needs a fresh decision - a warning accepted for the previous
	// switch must not silently cover this one too.
	m_objectiveOffsetWarningAccepted = false;

	auto hasCalibration = hasObjectiveCalibration(newSlot);
	if (hasCalibration) {
		// setScaleCalibration() leaves m_positionScanner's [um] value untouched - see its own
		// comment for why.
		setScaleCalibration(getObjectiveCalibration(newSlot));
	}
	auto calibration = getObjectiveCalibration(newSlot);
	auto hasFovOffset = hasCalibration && calibration.hasFovOffset;
	auto offsetUm = hasFovOffset ? calibration.fovOffsetUm : POINT2{ 0, 0 };
	auto offsetSigmaUm = hasFovOffset ? calibration.fovOffsetSigmaUm : 0.0;

	// m_startPosition has no FOV-offset baked into it at all (see enableMeasurementMode()'s own
	// comment), so there is nothing to correct here when the active objective's FOV-offset changes.

	// previousSlot == -1 is the initial hardware read at startup/connect, not an
	// operator-driven switch - do not warn about it (there is nothing to have translated
	// grids/ROIs relative to yet).
	if (previousSlot >= 0) {
		emit(s_objectiveSwitched(previousSlot, newSlot, hasCalibration, hasFovOffset, offsetUm, offsetSigmaUm));
	}

	tryApplyPendingRestoredMarker();
}

std::vector<POINT2> ScanControl::getPositionsPix(const std::vector<POINT3>& positionsMicrometer) {
	return getPositionsPix(positionsMicrometer, false);
}

std::vector<POINT2> ScanControl::getPositionsPix(const std::vector<POINT3>& positionsMicrometer, bool positionsAreAbsolute) {
	// Cache the requested positions so we can re-emit updated positions
	// in case the scale calibration changes
	m_AOI_positions = positionsMicrometer;
	m_AOI_positionsAbsolute = positionsAreAbsolute;

	return convertPositionsToPix();
};

POINT2 ScanControl::getPositionPix(POINT3 positionMicrometer, bool positionIsAbsolute) {
	const auto offset = getPositionOffset(positionIsAbsolute);
	return microMeterToPix(POINT2{ positionMicrometer.x, positionMicrometer.y } + offset);
}

POINT2 ScanControl::getPositionOffset(bool positionIsAbsolute) {
	// FOV-offset is a camera-frame translation, not part of a physical stage target.
	// Apply it once in EVERY display mode, just as announcePositionScanner() does.
	// Dropping it on Start moves only the overlay, although the stage still measures
	// the correct sample points.
	const auto fovOffset = getActiveObjectiveFovOffsetUm();
	if (positionIsAbsolute) {
		return POINT2{} - m_positionStage + fovOffset;
	}
	if (m_measurementMode) {
		// The captured start is stage + scanner, with no FOV-offset baked in.
		// For a measured target T, the stage backend sets stage = T - scanner;
		// hence T - stage + FOV projects exactly onto the laser marker.
		return m_startPosition - m_positionStage + fovOffset;
	}
	// At Start, start - stage == scanner, so the relative grid stays continuous.
	return m_positionScanner + fovOffset;
}

/*
 * Function converts a position in pixel to a position in um.
 * This is relative to the origin (pixOrigin) and not on an absolute scale e.g. of the translation stage.
 */
POINT2 ScanControl::pixToMicroMeter(POINT2 positionPix) {
	positionPix -= m_scaleCalibration.originPix;
	return positionPix.x * m_scaleCalibration.pixToMicrometerX + positionPix.y * m_scaleCalibration.pixToMicrometerY;
}

POINT2 ScanControl::microMeterToPix(POINT2 positionMicrometer) {
	return (positionMicrometer.x * m_scaleCalibration.micrometerToPixX + positionMicrometer.y * m_scaleCalibration.micrometerToPixY)
		+ m_scaleCalibration.originPix;
}

POINT2 ScanControl::microMeterToPix(POINT3 positionMicrometer) {
	return microMeterToPix(POINT2{ positionMicrometer.x, positionMicrometer.y });
}

/*
 * Protected definitions
 */
void ScanControl::setPresetAfter(ScanPreset presetType) {}

void ScanControl::calculateBounds() {
	// Bounds of the stage
	m_absoluteBounds = {
		-150000,	// [um] minimal x-value
		 150000,	// [um] maximal x-value
		-150000,	// [um] minimal y-value
		 150000,	// [um] maximal y-value
		-150000,	// [um] minimal z-value
		 150000		// [um] maximal z-value
	};
}

void ScanControl::calculateHomePositionBounds() {
	m_homePositionBounds.xMin = m_absoluteBounds.xMin - m_homePosition.x;
	m_homePositionBounds.xMax = m_absoluteBounds.xMax - m_homePosition.x;
	m_homePositionBounds.yMin = m_absoluteBounds.yMin - m_homePosition.y;
	m_homePositionBounds.yMax = m_absoluteBounds.yMax - m_homePosition.y;
	m_homePositionBounds.zMin = m_absoluteBounds.zMin - m_homePosition.z;
	m_homePositionBounds.zMax = m_absoluteBounds.zMax - m_homePosition.z;

	emit(homePositionBoundsChanged(m_homePositionBounds));
}

void ScanControl::calculateCurrentPositionBounds() {
	auto currentPosition = getPosition();
	calculateCurrentPositionBounds(currentPosition);
}

void ScanControl::calculateCurrentPositionBounds(POINT3 currentPosition) {
	m_currentPositionBounds.xMin = m_absoluteBounds.xMin - currentPosition.x;
	m_currentPositionBounds.xMax = m_absoluteBounds.xMax - currentPosition.x;
	m_currentPositionBounds.yMin = m_absoluteBounds.yMin - currentPosition.y;
	m_currentPositionBounds.yMax = m_absoluteBounds.yMax - currentPosition.y;
	m_currentPositionBounds.zMin = m_absoluteBounds.zMin - currentPosition.z;
	m_currentPositionBounds.zMax = m_absoluteBounds.zMax - currentPosition.z;

	emit(currentPositionBoundsChanged(m_currentPositionBounds));
}

/*
 * Announces updated marker positions if necessary.
 *
 * The AOI markers (crosses/ROI) redraw whenever stage or scanner changes: live-preview mode
 * tracks the scanner, absolute and measurement mode track stage+scanner combined (see
 * getPositionOffset()) - in every mode the grid pans so that whichever point is currently
 * being measured lands at the same fixed screen pixel (see commit 0c70d11, the original
 * version of this mechanism, and announcePositionScanner() below for the marker it lands on).
 */
void ScanControl::announcePositions() {
	const auto stageChanged = abs(m_positionStageOld - m_positionStage) >= 1e-6;
	const auto scannerChanged = abs(m_positionScannerOld - m_positionScanner) >= 1e-6;
	if (!stageChanged && !scannerChanged) {
		return;
	}

	m_positionStageOld = m_positionStage;
	m_positionScannerOld = m_positionScanner;

	// Emitted first so a queued receiver processes the offset snapshot before the pixel
	// positions that were computed from the exact same offset (see s_gridOffsetChanged()).
	emit(s_gridOffsetChanged(getPositionOffset(m_AOI_positionsAbsolute), m_AOI_positionsAbsolute));
	emit(s_scaleCalibrationChanged(convertPositionsToPix()));
}

void ScanControl::announcePositionScanner() {
	// A static calibration reference (see locatePositionScanner()), not something that
	// tracks the stage - the grid itself is what pans past this fixed point during a scan
	// (see getPositionOffset()/announcePositions()).
	//
	// + the active objective's own FOV-center offset, to convert m_positionScanner (stored
	// objective-invariant - see locatePositionScanner()) back into this objective's own raw
	// pixel frame - the same "m_positionScanner + fovOffsetUm(active)" getPositionOffset()
	// already uses for grids. Without this the marker was drawn correctly only on the
	// reference objective (fovOffsetUm == {0,0}) and landed off by fovOffsetUm on every other
	// one - exactly the "close but not exact" / clicking-misses-only-on-20x symptom.
	const auto positionScannerPix = microMeterToPix(m_positionScanner + getActiveObjectiveFovOffsetUm());
	emit(s_positionScannerChanged(positionScannerPix));
}

void ScanControl::registerCapability(Capabilities capability) {
	// Don't add a capability twice
	if (!supportsCapability(capability)) {
		m_capabilities.push_back(capability);
	}
}

/*
 * Private definitions
 */

std::vector<POINT2> ScanControl::convertPositionsToPix() {
	auto positionsPix = std::vector<POINT2>(m_AOI_positions.size());
	const auto offset = getPositionOffset(m_AOI_positionsAbsolute);

	std::transform(m_AOI_positions.begin(), m_AOI_positions.end(), positionsPix.begin(),
		[this, offset](POINT3 point) {
			return this->microMeterToPix(POINT2{ point.x, point.y } + offset);
		}
	);
	return positionsPix;
}

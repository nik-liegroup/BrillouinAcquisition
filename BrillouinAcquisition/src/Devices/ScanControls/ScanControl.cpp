#include "stdafx.h"
#include "ScanControl.h"

#include <chrono>
#include <thread>

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

	m_positionScanner = pixToMicroMeter(positionLaserPix);

	announcePositionScanner();
	announcePositions();
}

bool ScanControl::supportsCapability(Capabilities capability) {
	return std::find(m_capabilities.begin(), m_capabilities.end(), capability) != m_capabilities.end();
}

void ScanControl::setPositionInPix(POINT2 positionPix) {
	// This is the wanted position of the laser focus
	auto positionMicrometer = pixToMicroMeter(positionPix);
	// We have to subtract the position of the scanner to get to the relative movement
	positionMicrometer -= m_positionScanner;
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

void ScanControl::enableMeasurementMode(bool enabled) {
	// When enabling the measurement mode, we have to safe the start position,
	// so the AOI positions display has the correct origin.
	if (enabled) {
		auto pos = getPosition();
		m_startPosition = POINT2{ pos.x, pos.y };
	}
	m_measurementMode = enabled;
}

void ScanControl::setPreset(ScanPreset presetType) {
	auto preset = getPreset(presetType);
	getElements();

	for (gsl::index ii{ 0 }; ii < m_deviceElements.size(); ii++) {
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
	/*
	 * In order to prevent having to relocate the scanner position,
	 * we convert the scanner position to pixel using the old scale calibration
	 * and back to micro meter using the new scale.
	 */
	auto posScanner = microMeterToPix(m_positionScanner);
	m_scaleCalibration = scaleCalibration;
	m_positionScanner = pixToMicroMeter(posScanner);

	calculateBounds();
	calculateHomePositionBounds();
	emit(s_scaleCalibrationChanged(convertPositionsToPix()));
}

ScaleCalibrationData ScanControl::getScaleCalibration() {
	return m_scaleCalibration;
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
	// This is the mechanism from commit 0c70d11: the grid itself pans with the current
	// stage position, so that whichever point is currently being measured always lands at
	// the same fixed screen pixel - coinciding with the laser marker, which is a static
	// calibration reference (see announcePositionScanner()) and does NOT itself track the
	// stage. What looks like "the marker moving through the grid" is actually the grid
	// sliding past a fixed marker.
	//
	// In normal (live-preview) mode, the positions are shown relative to the scanner
	// position, so they track wherever the laser currently points within the FOV.
	auto offset = m_positionScanner;
	if (positionIsAbsolute) {
		// Absolute positions are stored as the raw target stage+scanner position directly
		// (absoluteGridOriginUm + gridOffset, see gridOffsetToAbsoluteTarget()), so the
		// scanner contribution is already baked into the stored value itself - subtracting
		// it again here would double-count it and shift the whole grid by that amount.
		// Only the stage position (which is what actually changes as the grid is scanned)
		// needs to be undone, exactly like the measurement-mode branch below.
		offset = POINT2{} - m_positionStage;
	}
	// In measurement mode, the positions are shown relative to the start position.
	else if (m_measurementMode) {
		// m_startPosition is captured as getPosition(BOTH) (stage + scanner) in
		// enableMeasurementMode(), but the scanner term cancels exactly the same way as
		// above - only stage needs to be subtracted here. This is the literal formula from
		// commit 0c70d11; adding a "- m_positionScanner" term here (as a previous revision
		// of this function did) shifts the whole grid by the scanner offset instead of
		// leaving it centered on the marker.
		offset = m_startPosition - m_positionStage;
	}
	return offset;
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
	const auto positionScannerPix = microMeterToPix(m_positionScanner);
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

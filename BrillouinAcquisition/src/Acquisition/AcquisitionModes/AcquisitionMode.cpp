#include "stdafx.h"
#include "AcquisitionMode.h"

/*
 * Public definitions
 */

AcquisitionMode::AcquisitionMode(QObject *parent, Acquisition *acquisition, ScanControl*& scanControl)
	: QObject(parent), m_acquisition(acquisition), m_scanControl(scanControl) {
}

AcquisitionMode::~AcquisitionMode() {
}

/*
 * Public slots
 */

ACQUISITION_STATUS AcquisitionMode::getStatus() {
	return m_status;
}

/*
 * Protected definitions
 */

void AcquisitionMode::setAcquisitionStatus(ACQUISITION_STATUS status) {
	m_status = status;
	emit(s_acquisitionStatus(m_status));
}

void AcquisitionMode::writeScaleCalibration(std::unique_ptr <StorageWrapper>& storage, ACQUISITION_MODE mode) {
	auto scaleCalibration = m_scanControl->getScaleCalibration();

	auto positionStage = m_scanControl->getPosition(PositionType::STAGE);
	auto positionScanner = m_scanControl->getPosition(PositionType::SCANNER);

	auto extended = ScaleCalibrationDataExtended{ scaleCalibration, positionStage, positionScanner };

	// Objective identity/FOV-offset context this specific run resolved its positions against -
	// this is the one place all four callers of writeScaleCalibration() (Brillouin's own
	// measurement, the overview brightfield captured during a Brillouin run, standalone
	// Fluorescence, and ODT) get it from, instead of only Brillouin's own measurement metadata
	// carrying it.
	auto activeCalibration = m_scanControl->getActiveObjectiveCalibration();
	extended.objectiveSlot = m_scanControl->getActiveObjectiveSlot();
	extended.objectiveName = activeCalibration.objectiveName;
	extended.magnification = activeCalibration.magnification;
	extended.referenceObjectiveName = activeCalibration.referenceObjectiveName;
	extended.hasFovOffset = activeCalibration.hasFovOffset;
	extended.fovOffsetUm = activeCalibration.fovOffsetUm;
	extended.fovOffsetSigmaUm = activeCalibration.fovOffsetSigmaUm;
	extended.missingOffsetAccepted = !activeCalibration.hasFovOffset && m_scanControl->isMissingObjectiveOffsetAccepted();

	storage->setScaleCalibration(mode, extended);
}

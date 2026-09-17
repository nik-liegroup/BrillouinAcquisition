#include "stdafx.h"
#include "CppUnitTest.h"
#include "..\BrillouinAcquisition\src\Devices\ScanControls\ScanControl.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace BrillouinAcquisitionUnitTest {

// Exercise the real ScanControl projection and click-to-move code without hardware.
class ProjectionScanControl : public ScanControl {
public:
	ProjectionScanControl() {
		m_deviceElements.emplace_back("Objective", 2, 0);
		m_positionStage = { 1000, -500 };
		m_positionScanner = { 12, -7 };
		for (int slot = 1; slot <= 2; ++slot) {
			ObjectiveCalibrationData calibration;
			const double scale = slot == 1 ? 2.0 : 4.0;
			calibration.micrometerToPixX = { scale, 0 };
			calibration.micrometerToPixY = { 0, -scale };
			calibration.pixToMicrometerX = { 1.0 / scale, 0 };
			calibration.pixToMicrometerY = { 0, -1.0 / scale };
			calibration.originPix = { 13, 27 };
			calibration.hasFovOffset = true;
			calibration.fovOffsetUm = slot == 1 ? POINT2{} : POINT2{ -40, 15 };
			setObjectiveCalibration(slot, calibration);
		}
		switchObjective(1);
	}
	void switchObjective(int slot) { emit(elementPositionChanged(m_deviceElements[0], slot)); }
	void init() override {}
	void connectDevice() override {}
	void disconnectDevice() override {}
	void setElement(DeviceElement, double) override {}
	int getElement(const DeviceElement&) override { return getActiveObjectiveSlot(); }
	void getElements() override {}
	void setPosition(POINT2 target) override { m_positionStage = target - m_positionScanner; }
	void setPosition(POINT3 target) override {
		setPosition(POINT2{ target.x, target.y });
		m_positionFocus = target.z;
	}
};

TEST_CLASS(ScanControlProjectionUnitTest) {
public:
	static void same(POINT2 expected, POINT2 actual) {
		Assert::AreEqual(expected.x, actual.x, 1e-9);
		Assert::AreEqual(expected.y, actual.y, 1e-9);
	}
	static POINT2 xy(POINT3 p) { return { p.x, p.y }; }

	TEST_METHOD(StartKeepsPreviewAndMeasuredPointReachesLaser) {
		for (bool absolute : { false, true }) {
			for (int slot : { 1, 2 }) {
				ProjectionScanControl scan;
				scan.switchObjective(slot);
				const auto start = scan.getPosition();
				const POINT3 delta{ 25, -14, 0 };
				const auto target = POINT3{ start.x + delta.x, start.y + delta.y, start.z };
				const auto displayed = absolute ? target : delta;
				const auto idle = scan.getPositionPix(displayed, absolute);
				scan.enableMeasurementMode(true);
				same(idle, scan.getPositionPix(displayed, absolute));
				// The same physical move must be made for either objective.
				const auto stageBefore = scan.getPosition(PositionType::STAGE);
				scan.setPosition(target);
				const auto stageAfter = scan.getPosition(PositionType::STAGE);
				same({ delta.x, delta.y }, { stageAfter.x - stageBefore.x, stageAfter.y - stageBefore.y });
				const auto marker = scan.microMeterToPix(xy(scan.getPosition(PositionType::SCANNER)) + scan.getActiveObjectiveFovOffsetUm());
				same(marker, scan.getPositionPix(displayed, absolute));
				// Clicking the measured grid point must cause no additional movement.
				scan.setPositionInPix(scan.getPositionPix(displayed, absolute));
				same(xy(stageAfter), xy(scan.getPosition(PositionType::STAGE)));
				scan.setPosition(start);
				scan.enableMeasurementMode(false);
				same(idle, scan.getPositionPix(displayed, absolute));
			}
		}
	}

	TEST_METHOD(ObjectiveSwitchReprojectsSameSamplePointInBothPhases) {
		for (bool absolute : { false, true }) {
			for (bool measuring : { false, true }) {
				ProjectionScanControl scan;
				const auto start = scan.getPosition();
				const POINT3 delta{ 25, -14, 0 };
				const auto displayed = absolute ? POINT3{ start.x + delta.x, start.y + delta.y, start.z } : delta;
				scan.enableMeasurementMode(measuring);
				const auto before = scan.getPositionPix(displayed, absolute);
				const auto stage = scan.getPosition(PositionType::STAGE);
				scan.switchObjective(2);
				// Independent image registration: 2x magnification about origin (13,27),
				// then F=(-40,15) um mapped with (4,-4) pixels/um.
				const POINT2 expected{ 13 + 2 * (before.x - 13) - 160, 27 + 2 * (before.y - 27) - 60 };
				same(expected, scan.getPositionPix(displayed, absolute));
				same(xy(stage), xy(scan.getPosition(PositionType::STAGE)));
				scan.switchObjective(1);
				same(before, scan.getPositionPix(displayed, absolute));
			}
		}
	}
};
}

#include "stdafx.h"
#include "CppUnitTest.h"
#include "../BrillouinAcquisition/src/Acquisition/AcquisitionModes/BrillouinResume.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace BrillouinAcquisitionUnitTest {

	TEST_CLASS(ResumeSettingsTest) {
	public:
		// Settings that differ from the defaults in as many places as possible
		static void changeSettings(BRILLOUIN_SETTINGS& s) {
			s.setXMin(-40.0);
			s.setXMax(60.0);
			s.setXSteps(11);
			s.setYMin(-25.5);
			s.setYMax(30.5);
			s.setYSteps(7);
			s.setZMin(-5.0);
			s.setZMax(35.0);
			s.setZSteps(9);
			s.sample = "Brain, stage 40";
			s.preCalibration = false;
			s.conCalibrationInterval = 7.5;
			s.nrCalibrationImages = 12;
			s.repetitions.count = 3;
			s.repetitions.interval = 22.5;
			s.repetitions.filePerRepetition = true;
			s.useRoiMask = true;
			s.roiPolygonUm = { POINT2{ -10.0, -10.0 }, POINT2{ 20.0, -8.5 }, POINT2{ 15.0, 14.25 } };
			s.useBackgroundRoiMask = true;
			s.backgroundRoiPolygonUm = { POINT2{ 100.0, 100.0 }, POINT2{ 120.0, 100.0 }, POINT2{ 110.0, 130.0 } };
			s.useSurfaceFollow = true;
			s.surfaceFollowHalfRangeUm = 12.5;
			s.surfaceMaxRewindUm = 77.0;
			s.surfaceDropFraction = 0.2;
			s.surfaceScanDirection = -1;
			s.preScanXYBin = 5;
			s.surfaceProxyRoiFrameOriginLeft = 123456789012LL;
			s.surfaceProxyRoi2FrameHeightPhysical = 2048;
			s.mediumReferenceValue = 412.75;
			s.gridCoordinatesAbsolute = true;
			s.absoluteGridOriginUm = POINT3{ 1.5, -2.5, 3.5 };
			s.saveOverviewBrightfieldPerZ = true;
			s.overviewBrightfieldExposureMs = 9;
			s.overviewBrightfieldFullGrid = true;
			s.overviewBrightfieldFullStackMosaic = true;
			s.overviewTrigger = OVERVIEW_TRIGGER::PER_REPETITION;
			s.overviewBeforeMeasurement = true;
			s.overviewAfterMeasurement = true;
			OVERVIEW_OBJECTIVE_SETTINGS entry;
			entry.slot = 2;
			entry.active = true;
			entry.exposureMs = 6;
			entry.gain = 1.5;
			entry.fullGrid = true;
			entry.fullStack = true;
			entry.stackSteps = 7;
			entry.stackRangeUm = 33.0;
			OVERVIEW_OBJECTIVE_SETTINGS second;
			second.slot = 4;
			second.exposureMs = 3;
			s.overviewObjectives = { entry, second };
			s.overviewObjectiveRetractUm = -2000.0;
			s.driftCorrectionEnabled = true;
			s.driftObjectiveSlot = 2;
			s.driftMinNcc = 0.55;
			s.driftMaxStepUm = 90.0;
			s.driftExtrapolate = true;
			s.surfaceRescan = SURFACE_RESCAN::PER_PLANE;
			s.useDoseProtection = true;
			s.extendFramesOnLowSignal = true;
			s.maxAdditionalFrames = 4;
			s.camera.exposureTime = 0.35;
			s.camera.frameCount = 6;
			s.camera.gain = 2.0;
			s.camera.spuriousNoiseFilter = false;
			s.camera.roi.left = 100;
			s.camera.roi.width_physical = 800;
			s.camera.roi.height_binned = 120;
			s.camera.roi.binning = L"2x2";
			s.camera.roi.bytesPerFrame = 38400;
			s.camera.readout.pixelReadoutRate = L"280 MHz";
			s.camera.readout.preAmpGain = L"12-bit (low noise)";
			s.camera.readout.dataType = "unsigned short";
		}

		TEST_METHOD(Settings_survive_the_round_trip) {
			BRILLOUIN_SETTINGS original;
			changeSettings(original);
			const auto record = resume::encodeSettings(original);

			BRILLOUIN_SETTINGS restored;
			const auto applied = resume::decodeSettings(record, &restored);
			Assert::IsTrue(applied > 100);

			// the restored settings give the same record again
			const auto again = resume::encodeSettings(restored);
			Assert::IsTrue(again.numbers == record.numbers);
			Assert::IsTrue(again.strings == record.strings);

			// and a few of them explicitly
			Assert::AreEqual(-40.0, restored.xMin, 1e-12);
			Assert::AreEqual(60.0, restored.xMax, 1e-12);
			Assert::AreEqual(11, restored.xSteps);
			Assert::AreEqual(9, restored.zSteps);
			Assert::AreEqual(std::string("Brain, stage 40"), restored.sample);
			Assert::IsFalse(restored.preCalibration);
			Assert::AreEqual(3, restored.repetitions.count);
			Assert::IsTrue(restored.repetitions.filePerRepetition);
			Assert::AreEqual((size_t)3, restored.roiPolygonUm.size());
			Assert::AreEqual(14.25, restored.roiPolygonUm[2].y, 1e-12);
			Assert::AreEqual((size_t)3, restored.backgroundRoiPolygonUm.size());
			Assert::AreEqual(123456789012LL, restored.surfaceProxyRoiFrameOriginLeft);
			Assert::AreEqual(412.75, restored.mediumReferenceValue, 1e-12);
			Assert::IsTrue(restored.gridCoordinatesAbsolute);
			Assert::AreEqual(-2.5, restored.absoluteGridOriginUm.y, 1e-12);
			Assert::IsTrue(restored.overviewTrigger == OVERVIEW_TRIGGER::PER_REPETITION);
			Assert::IsTrue(restored.surfaceRescan == SURFACE_RESCAN::PER_PLANE);
			Assert::AreEqual((size_t)2, restored.overviewObjectives.size());
			Assert::AreEqual(2, restored.overviewObjectives[0].slot);
			Assert::IsTrue(restored.overviewObjectives[0].active);
			Assert::AreEqual(7, restored.overviewObjectives[0].stackSteps);
			Assert::AreEqual(33.0, restored.overviewObjectives[0].stackRangeUm, 1e-12);
			Assert::AreEqual(4, restored.overviewObjectives[1].slot);
			Assert::IsFalse(restored.overviewObjectives[1].active);
			Assert::AreEqual(-2000.0, restored.overviewObjectiveRetractUm, 1e-12);
			Assert::AreEqual(0.35, restored.camera.exposureTime, 1e-12);
			Assert::AreEqual(6LL, restored.camera.frameCount);
			Assert::AreEqual(100LL, restored.camera.roi.left);
			Assert::IsTrue(restored.camera.roi.binning == L"2x2");
			Assert::AreEqual(38400, restored.camera.roi.bytesPerFrame);
			Assert::IsTrue(restored.camera.readout.preAmpGain == L"12-bit (low noise)");
			Assert::AreEqual(std::string("unsigned short"), restored.camera.readout.dataType);
		}

		TEST_METHOD(The_default_settings_round_trip_too) {
			BRILLOUIN_SETTINGS defaults;
			const auto record = resume::encodeSettings(defaults);
			BRILLOUIN_SETTINGS other;
			changeSettings(other);
			resume::decodeSettings(record, &other);		// everything is overwritten by the defaults
			const auto again = resume::encodeSettings(other);
			Assert::IsTrue(again.numbers == record.numbers);
			Assert::IsTrue(again.strings == record.strings);
		}

		TEST_METHOD(Entries_that_are_missing_leave_the_setting_alone) {
			BRILLOUIN_SETTINGS settings;
			changeSettings(settings);
			resume::Record record;
			record.numbers["camera.exposureTime"] = { 1.25 };
			const auto applied = resume::decodeSettings(record, &settings);
			Assert::AreEqual(1, applied);
			Assert::AreEqual(1.25, settings.camera.exposureTime, 1e-12);
			Assert::AreEqual(-40.0, settings.xMin, 1e-12);		// untouched
			Assert::AreEqual(6LL, settings.camera.frameCount);
		}

		TEST_METHOD(A_grid_with_both_limits_below_the_current_ones_is_restored) {
			// setting xMin before xMax must not leave the limits entangled with the previous grid
			BRILLOUIN_SETTINGS settings;
			settings.setXMin(50.0);
			settings.setXMax(80.0);
			BRILLOUIN_SETTINGS saved;
			saved.setXMin(-30.0);
			saved.setXMax(-10.0);
			resume::decodeSettings(resume::encodeSettings(saved), &settings);
			Assert::AreEqual(-30.0, settings.xMin, 1e-12);
			Assert::AreEqual(-10.0, settings.xMax, 1e-12);
			// and above
			BRILLOUIN_SETTINGS up;
			up.setXMin(100.0);
			up.setXMax(200.0);
			resume::decodeSettings(resume::encodeSettings(up), &settings);
			Assert::AreEqual(100.0, settings.xMin, 1e-12);
			Assert::AreEqual(200.0, settings.xMax, 1e-12);
		}

		TEST_METHOD(The_names_are_those_of_the_record) {
			BRILLOUIN_SETTINGS settings;
			changeSettings(settings);
			const auto record = resume::encodeSettings(settings);
			const auto numberNames = resume::settingNames(true);
			const auto textNames = resume::settingNames(false);
			Assert::AreEqual(record.numbers.size(), numberNames.size());
			Assert::AreEqual(record.strings.size(), textNames.size());
			for (const auto& name : numberNames) {
				Assert::IsTrue(record.numbers.count(name) == 1);
			}
		}

		TEST_METHOD(Text_with_non_ascii_characters_survives) {
			const std::wstring text = L"12-bit µm – test";
			Assert::IsTrue(resume::fromUtf8(resume::toUtf8(text)) == text);
			Assert::AreEqual(std::string("abc"), resume::toUtf8(L"abc"));
		}
	};

}

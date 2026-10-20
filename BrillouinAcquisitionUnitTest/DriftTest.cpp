#include "stdafx.h"
#include "CppUnitTest.h"
#include "../BrillouinAcquisition/src/lib/math/drift.h"
#include "TestScenes.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace BrillouinAcquisitionUnitTest {

	TEST_CLASS(DriftTest) {
	public:
		// A calibration as the real setup measures it: 2.12 pixels per um, camera axes turned by 90 degrees against the stage.
		static drift::StageMap rotatedMap() {
			return drift::stageMapFromCalibration(POINT2{ 0.0, 2.12157472 }, POINT2{ -2.12157472, 0.0 });
		}
		// and one that is only a bit rotated, so no axis is special
		static drift::StageMap tiltedMap() {
			return drift::stageMapFromCalibration(POINT2{ 1.9, 0.5 }, POINT2{ -0.5, 1.9 });
		}

		static constexpr int kWidth = 192;
		static constexpr int kHeight = 160;

		static drift::TileFrame tile(
			const std::vector<testscenes::Blob>& blobs, const drift::StageMap& map, POINT2 position, POINT2 driftUm, double noise = 0.0,
			unsigned seed = 1
		) {
			drift::TileFrame frame;
			frame.positionUm = position;
			const auto pixels = testscenes::stageScene(blobs, kWidth, kHeight, map, position, driftUm, noise, seed);
			frame.image = registration::prepare(pixels, kWidth, kHeight, 1);
			return frame;
		}

		TEST_METHOD(StageMap_from_the_calibration) {
			// the stored image has its rows inverted against the live view
			const auto map = rotatedMap();
			Assert::AreEqual(0.0, map.colX, 1e-12);
			Assert::AreEqual(-2.12157472, map.colY, 1e-9);
			Assert::AreEqual(-2.12157472, map.rowX, 1e-9);
			Assert::AreEqual(0.0, map.rowY, 1e-12);
			Assert::IsTrue(map.isValid());
			Assert::AreEqual(2.12157472, map.pixelsPerUm(), 1e-9);
			Assert::IsFalse(drift::StageMap{}.isValid());
		}

		TEST_METHOD(A_moved_stage_without_drift_is_no_drift) {
			// frame 2 is taken with the stage 10 um in x and 5 um in y away; the scene content moves by -map * move in the pixels, and the
			// conversion has to give back a drift of zero
			const auto map = tiltedMap();
			const auto movePixels = POINT2{ -(map.colX * 10.0 + map.colY * 5.0), -(map.rowX * 10.0 + map.rowY * 5.0) };
			const auto result = drift::shiftToDriftUm(map, movePixels.x, movePixels.y, POINT2{ 110.0, 55.0 }, POINT2{ 100.0, 50.0 });
			Assert::AreEqual(0.0, result.x, 1e-9);
			Assert::AreEqual(0.0, result.y, 1e-9);
		}

		TEST_METHOD(Pixel_shift_to_drift_with_the_stage_standing_still) {
			// the sample moved by (3, -2) um: the content shifts by map * (3, -2) pixels
			const auto map = rotatedMap();
			const auto shiftCol = map.colX * 3.0 + map.colY * -2.0;
			const auto shiftRow = map.rowX * 3.0 + map.rowY * -2.0;
			const auto result = drift::shiftToDriftUm(map, shiftCol, shiftRow, POINT2{ 5.0, 5.0 }, POINT2{ 5.0, 5.0 });
			Assert::AreEqual(3.0, result.x, 1e-9);
			Assert::AreEqual(-2.0, result.y, 1e-9);
		}

		TEST_METHOD(Single_tile_drift_is_recovered) {
			const auto map = rotatedMap();
			const auto blobs = testscenes::makeBlobs(2.0 * kWidth / map.pixelsPerUm(), 2.0 * kHeight / map.pixelsPerUm(), 5, 70, 2.5, 8.0);
			const auto reference = std::vector<drift::TileFrame>{ tile(blobs, map, POINT2{ 20.0, 30.0 }, POINT2{ 0.0, 0.0 }) };
			const auto moved = std::vector<drift::TileFrame>{ tile(blobs, map, POINT2{ 20.0, 30.0 }, POINT2{ 2.4, -1.7 }, 0.0, 2) };
			const auto step = drift::measureStep(reference, moved, map);
			Assert::IsTrue(step.valid);
			Assert::AreEqual(2.4, step.driftUm.x, 0.1);
			Assert::AreEqual(-1.7, step.driftUm.y, 0.1);
			Assert::AreEqual(1, step.tilesUsed);
			Assert::IsTrue(step.meanNcc > 0.9);
		}

		TEST_METHOD(Drift_is_recovered_when_the_stage_moved_as_well) {
			// the second overview is captured after the grid followed the sample: other stage position, drift (4, 1.5) um
			const auto map = tiltedMap();
			const auto blobs = testscenes::makeBlobs(2.0 * kWidth / map.pixelsPerUm(), 2.0 * kHeight / map.pixelsPerUm(), 6, 70, 2.5, 8.0);
			const auto reference = std::vector<drift::TileFrame>{ tile(blobs, map, POINT2{ 40.0, 40.0 }, POINT2{ 0.0, 0.0 }) };
			const auto moved = std::vector<drift::TileFrame>{ tile(blobs, map, POINT2{ 44.0, 41.5 }, POINT2{ 4.0, 1.5 }, 2.0, 3) };
			const auto step = drift::measureStep(reference, moved, map);
			Assert::IsTrue(step.valid);
			Assert::AreEqual(4.0, step.driftUm.x, 0.15);
			Assert::AreEqual(1.5, step.driftUm.y, 0.15);
		}

		TEST_METHOD(Several_tiles_are_combined_and_a_bad_tile_is_ignored) {
			const auto map = rotatedMap();
			const auto blobs = testscenes::makeBlobs(2.0 * kWidth / map.pixelsPerUm(), 2.0 * kHeight / map.pixelsPerUm(), 7, 70, 2.5, 8.0);
			const auto otherBlobs = testscenes::makeBlobs(2.0 * kWidth / map.pixelsPerUm(), 2.0 * kHeight / map.pixelsPerUm(), 900, 70, 2.5, 8.0);
			const auto drifted = POINT2{ 1.2, 2.2 };
			// three tiles at different stage positions; the third one shows an unrelated scene in the later overview (a dirty window)
			const auto reference = std::vector<drift::TileFrame>{
				tile(blobs, map, POINT2{ 0.0, 0.0 }, POINT2{ 0.0, 0.0 }), tile(blobs, map, POINT2{ 120.0, 0.0 }, POINT2{ 0.0, 0.0 }),
				tile(blobs, map, POINT2{ 0.0, 90.0 }, POINT2{ 0.0, 0.0 })
			};
			const auto moved = std::vector<drift::TileFrame>{
				tile(blobs, map, POINT2{ 0.0, 0.0 }, drifted, 1.0, 4), tile(blobs, map, POINT2{ 120.0, 0.0 }, drifted, 1.0, 5),
				tile(otherBlobs, map, POINT2{ 0.0, 90.0 }, POINT2{ 0.0, 0.0 }, 1.0, 6)
			};
			const auto step = drift::measureStep(reference, moved, map);
			Assert::IsTrue(step.valid);
			Assert::AreEqual(1.2, step.driftUm.x, 0.15);
			Assert::AreEqual(2.2, step.driftUm.y, 0.15);
			Assert::IsTrue(step.tilesUsed >= 2);
			Assert::AreEqual(3, step.tilesCompared);
		}

		TEST_METHOD(Tiles_at_different_stage_positions_are_not_paired) {
			const auto map = rotatedMap();
			const auto blobs = testscenes::makeBlobs(2.0 * kWidth / map.pixelsPerUm(), 2.0 * kHeight / map.pixelsPerUm(), 8, 70, 2.5, 8.0);
			const auto reference = std::vector<drift::TileFrame>{ tile(blobs, map, POINT2{ 0.0, 0.0 }, POINT2{ 0.0, 0.0 }) };
			const auto moved = std::vector<drift::TileFrame>{ tile(blobs, map, POINT2{ 500.0, 500.0 }, POINT2{ 0.0, 0.0 }) };
			const auto step = drift::measureStep(reference, moved, map);
			Assert::IsFalse(step.valid);
			Assert::AreEqual(0, step.tilesCompared);
		}

		TEST_METHOD(Tiles_are_paired_by_their_planned_position_when_the_grid_followed_the_drift) {
			// the grid followed a drift of 10 um in x, so the second overview's stage position is 10 um away from the first one's although it is
			// the same tile (pairing distance 5 um): the planned (anchor) position pairs them, and the drift is still the full 10 um
			const auto map = tiltedMap();
			const auto blobs = testscenes::makeBlobs(2.0 * kWidth / map.pixelsPerUm(), 2.0 * kHeight / map.pixelsPerUm(), 9, 70, 2.5, 8.0);
			auto first = tile(blobs, map, POINT2{ 40.0, 40.0 }, POINT2{ 0.0, 0.0 });
			auto second = tile(blobs, map, POINT2{ 50.0, 40.0 }, POINT2{ 10.0, 0.0 }, 1.0, 3);
			const auto unanchored = drift::measureStep({ first }, { second }, map, 0.3, 250.0, 5.0);
			Assert::IsFalse(unanchored.valid);
			first.anchorUm = POINT2{ 40.0, 40.0 };
			second.anchorUm = POINT2{ 40.0, 40.0 };
			const auto step = drift::measureStep({ first }, { second }, map, 0.3, 250.0, 5.0);
			Assert::IsTrue(step.valid);
			Assert::AreEqual(10.0, step.driftUm.x, 0.2);
			Assert::AreEqual(0.0, step.driftUm.y, 0.2);
		}

		TEST_METHOD(Unrelated_overviews_give_no_valid_step) {
			const auto map = rotatedMap();
			const auto blobsA = testscenes::makeBlobs(2.0 * kWidth / map.pixelsPerUm(), 2.0 * kHeight / map.pixelsPerUm(), 9, 70, 2.5, 8.0);
			const auto blobsB = testscenes::makeBlobs(2.0 * kWidth / map.pixelsPerUm(), 2.0 * kHeight / map.pixelsPerUm(), 123, 70, 2.5, 8.0);
			const auto reference = std::vector<drift::TileFrame>{ tile(blobsA, map, POINT2{ 0.0, 0.0 }, POINT2{ 0.0, 0.0 }) };
			const auto moved = std::vector<drift::TileFrame>{ tile(blobsB, map, POINT2{ 0.0, 0.0 }, POINT2{ 0.0, 0.0 }) };
			// smooth synthetic scenes correlate by chance far more than real tissue does (real tiles of different animals stay below 0.2), so the
			// limit is set high here
			const auto step = drift::measureStep(reference, moved, map, 0.85);
			Assert::IsFalse(step.valid);
			Assert::AreEqual(1, step.tilesCompared);
		}

		static drift::StepMeasurement measurement(double x, double y, double ncc, int tiles = 3) {
			drift::StepMeasurement step;
			step.valid = true;
			step.driftUm = POINT2{ x, y };
			step.meanNcc = ncc;
			step.maxNcc = ncc;
			step.tilesCompared = tiles;
			step.tilesUsed = tiles;
			return step;
		}

		TEST_METHOD(Controller_accumulates_accepted_steps) {
			drift::DriftController controller{ drift::DriftSettings{ true } };
			Assert::IsFalse(controller.hasReference());
			auto update = controller.setReference(0.0);
			Assert::IsTrue(update.decision == drift::DriftDecision::REFERENCE);
			update = controller.update(measurement(1.0, 0.5, 0.9), 270.0);
			Assert::IsTrue(update.decision == drift::DriftDecision::APPLIED);
			update = controller.update(measurement(0.5, -0.5, 0.8), 540.0);
			Assert::IsTrue(update.decision == drift::DriftDecision::APPLIED);
			Assert::AreEqual(1.5, controller.totalUm().x, 1e-12);
			Assert::AreEqual(0.0, controller.totalUm().y, 1e-12);
			Assert::AreEqual(540.0, controller.referenceTime(), 1e-12);
		}

		TEST_METHOD(Controller_keeps_the_grid_when_the_match_is_bad) {
			drift::DriftController controller{ drift::DriftSettings{ true } };
			controller.setReference(0.0);
			controller.update(measurement(1.0, 0.0, 0.9), 270.0);

			auto update = controller.update(measurement(2.0, 2.0, 0.2), 540.0);
			Assert::IsTrue(update.decision == drift::DriftDecision::REJECTED_LOW_CONFIDENCE);
			drift::StepMeasurement none;
			none.tilesCompared = 0;
			update = controller.update(none, 540.0);
			Assert::IsTrue(update.decision == drift::DriftDecision::REJECTED_NO_MATCH);
			none.tilesCompared = 2;		// tiles were compared but none was good enough
			update = controller.update(none, 540.0);
			Assert::IsTrue(update.decision == drift::DriftDecision::REJECTED_LOW_CONFIDENCE);

			// nothing changed, and the reference stays the last accepted overview
			Assert::AreEqual(1.0, controller.totalUm().x, 1e-12);
			Assert::AreEqual(270.0, controller.referenceTime(), 1e-12);
		}

		TEST_METHOD(Controller_rejects_implausible_steps) {
			drift::DriftSettings settings{ true };
			settings.maxStepUm = 50.0;
			settings.maxSpeedUmPerMin = 10.0;
			drift::DriftController controller{ settings };
			controller.setReference(0.0);
			// 80 um: further than allowed
			auto update = controller.update(measurement(80.0, 0.0, 0.95), 300.0);
			Assert::IsTrue(update.decision == drift::DriftDecision::REJECTED_IMPLAUSIBLE);
			// 40 um in one minute: faster than allowed
			update = controller.update(measurement(40.0, 0.0, 0.95), 60.0);
			Assert::IsTrue(update.decision == drift::DriftDecision::REJECTED_IMPLAUSIBLE);
			// 40 um in ten minutes: fine
			update = controller.update(measurement(40.0, 0.0, 0.95), 600.0);
			Assert::IsTrue(update.decision == drift::DriftDecision::APPLIED);
			Assert::AreEqual(40.0, controller.totalUm().x, 1e-12);
		}

		TEST_METHOD(Controller_carries_the_drift_over_to_the_next_series) {
			drift::DriftController controller{ drift::DriftSettings{ true } };
			const auto update = controller.setReference(100.0, POINT2{ 7.0, -3.0 });
			Assert::IsTrue(update.decision == drift::DriftDecision::REFERENCE);
			Assert::AreEqual(7.0, controller.totalUm().x, 1e-12);
			Assert::AreEqual(-3.0, controller.totalUm().y, 1e-12);
			controller.update(measurement(1.0, 1.0, 0.9), 400.0);
			Assert::AreEqual(8.0, controller.totalUm().x, 1e-12);
			Assert::AreEqual(-2.0, controller.totalUm().y, 1e-12);
		}

		TEST_METHOD(Controller_extrapolates_only_when_asked) {
			drift::DriftSettings settings{ true };
			drift::DriftController controller{ settings };
			controller.setReference(0.0);
			controller.update(measurement(3.0, 0.0, 0.9), 300.0);		// 0.01 um/s
			Assert::AreEqual(3.0, controller.offsetAt(450.0).x, 1e-12);

			settings.extrapolate = true;
			controller.setSettings(settings);
			Assert::AreEqual(4.5, controller.offsetAt(450.0).x, 1e-9);			// 150 s at 0.01 um/s
			Assert::AreEqual(9.0, controller.offsetAt(10000.0).x, 1e-9);		// limited to twice the measured interval (600 s at 0.01 um/s)
			Assert::AreEqual(3.0, controller.offsetAt(200.0).x, 1e-12);			// never backwards
		}

		TEST_METHOD(Controller_without_a_reference_takes_the_first_overview_as_reference) {
			drift::DriftController controller{ drift::DriftSettings{ true } };
			const auto update = controller.update(measurement(5.0, 5.0, 0.9), 10.0);
			Assert::IsTrue(update.decision == drift::DriftDecision::REFERENCE);
			Assert::AreEqual(0.0, controller.totalUm().x, 1e-12);
			Assert::IsTrue(controller.hasReference());
		}

		TEST_METHOD(Full_chain_over_several_overviews_with_noise) {
			// five overviews of a drifting sample, each compared with the last one: the cumulative drift is the sum of the steps
			const auto map = tiltedMap();
			const auto blobs = testscenes::makeBlobs(2.0 * kWidth / map.pixelsPerUm(), 2.0 * kHeight / map.pixelsPerUm(), 21, 70, 2.5, 8.0);
			const std::vector<POINT2> trueDrift = { { 0.0, 0.0 }, { 1.5, 0.4 }, { 3.1, 0.9 }, { 4.2, 1.1 }, { 6.0, 2.0 } };
			drift::DriftController controller{ drift::DriftSettings{ true } };
			controller.setReference(0.0);
			auto referenceFrame = std::vector<drift::TileFrame>{ tile(blobs, map, POINT2{ 0.0, 0.0 }, trueDrift[0], 2.0, 10) };
			for (size_t i = 1; i < trueDrift.size(); i++) {
				// the grid follows the sample: the stage is where the controller says
				const auto stage = controller.totalUm();
				const auto frame = std::vector<drift::TileFrame>{ tile(blobs, map, stage, trueDrift[i], 2.0, 10 + (unsigned)i) };
				const auto step = drift::measureStep(referenceFrame, frame, map);
				Assert::IsTrue(step.valid);
				const auto update = controller.update(step, 270.0 * (double)i);
				Assert::IsTrue(update.decision == drift::DriftDecision::APPLIED);
				referenceFrame = frame;
			}
			Assert::AreEqual(6.0, controller.totalUm().x, 0.3);
			Assert::AreEqual(2.0, controller.totalUm().y, 0.3);
		}
	};

}

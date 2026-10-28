#include "stdafx.h"
#include "CppUnitTest.h"
#include "../BrillouinAcquisition/src/lib/h5bm.h"
#include "../BrillouinAcquisition/src/Acquisition/AcquisitionModes/ResumeFile.h"

#include <filesystem>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace BrillouinAcquisitionUnitTest {

	/*
	 * Continue of an interrupted measurement against REAL HDF5 files (created in the temp folder, no data files needed): the measurement writes
	 * what a continue needs, "crashes" (the file is closed without the measurement having finished), the file is opened again and read.
	 * The decisions of a continue (objective, surface scan) are tested without any file.
	 */
	TEST_CLASS(ResumeFileTest) {
	public:
		// A file in the temp folder that is removed again when the test is over. Declared before the H5BM that uses it, so it is closed first.
		struct TempFile {
			std::string path;
			explicit TempFile(const std::string& name) {
				// the error messages HDF5 prints when the software probes for something that is not there are not of interest here
				H5Eset_auto2(H5E_DEFAULT, nullptr, nullptr);
				path = (std::filesystem::temp_directory_path() / ("brillouin_resume_test_" + name + ".h5")).string();
				std::error_code ignored;
				std::filesystem::remove(path, ignored);
			}
			~TempFile() {
				std::error_code ignored;
				std::filesystem::remove(path, ignored);
			}
		};

		static resume::Record settingsRecord() {
			resume::Record record;
			record.numbers["gridCoordinatesAbsolute"] = { 0.0 };
			record.numbers["xMin"] = { -40.0 };
			record.numbers["xMax"] = { 60.0 };
			record.numbers["xSteps"] = { 3.0 };
			record.numbers["yMin"] = { 0.0 };
			record.numbers["yMax"] = { 10.0 };
			record.numbers["ySteps"] = { 2.0 };
			record.numbers["zMin"] = { 0.0 };
			record.numbers["zMax"] = { 0.0 };
			record.numbers["zSteps"] = { 1.0 };
			record.numbers["preScanXYBin"] = { 3.0 };
			record.numbers["camera.exposureTime"] = { 0.2 };
			record.numbers["roiPolygonUm"] = {};			// an empty list must survive
			record.numbers["roiPolygonUm.filled"] = { 1.0, 2.0, 3.0, 4.0 };
			record.strings["sample"] = "Brain, stage 40 & more";
			record.strings["camera.roi.binning"] = "2x2";
			record.strings["empty"] = "";					// an empty text must survive
			return record;
		}

		static std::vector<std::string> numberNames(const resume::Record& record) {
			std::vector<std::string> names;
			for (const auto& entry : record.numbers) {
				names.push_back(entry.first);
			}
			return names;
		}

		static std::vector<std::string> textNames(const resume::Record& record) {
			std::vector<std::string> names;
			for (const auto& entry : record.strings) {
				names.push_back(entry.first);
			}
			return names;
		}

		// 3 x 2 x 1 points in measuring order, z as a surface scan left them
		static resume::Plan plan() {
			resume::Plan plan;
			for (int y = 0; y < 2; y++) {
				for (int x = 0; x < 3; x++) {
					plan.indices.push_back(INDEX3{ x, y, 0 });
					plan.positions.push_back(POINT3{ 100.0 + 10.0 * x, 200.0 + 10.0 * y, 50.0 + x + 0.5 * y });
					plan.positionsRelative.push_back(POINT3{ 10.0 * x, 10.0 * y, x + 0.5 * y });
					plan.calibrationAllowed.push_back(x == 0);
				}
			}
			return plan;
		}

		// A measurement that has started measuring: resume record, plan, state; points are added by writePoint()
		static void startMeasurement(H5BM& file, const std::string& state = "measuring") {
			file.newRepetition(ACQUISITION_MODE::BRILLOUIN);
			file.setResolution("x", 3);
			file.setResolution("y", 2);
			file.setResolution("z", 1);
			resume::writeBasics(&file, settingsRecord(), POINT3{ 11.5, -22.5, 33.5 }, 2, 0, -1, false, 1, 0, 2);
			resume::writePlan(&file, plan());
			file.setRepetitionState(ACQUISITION_MODE::BRILLOUIN, state);
		}

		static void writePoint(H5BM& file, int x, int y, bool complete) {
			static hsize_t dims[3] = { 1, 2, 2 };
			const std::vector<unsigned short> data{ 1, 2, 3, 4 };
			const std::string date = "2026-10-20T10:0" + std::to_string(y * 3 + x) + ":00.000+02:00";
			if (complete) {
				IMAGE<unsigned short> image(x, y, 0, 3, dims, date, data, 0.5, 1.0, CAMERA_ROI{}, POINT3{ 1.0, 2.0, 3.0 }, true, POINT3{ 1.0, 2.0, 3.0 });
				file.setPayloadData(&image);
			} else {
				// without its position attributes: the write was cut short
				file.setPayloadData(x, y, 0, data, 3, dims, date, 0.5, 1.0);
			}
		}

		static bool read(H5BM& file, resume::Info* info, std::string* problem) {
			const auto record = settingsRecord();
			return resume::readInfo(&file, ACQUISITION_MODE::BRILLOUIN, numberNames(record), textNames(record), info, problem);
		}

		TEST_METHOD(A_crashed_measurement_is_read_back_with_everything_it_needs) {
			TempFile temp("crash");
			const auto path = temp.path;
			{
				H5BM file(nullptr, path, H5F_ACC_TRUNC);
				startMeasurement(file);
				for (int i = 0; i < 4; i++) {
					writePoint(file, i % 3, i / 3, true);
				}
				writePoint(file, 1, 1, false);		// the point that was being written when the software stopped
			}												// closed without the measurement being finished

			H5BM reopened(nullptr, path, H5F_ACC_RDWR);
			resume::Info info;
			std::string problem;
			Assert::IsTrue(read(reopened, &info, &problem));
			Assert::AreEqual(0, info.repetition);
			Assert::IsTrue(info.state == "measuring");
			Assert::AreEqual(6, info.pointsTotal);
			Assert::AreEqual(4, info.pointsDone);
			Assert::IsTrue(info.orderedPrefix);
			Assert::AreEqual((size_t)1, info.incompletePoints.size());
			Assert::AreEqual(reopened.payloadIndex(1, 1, 0), info.incompletePoints[0]);
			Assert::IsTrue(info.lastDataDate == "2026-10-20T10:03:00.000+02:00");
			Assert::AreEqual(11.5, info.startPosition.x, 1e-12);
			Assert::AreEqual(-22.5, info.startPosition.y, 1e-12);
			Assert::AreEqual(33.5, info.startPosition.z, 1e-12);
			Assert::AreEqual(2, info.objectiveSlot);
			Assert::AreEqual(-1, info.fluorescenceRepetition);
			Assert::IsFalse(info.gridAbsolute);
			Assert::IsTrue(info.hasScanOrder);
			Assert::IsFalse(info.scanAutomatic);
			Assert::AreEqual(1, info.scanX);
			Assert::AreEqual(0, info.scanY);
			Assert::AreEqual(2, info.scanZ);

			// the plan, with the z the surface scan gave
			const auto expected = plan();
			Assert::AreEqual((size_t)6, info.positions.size());
			for (size_t i = 0; i < 6; i++) {
				Assert::AreEqual(expected.positions[i].x, info.positions[i].x, 1e-12);
				Assert::AreEqual(expected.positions[i].z, info.positions[i].z, 1e-12);
				Assert::AreEqual(expected.positionsRelative[i].y, info.positionsRelative[i].y, 1e-12);
				Assert::AreEqual(expected.indices[i].x, info.indices[i].x);
				Assert::AreEqual(expected.indices[i].y, info.indices[i].y);
				Assert::IsTrue(expected.calibrationAllowed[i] == info.calibrationAllowed[i]);
			}

			// the settings, including an empty list and an empty text
			const auto record = settingsRecord();
			Assert::IsTrue(info.settings.numbers == record.numbers);
			Assert::IsTrue(info.settings.strings == record.strings);
		}

		TEST_METHOD(Points_that_are_not_the_first_ones_of_the_plan_are_not_continued) {
			TempFile temp("order");
			const auto path = temp.path;
			H5BM file(nullptr, path, H5F_ACC_TRUNC);
			startMeasurement(file);
			writePoint(file, 0, 0, true);
			writePoint(file, 1, 0, true);
			writePoint(file, 0, 1, true);		// the third point of the plan (2, 0) is missing
			resume::Info info;
			std::string problem;
			Assert::IsTrue(read(file, &info, &problem));
			Assert::AreEqual(2, info.pointsDone);
			Assert::IsFalse(info.orderedPrefix);
		}

		TEST_METHOD(A_cancelled_measurement_can_be_continued_a_finished_one_cannot) {
			TempFile temp("states");
			const auto path = temp.path;
			H5BM file(nullptr, path, H5F_ACC_TRUNC);
			startMeasurement(file, "stopped");
			writePoint(file, 0, 0, true);
			resume::Info info;
			std::string problem;
			Assert::IsTrue(read(file, &info, &problem));
			Assert::IsTrue(info.state == "stopped");
			Assert::AreEqual(1, info.pointsDone);

			for (const char* state : { "finished", "preparing", "something else" }) {
				file.setRepetitionState(ACQUISITION_MODE::BRILLOUIN, state);
				resume::Info refused;
				std::string why;
				Assert::IsFalse(read(file, &refused, &why));
				Assert::IsTrue(!why.empty());
			}
		}

		TEST_METHOD(A_file_of_an_older_version_is_refused_with_a_reason) {
			TempFile temp("old");
			const auto path = temp.path;
			H5BM file(nullptr, path, H5F_ACC_TRUNC);
			file.newRepetition(ACQUISITION_MODE::BRILLOUIN);		// no state, no resume record
			resume::Info info;
			std::string problem;
			Assert::IsFalse(read(file, &info, &problem));
			Assert::IsTrue(problem.find("version") != std::string::npos);

			TempFile tempEmpty("empty");
			H5BM empty(nullptr, tempEmpty.path, H5F_ACC_TRUNC);	// no Brillouin measurement at all
			Assert::IsFalse(read(empty, &info, &problem));
			Assert::IsTrue(problem.find("no Brillouin") != std::string::npos);
		}

		TEST_METHOD(The_last_repetition_is_the_one_that_is_read) {
			TempFile temp("repetitions");
			const auto path = temp.path;
			H5BM file(nullptr, path, H5F_ACC_TRUNC);
			startMeasurement(file, "finished");		// repetition 0
			writePoint(file, 0, 0, true);
			file.newRepetition(ACQUISITION_MODE::BRILLOUIN);		// repetition 1
			file.setResolution("x", 3);
			file.setResolution("y", 2);
			file.setResolution("z", 1);
			resume::writeBasics(&file, settingsRecord(), POINT3{ 1.0, 2.0, 3.0 }, 1, 1, -1, true, 0, 1, 2);
			resume::writePlan(&file, plan());
			file.setRepetitionState(ACQUISITION_MODE::BRILLOUIN, "measuring");
			Assert::AreEqual(1, file.lastRepetition(ACQUISITION_MODE::BRILLOUIN));
			Assert::IsTrue(file.getRepetitionState(ACQUISITION_MODE::BRILLOUIN, 0) == "finished");
			Assert::IsTrue(file.getRepetitionState(ACQUISITION_MODE::BRILLOUIN, 1) == "measuring");

			resume::Info info;
			std::string problem;
			Assert::IsTrue(read(file, &info, &problem));
			Assert::AreEqual(1, info.repetition);
			Assert::AreEqual(0, info.pointsDone);					// repetition 1 has no points yet
			Assert::AreEqual(1, info.seriesRepetition);
			Assert::AreEqual(1, info.objectiveSlot);
			Assert::IsTrue(info.scanAutomatic);
		}

		TEST_METHOD(A_continued_measurement_does_not_overwrite_what_is_in_the_file) {
			TempFile temp("skip");
			const auto path = temp.path;
			H5BM file(nullptr, path, H5F_ACC_TRUNC);
			startMeasurement(file);
			std::vector<double> values;
			Assert::IsTrue(file.getResumeNumbers("plan-pos-z", &values));
			Assert::AreEqual(50.0, values[0], 1e-12);

			// the plan is written again with other values: with the skip flag the file keeps what it has ...
			auto changed = plan();
			for (auto& position : changed.positions) {
				position.z = 999.0;
			}
			file.setSkipExistingDatasets(true);
			resume::writePlan(&file, changed);
			Assert::IsTrue(file.getResumeNumbers("plan-pos-z", &values));
			Assert::AreEqual(50.0, values[0], 1e-12);
			// ... and writes what is new
			file.setResumeNumbers("something-new", std::vector<double>{ 7.0 });
			Assert::IsTrue(file.getResumeNumbers("something-new", &values));
			Assert::AreEqual(7.0, values[0], 1e-12);

			// back to normal: the same dataset is overwritten (a surface rescan updates the plan this way)
			file.setSkipExistingDatasets(false);
			resume::writePlan(&file, changed);
			Assert::IsTrue(file.getResumeNumbers("plan-pos-z", &values));
			Assert::AreEqual(999.0, values[0], 1e-12);
		}

		TEST_METHOD(Points_that_were_cut_short_are_removed_and_the_others_stay) {
			TempFile temp("incomplete");
			const auto path = temp.path;
			H5BM file(nullptr, path, H5F_ACC_TRUNC);
			startMeasurement(file);
			writePoint(file, 0, 0, true);
			writePoint(file, 1, 0, false);
			std::vector<int> complete, incomplete;
			file.listPayloadPoints(&complete, &incomplete);
			Assert::AreEqual((size_t)1, complete.size());
			Assert::AreEqual((size_t)1, incomplete.size());
			Assert::IsTrue(file.deletePayloadPoint(incomplete[0]));
			file.listPayloadPoints(&complete, &incomplete);
			Assert::AreEqual((size_t)1, complete.size());
			Assert::AreEqual((size_t)0, incomplete.size());
			// the point can be written again, complete this time
			writePoint(file, 1, 0, true);
			file.listPayloadPoints(&complete, &incomplete);
			Assert::AreEqual((size_t)2, complete.size());
		}

		TEST_METHOD(A_surface_scan_that_stopped_is_continued_with_its_finished_columns) {
			TempFile temp("surface");
			const auto path = temp.path;
			{
				H5BM file(nullptr, path, H5F_ACC_TRUNC);
				file.newRepetition(ACQUISITION_MODE::BRILLOUIN);
				resume::writeBasics(&file, settingsRecord(), POINT3{ 1.0, 2.0, 3.0 }, 1, 0, -1, true, 0, 1, 2);
				file.setRepetitionState(ACQUISITION_MODE::BRILLOUIN, "surface-scan");
				resume::writeSurfaceReference(&file, 412.5, 0.2, 7, 10);
				resume::writeSurfaceColumn(&file, 0, 0, true, 12.5, 0, 90.5, { 1.0, 2.0, 3.0 }, { 400.0, 380.0, 60.0 });
				resume::writeSurfaceColumn(&file, 1, 0, false, 0.0, -1, 410.0, {}, {});			// no surface found: no curve at all
				resume::writeSurfaceColumn(&file, 6, 9, true, -3.0, 1, 88.0, { 5.0 }, { 70.0 });
				file.flushFile();
			}
			H5BM reopened(nullptr, path, H5F_ACC_RDWR);
			resume::Info info;
			std::string problem;
			Assert::IsTrue(read(reopened, &info, &problem));
			Assert::IsTrue(info.surfaceScanPhase);
			Assert::AreEqual(412.5, info.surfaceReference, 1e-12);
			Assert::AreEqual((size_t)3, info.surfaceColumns.size());
			const auto& first = info.surfaceColumns[0];
			Assert::AreEqual(0, first.xi);
			Assert::AreEqual(0, first.yi);
			Assert::IsTrue(first.valid);
			Assert::AreEqual(12.5, first.z, 1e-12);
			Assert::AreEqual(90.5, first.metric, 1e-12);
			Assert::AreEqual((size_t)3, first.curveZ.size());
			Assert::AreEqual(380.0, first.curveMetric[1], 1e-12);
			const auto& second = info.surfaceColumns[1];
			Assert::AreEqual(1, second.xi);
			Assert::IsFalse(second.valid);
			Assert::AreEqual(-1, second.order);
			Assert::AreEqual((size_t)0, second.curveZ.size());
			const auto& third = info.surfaceColumns[2];
			Assert::AreEqual(6, third.xi);
			Assert::AreEqual(9, third.yi);

			// stopped before the medium reference was measured: nothing to continue
			TempFile tempEarly("surface_early");
			H5BM early(nullptr, tempEarly.path, H5F_ACC_TRUNC);
			early.newRepetition(ACQUISITION_MODE::BRILLOUIN);
			resume::writeBasics(&early, settingsRecord(), POINT3{ 0.0, 0.0, 0.0 }, 1, 0, -1, true, 0, 1, 2);
			early.setRepetitionState(ACQUISITION_MODE::BRILLOUIN, "surface-scan-stopped");
			Assert::IsFalse(read(early, &info, &problem));
		}

		TEST_METHOD(Every_continue_is_noted_in_the_file) {
			TempFile temp("events");
			const auto path = temp.path;
			H5BM file(nullptr, path, H5F_ACC_TRUNC);
			startMeasurement(file);
			resume::Info info;
			std::string problem;
			Assert::IsTrue(read(file, &info, &problem));
			Assert::AreEqual(0, info.resumeEvents);
			resume::writeResumeEvent(&file, 0, 4, 6, 1.7e9, false, 1, "measuring", "2026-10-20T10:03:00.000+02:00", "2026-10-20T11:00:00.000+02:00");
			resume::writeResumeEvent(&file, 1, 5, 6, 1.7e9 + 600, true, 0, "stopped", "2026-10-20T11:05:00.000+02:00", "2026-10-20T11:30:00.000+02:00");
			Assert::IsTrue(read(file, &info, &problem));
			Assert::AreEqual(2, info.resumeEvents);
			std::vector<double> values;
			Assert::IsTrue(file.getResumeNumbers("event-0000-was-cancelled", &values));
			Assert::AreEqual(0.0, values[0], 1e-12);
			Assert::IsTrue(file.getResumeNumbers("event-0001-was-cancelled", &values));
			Assert::AreEqual(1.0, values[0], 1e-12);
			Assert::IsTrue(file.getResumeNumbers("event-0000-incomplete-points-removed", &values));
			Assert::AreEqual(1.0, values[0], 1e-12);
			std::string text;
			Assert::IsTrue(file.getResumeText("event-0000-previous-state", &text));
			Assert::IsTrue(text == "measuring");
			Assert::IsTrue(file.getResumeText("event-0001-last-point-date", &text));
			Assert::IsTrue(text == "2026-10-20T11:05:00.000+02:00");
		}

		TEST_METHOD(Overview_events_and_surface_rescans_are_counted_with_the_grid_offset_they_left) {
			TempFile temp("engine");
			const auto path = temp.path;
			H5BM file(nullptr, path, H5F_ACC_TRUNC);
			startMeasurement(file);
			const hsize_t one[1] = { 1 };
			const hsize_t two[1] = { 2 };
			file.setPositions("drift-carried-over-x-um", { 1.5 }, 1, one);
			file.setPositions("drift-carried-over-y-um", { -2.5 }, 1, one);
			file.setPositions("overview-event-0000-point-index", { -1.0 }, 1, one);
			file.setPositions("overview-event-0000-grid-offset-after-um", { 0.0, 0.0 }, 1, two);
			file.setPositions("overview-event-0001-point-index", { 2.0 }, 1, one);
			file.setPositions("overview-event-0001-grid-offset-after-um", { 3.0, -4.0 }, 1, two);
			file.setPositions("overview-event-0002-point-index", { 5.0 }, 1, one);		// the offset was never written: the event is cut short
			file.setPositions("surface-rescan-0000-success", { 1.0 }, 1, one);
			resume::Info info;
			std::string problem;
			Assert::IsTrue(read(file, &info, &problem));
			Assert::AreEqual(2, info.overviewEvents);
			Assert::AreEqual(2, info.eventPointIndex[1]);
			Assert::AreEqual(3.0, info.eventOffsetAfter[1].x, 1e-12);
			Assert::AreEqual(-4.0, info.eventOffsetAfter[1].y, 1e-12);
			Assert::AreEqual(1, info.surfaceRescans);
			Assert::AreEqual(1.5, info.carriedOffset.x, 1e-12);
			Assert::AreEqual(-2.5, info.carriedOffset.y, 1e-12);
		}

		TEST_METHOD(The_surface_of_an_earlier_repetition_is_found_with_its_grid) {
			TempFile temp("previous");
			const auto path = temp.path;
			H5BM file(nullptr, path, H5F_ACC_TRUNC);
			startMeasurement(file, "finished");
			const hsize_t dims[2] = { 2, 3 };
			file.setPositions("surface-prescan-z-um", { 1.0, 2.0, 3.0, 4.0, 5.0, 6.0 }, 2, dims);
			file.setPositions("surface-prescan-found-mask", { 1.0, 1.0, 0.0, 2.0, 1.0, 1.0 }, 2, dims);
			file.newRepetition(ACQUISITION_MODE::BRILLOUIN);		// the next repetition

			resume::PreviousSurface previous;
			Assert::IsFalse(resume::readPreviousSurface(&file, ACQUISITION_MODE::BRILLOUIN, 0, &previous));		// nothing before repetition 0
			Assert::IsTrue(resume::readPreviousSurface(&file, ACQUISITION_MODE::BRILLOUIN, 1, &previous));
			Assert::AreEqual(0, previous.repetition);
			Assert::AreEqual((size_t)6, previous.z.size());
			Assert::AreEqual(5.0, previous.z[4], 1e-12);
			Assert::AreEqual(2.0, previous.found[3], 1e-12);
			Assert::IsTrue(previous.hasPlan);
			Assert::AreEqual((size_t)6, previous.planZ.size());
			Assert::AreEqual(11.5, previous.startPosition.x, 1e-12);
			Assert::AreEqual(-40.0, previous.settings["xMin"], 1e-12);
			Assert::AreEqual(3.0, previous.settings["xSteps"], 1e-12);
		}

		TEST_METHOD(Vectors_of_any_length_and_texts_of_any_content_survive) {
			TempFile temp("vectors");
			const auto path = temp.path;
			{
				H5BM file(nullptr, path, H5F_ACC_TRUNC);
				file.newRepetition(ACQUISITION_MODE::BRILLOUIN);
				file.setResumeNumbers("empty", {});
				file.setResumeNumbers("one", { 3.25 });
				std::vector<double> many(5000);
				for (size_t i = 0; i < many.size(); i++) {
					many[i] = 0.5 * (double)i;
				}
				file.setResumeNumbers("many", many);
				file.setResumeText("empty", "");
				file.setResumeText("long", std::string(3000, 'x') + " um {}[]");
			}
			H5BM file(nullptr, path, H5F_ACC_RDWR);
			Assert::IsTrue(file.openRepetition(ACQUISITION_MODE::BRILLOUIN, 0));
			std::vector<double> values{ 1.0, 2.0 };
			Assert::IsTrue(file.getResumeNumbers("empty", &values));
			Assert::AreEqual((size_t)0, values.size());
			Assert::IsTrue(file.getResumeNumbers("one", &values));
			Assert::AreEqual((size_t)1, values.size());
			Assert::AreEqual(3.25, values[0], 1e-12);
			Assert::IsTrue(file.getResumeNumbers("many", &values));
			Assert::AreEqual((size_t)5000, values.size());
			Assert::AreEqual(2499.5, values[4999], 1e-12);
			Assert::IsFalse(file.getResumeNumbers("does not exist", &values));
			std::string text = "unchanged";
			Assert::IsTrue(file.getResumeText("empty", &text));
			Assert::IsTrue(text.empty());
			Assert::IsTrue(file.getResumeText("long", &text));
			Assert::AreEqual((size_t)3008, text.size());
			Assert::IsFalse(file.getResumeText("does not exist", &text));
		}

		// ---------------------------------------------------------------------------------------------- decisions, no file

		TEST_METHOD(The_objective_of_the_measurement_is_turned_back_only_when_everything_allows_it) {
			using resume::ObjectiveAction;
			// the right objective, or none recorded: nothing to do
			Assert::IsTrue(resume::decideObjective(2, 2, true, -2000.0, true) == ObjectiveAction::NONE);
			Assert::IsTrue(resume::decideObjective(1, -1, true, -2000.0, true) == ObjectiveAction::NONE);
			// the 10x in the beam (software stopped during an overview), the 20x measured: turned back if the operator agrees, z can be retracted
			// and the changer may switch automatically (scale, FOV offset and focus offset exist)
			Assert::IsTrue(resume::decideObjective(1, 2, true, -2000.0, true) == ObjectiveAction::SWITCH);
			// the operator said no
			Assert::IsTrue(resume::decideObjective(1, 2, false, -2000.0, true) == ObjectiveAction::REFUSE);
			// no retract distance: never turn the nosepiece without moving away from the sample
			Assert::IsTrue(resume::decideObjective(1, 2, true, 0.0, true) == ObjectiveAction::REFUSE);
			// calibration missing
			Assert::IsTrue(resume::decideObjective(1, 2, true, -2000.0, false) == ObjectiveAction::REFUSE);
		}

		static resume::PreviousSurface previousSurface() {
			resume::PreviousSurface previous;
			previous.repetition = 0;
			previous.z.assign(70, 1.0);
			previous.found.assign(70, 1.0);
			previous.hasPlan = true;
			previous.settings = { { "xMin", -40.0 }, { "xMax", 60.0 }, { "xSteps", 11.0 }, { "yMin", 0.0 }, { "yMax", 90.0 }, { "ySteps", 10.0 }, { "zMin", 0.0 },
				{ "zMax", 20.0 }, { "zSteps", 5.0 }, { "gridCoordinatesAbsolute", 0.0 }, { "preScanXYBin", 3.0 }, { "absoluteGridOriginUm.x", 5.0 },
				{ "absoluteGridOriginUm.y", 6.0 } };
			return previous;
		}

		static resume::GridSignature gridNow() {
			resume::GridSignature now;
			now.xMin = -40.0;
			now.xMax = 60.0;
			now.xSteps = 11.0;
			now.yMin = 0.0;
			now.yMax = 90.0;
			now.ySteps = 10.0;
			now.zMin = 0.0;
			now.zMax = 20.0;
			now.zSteps = 5.0;
			now.absolute = false;
			now.preScanXYBin = 3.0;
			now.originX = 5.0;
			now.originY = 6.0;
			return now;
		}

		TEST_METHOD(An_earlier_surface_is_offered_only_for_the_same_grid) {
			const auto previous = previousSurface();
			auto now = gridNow();
			auto options = resume::surfaceOptions(previous, now, 70);
			Assert::IsTrue(options.xyGridSame && options.zGridSame && options.priorCompatible && options.canReuse);

			// other z planes: the scan can start from the earlier one, its surface cannot be reused
			now.zSteps = 6.0;
			options = resume::surfaceOptions(previous, now, 70);
			Assert::IsTrue(options.priorCompatible);
			Assert::IsFalse(options.canReuse);

			// another xy extent, step count or scan bin: neither
			for (int what = 0; what < 4; what++) {
				now = gridNow();
				if (what == 0) now.xMax = 61.0;
				if (what == 1) now.ySteps = 11.0;
				if (what == 2) now.preScanXYBin = 4.0;
				if (what == 3) now.absolute = true;
				options = resume::surfaceOptions(previous, now, 70);
				Assert::IsFalse(options.priorCompatible);
				Assert::IsFalse(options.canReuse);
			}

			// the same grid but a coarse scan of another size (e.g. the ROI mask changed the columns): no start from the earlier scan
			options = resume::surfaceOptions(previous, gridNow(), 64);
			Assert::IsFalse(options.priorCompatible);

			// absolute coordinates: the origin counts; relative: it does not
			now = gridNow();
			now.originX = 99.0;
			Assert::IsTrue(resume::surfaceOptions(previous, now, 70).priorCompatible);
			auto absolutePrevious = previous;
			absolutePrevious.settings["gridCoordinatesAbsolute"] = 1.0;
			now.absolute = true;
			Assert::IsFalse(resume::surfaceOptions(absolutePrevious, now, 70).priorCompatible);
			now.originX = 5.0;
			Assert::IsTrue(resume::surfaceOptions(absolutePrevious, now, 70).priorCompatible);

			// a plan that is not in the file (written by another version): no reuse
			auto noPlan = previous;
			noPlan.hasPlan = false;
			Assert::IsFalse(resume::surfaceOptions(noPlan, gridNow(), 70).canReuse);
			// settings that are not in the file: nothing is offered
			resume::PreviousSurface unknown;
			unknown.z.assign(70, 1.0);
			Assert::IsFalse(resume::surfaceOptions(unknown, gridNow(), 70).priorCompatible);
		}

		TEST_METHOD(A_grid_moved_by_the_drift_of_the_last_run_is_still_the_same_grid) {
			const auto previous = previousSurface();
			// the run ended with a drift of (3.25, -1.5) um: the software moved the grid by that much
			auto now = gridNow();
			now.xMin += 3.25;
			now.xMax += 3.25;
			now.yMin -= 1.5;
			now.yMax -= 1.5;
			// without knowing about the move: another grid
			Assert::IsFalse(resume::surfaceOptions(previous, now, 70).priorCompatible);
			// moved by exactly the drift: the same columns on the sample
			now.shiftX = 3.25;
			now.shiftY = -1.5;
			const auto options = resume::surfaceOptions(previous, now, 70);
			Assert::IsTrue(options.priorCompatible && options.canReuse);
			// moved further by hand: another grid
			now.xMax += 1.0;
			Assert::IsFalse(resume::surfaceOptions(previous, now, 70).priorCompatible);
			// a shift that does not match the move
			now = gridNow();
			now.shiftX = 3.25;
			Assert::IsFalse(resume::surfaceOptions(previous, now, 70).priorCompatible);
		}

		TEST_METHOD(The_answer_to_the_surface_question_defaults_to_starting_from_the_earlier_scan) {
			const auto both = resume::surfaceOptions(previousSurface(), gridNow(), 70);
			Assert::AreEqual(1, resume::resolveSurfaceAnswer(-1, both));		// nobody answered: Yes
			Assert::AreEqual(0, resume::resolveSurfaceAnswer(0, both));
			Assert::AreEqual(1, resume::resolveSurfaceAnswer(1, both));
			Assert::AreEqual(2, resume::resolveSurfaceAnswer(2, both));
			Assert::AreEqual(0, resume::resolveSurfaceAnswer(7, both));			// nonsense: a new scan

			auto now = gridNow();
			now.zSteps = 6.0;
			const auto priorOnly = resume::surfaceOptions(previousSurface(), now, 70);
			Assert::AreEqual(1, resume::resolveSurfaceAnswer(-1, priorOnly));
			Assert::AreEqual(0, resume::resolveSurfaceAnswer(2, priorOnly));	// reuse is not possible: a new scan

			now = gridNow();
			now.xMax = 70.0;
			const auto none = resume::surfaceOptions(previousSurface(), now, 70);
			Assert::AreEqual(0, resume::resolveSurfaceAnswer(-1, none));		// nothing possible: a new scan
			Assert::AreEqual(0, resume::resolveSurfaceAnswer(1, none));
		}
	};

}

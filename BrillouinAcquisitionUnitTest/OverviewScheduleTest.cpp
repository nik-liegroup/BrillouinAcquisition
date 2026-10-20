#include "stdafx.h"
#include "CppUnitTest.h"
#include "../BrillouinAcquisition/src/Acquisition/Planning/OverviewSchedule.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace BrillouinAcquisitionUnitTest {

	TEST_CLASS(OverviewScheduleTest) {
	public:
		// 3 z planes of 4 points each, z outermost
		static std::vector<int> threePlanes() {
			return { 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2 };
		}

		static int momentCount(const OverviewSchedule& schedule) {
			auto count = (int)schedule.before.size();
			for (const auto& moments : schedule.afterPoint) {
				count += (int)moments.size();
			}
			return count;
		}

		TEST_METHOD(Per_plane_overview_follows_the_planes_like_before) {
			const auto schedule = planOverviewSchedule(OVERVIEW_TRIGGER::PER_PLANE, false, false, SURFACE_RESCAN::NEVER, threePlanes(), 3, true);
			Assert::IsTrue(schedule.before.empty());
			Assert::AreEqual(3, momentCount(schedule));
			Assert::AreEqual(1, (int)schedule.afterPoint[3].size());
			Assert::AreEqual(1, (int)schedule.afterPoint[7].size());
			Assert::AreEqual(1, (int)schedule.afterPoint[11].size());
			Assert::AreEqual(0, schedule.afterPoint[3][0].zIndex);
			Assert::AreEqual(1, schedule.afterPoint[7][0].zIndex);
			Assert::AreEqual(2, schedule.afterPoint[11][0].zIndex);
			Assert::IsTrue(schedule.afterPoint[3][0].kind == OVERVIEW_MOMENT_KIND::PLANE);
			Assert::IsTrue(schedule.afterPoint[0].empty());
		}

		TEST_METHOD(Per_repetition_overview_is_one_after_the_last_point) {
			const auto schedule = planOverviewSchedule(OVERVIEW_TRIGGER::PER_REPETITION, false, false, SURFACE_RESCAN::NEVER, threePlanes(), 3, true);
			Assert::AreEqual(1, momentCount(schedule));
			Assert::AreEqual(1, (int)schedule.afterPoint[11].size());
			Assert::IsTrue(schedule.afterPoint[11][0].kind == OVERVIEW_MOMENT_KIND::REPETITION);
		}

		TEST_METHOD(Before_and_after_are_additional_and_never_double_up) {
			// not during the measurement: the start and the end only
			auto schedule = planOverviewSchedule(OVERVIEW_TRIGGER::NONE, true, true, SURFACE_RESCAN::NEVER, threePlanes(), 3, true);
			Assert::AreEqual(1, (int)schedule.before.size());
			Assert::IsTrue(schedule.before[0].kind == OVERVIEW_MOMENT_KIND::BEFORE);
			Assert::AreEqual(1, (int)schedule.afterPoint[11].size());
			Assert::IsTrue(schedule.afterPoint[11][0].kind == OVERVIEW_MOMENT_KIND::END);
			Assert::AreEqual(2, momentCount(schedule));

			// per plane: the last plane's overview already is the one after the measurement, nothing is added
			schedule = planOverviewSchedule(OVERVIEW_TRIGGER::PER_PLANE, true, true, SURFACE_RESCAN::NEVER, threePlanes(), 3, true);
			Assert::AreEqual(4, momentCount(schedule));
			Assert::IsTrue(schedule.afterPoint[11][0].kind == OVERVIEW_MOMENT_KIND::PLANE);

			// per repetition: likewise
			schedule = planOverviewSchedule(OVERVIEW_TRIGGER::PER_REPETITION, true, true, SURFACE_RESCAN::NEVER, threePlanes(), 3, true);
			Assert::AreEqual(2, momentCount(schedule));
		}

		TEST_METHOD(Nothing_is_planned_when_everything_is_off) {
			const auto schedule = planOverviewSchedule(OVERVIEW_TRIGGER::NONE, false, false, SURFACE_RESCAN::NEVER, threePlanes(), 3, true);
			Assert::AreEqual(0, momentCount(schedule));
			Assert::AreEqual(12, (int)schedule.afterPoint.size());
		}

		TEST_METHOD(Without_z_outermost_the_batches_are_spread_over_the_points) {
			// z innermost: no plane is finished before the end, so the three batches are spread evenly and the last one is after the last point
			const std::vector<int> zInnermost = { 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2 };
			const auto schedule = planOverviewSchedule(OVERVIEW_TRIGGER::PER_PLANE, false, false, SURFACE_RESCAN::NEVER, zInnermost, 3, false);
			Assert::AreEqual(3, momentCount(schedule));
			Assert::AreEqual(1, (int)schedule.afterPoint[3].size());			// ceil(1 * 12 / 3) - 1
			Assert::AreEqual(1, (int)schedule.afterPoint[7].size());			// ceil(2 * 12 / 3) - 1
			Assert::AreEqual(1, (int)schedule.afterPoint[11].size());
			Assert::AreEqual(0, schedule.afterPoint[3][0].zIndex);
			Assert::AreEqual(1, schedule.afterPoint[7][0].zIndex);
			Assert::AreEqual(2, schedule.afterPoint[11][0].zIndex);
		}

		TEST_METHOD(Surface_rescan_with_every_overview_but_never_after_the_last_point) {
			const auto schedule = planOverviewSchedule(OVERVIEW_TRIGGER::PER_PLANE, true, true, SURFACE_RESCAN::WITH_OVERVIEW, threePlanes(), 3, true);
			Assert::IsTrue(schedule.rescanAfterPoint[3]);
			Assert::IsTrue(schedule.rescanAfterPoint[7]);
			Assert::IsFalse(schedule.rescanAfterPoint[11]);
			Assert::IsFalse(schedule.rescanAfterPoint[0]);
		}

		TEST_METHOD(Surface_rescan_per_plane_is_independent_of_the_overview) {
			const auto schedule = planOverviewSchedule(OVERVIEW_TRIGGER::NONE, false, false, SURFACE_RESCAN::PER_PLANE, threePlanes(), 3, true);
			Assert::AreEqual(0, momentCount(schedule));
			Assert::IsTrue(schedule.rescanAfterPoint[3]);
			Assert::IsTrue(schedule.rescanAfterPoint[7]);
			Assert::IsFalse(schedule.rescanAfterPoint[11]);
			Assert::IsFalse(schedule.rescanAfterPoint[5]);
		}

		TEST_METHOD(No_rescan_when_it_is_off) {
			const auto schedule = planOverviewSchedule(OVERVIEW_TRIGGER::PER_PLANE, true, true, SURFACE_RESCAN::NEVER, threePlanes(), 3, true);
			for (const auto flag : schedule.rescanAfterPoint) {
				Assert::IsFalse(flag);
			}
		}

		TEST_METHOD(A_single_point_and_a_single_plane) {
			const auto schedule = planOverviewSchedule(OVERVIEW_TRIGGER::PER_PLANE, true, true, SURFACE_RESCAN::PER_PLANE, { 0 }, 1, true);
			Assert::AreEqual(1, (int)schedule.before.size());
			Assert::AreEqual(1, (int)schedule.afterPoint[0].size());
			Assert::IsFalse(schedule.rescanAfterPoint[0]);
			Assert::AreEqual(0, (int)planOverviewSchedule(OVERVIEW_TRIGGER::PER_PLANE, true, true, SURFACE_RESCAN::NEVER, {}, 1, true).afterPoint.size());
		}
	};

}

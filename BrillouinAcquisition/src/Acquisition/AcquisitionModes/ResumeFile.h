#ifndef RESUMEFILE_H
#define RESUMEFILE_H

#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <string>
#include <vector>

#include "ResumeRecord.h"
#include "../../lib/math/points.h"

/*
 * What is written to the file of a Brillouin measurement so that it can be continued, and how it is read back (Continue after a crash or a
 * cancel, see Brillouin_Resume.cpp), plus the decisions a continue takes (which objective, which surface scan). No Qt and no hardware: the
 * functions are templates over the storage (H5BM / StorageWrapper), so they are tested against real HDF5 files and against nothing at all.
 *
 * In the file, per Brillouin repetition: the attribute acquisition_state of the repetition group ("preparing", "surface-scan",
 * "surface-scan-stopped", "measuring", "stopped", "finished"), and in its payload group the datasets positions-resume-<name> (every vector with
 * its length in front) and the attributes resume_<name> (text), see H5BM::setResumeNumbers().
 */
namespace resume {

	inline std::string padded(int number, int digits) {
		auto text = std::to_string(number);
		while ((int)text.size() < digits) {
			text = "0" + text;
		}
		return text;
	}

	// A measurement that stopped during its surface scan: the medium reference and every column that was finished
	struct SurfaceColumn {
		int xi{ 0 };
		int yi{ 0 };
		bool valid{ false };
		double z{ 0.0 };
		int order{ -1 };
		double metric{ 0.0 };
		std::vector<double> curveZ;
		std::vector<double> curveMetric;
	};

	// Everything a continue of an interrupted measurement needs
	struct Info {
		int repetition{ -1 };				// the Brillouin repetition in the file
		int fluorescenceRepetition{ -1 };	// the one holding its overview images, -1 if none
		std::string state;
		Record settings;					// the settings the measurement was started with
		POINT3 startPosition{ 0.0, 0.0, 0.0 };
		int objectiveSlot{ -1 };
		bool gridAbsolute{ false };
		int seriesRepetition{ 0 };			// which repetition of the series this was
		std::vector<POINT3> positions;		// the plan: stage positions (z as the surface scan left them), in measurement order
		std::vector<POINT3> positionsRelative;
		std::vector<INDEX3> indices;
		std::vector<bool> calibrationAllowed;
		int pointsTotal{ 0 };
		int pointsDone{ 0 };				// planned points that are complete in the file (the first ones of the plan)
		bool orderedPrefix{ true };			// false: the complete points are not the first ones of the plan
		std::vector<int> incompletePoints;	// created but not complete (the write was cut short): removed on continue
		std::string lastDataDate;
		int calibrations{ 0 };
		int backgrounds{ 0 };
		int overviewEvents{ 0 };
		int surfaceRescans{ 0 };
		int resumeEvents{ 0 };				// how often this measurement was continued before
		POINT2 carriedOffset{ 0.0, 0.0 };	// grid offset the repetition started with
		std::vector<int> eventPointIndex;	// per overview event: the point it came after, and the grid offset from then on
		std::vector<POINT2> eventOffsetAfter;
		bool surfaceScanPhase{ false };
		double surfaceReference{ std::numeric_limits<double>::quiet_NaN() };
		std::vector<SurfaceColumn> surfaceColumns;
		std::vector<double> prescanZ;		// the surface of the last surface scan (flat like m_surfacePreScanZUm)
		std::vector<double> prescanFound;
		double referenceThreshold{ std::numeric_limits<double>::quiet_NaN() };
		bool hasScanOrder{ false };
		bool scanAutomatic{ true };
		int scanX{ 0 };
		int scanY{ 1 };
		int scanZ{ 2 };
	};

	// The plan of a measurement: its points in measuring order
	struct Plan {
		std::vector<POINT3> positions;
		std::vector<POINT3> positionsRelative;
		std::vector<INDEX3> indices;
		std::vector<bool> calibrationAllowed;
	};

	// The surface scan of an earlier repetition of the file
	struct PreviousSurface {
		int repetition{ -1 };
		std::vector<double> z;				// coarse surface of that scan and where it was found
		std::vector<double> found;
		bool hasPlan{ false };				// the plan of that repetition is in the file: its surface can be used without a new scan
		std::vector<INDEX3> planIndices;
		std::vector<double> planZ;			// z of its points (stage frame)
		POINT3 startPosition{ 0.0, 0.0, 0.0 };
		std::map<std::string, double> settings;		// grid and surface scan settings of that repetition
	};

	// ------------------------------------------------------------------------------------------------------------------------ writing

	template <class Storage>
	void writeBasics(Storage* storage, const Record& settings, const POINT3& start, int objectiveSlot, int series, int fluorescenceRepetition,
		bool scanAutomatic, int scanX, int scanY, int scanZ) {
		for (const auto& entry : settings.numbers) {
			storage->setResumeNumbers("s." + entry.first, entry.second);
		}
		for (const auto& entry : settings.strings) {
			storage->setResumeText("s." + entry.first, entry.second);
		}
		storage->setResumeNumbers("version", std::vector<double>{ 1.0 });
		storage->setResumeNumbers("start-position", std::vector<double>{ start.x, start.y, start.z });
		storage->setResumeNumbers("objective-slot", std::vector<double>{ (double)objectiveSlot });
		storage->setResumeNumbers("series", std::vector<double>{ (double)series });
		// the scan order preference (automatic, and the loop level of x, y and z)
		storage->setResumeNumbers("scan-order", std::vector<double>{ scanAutomatic ? 1.0 : 0.0, (double)scanX, (double)scanY, (double)scanZ });
		storage->setResumeNumbers("fluorescence-repetition", std::vector<double>{ (double)fluorescenceRepetition });
	}

	template <class Storage>
	void writePlan(Storage* storage, const Plan& plan) {
		const auto count = plan.positions.size();
		std::vector<double> ix(count), iy(count), iz(count), px(count), py(count), pz(count), rx(count), ry(count), rz(count), allowed(count);
		for (size_t i = 0; i < count; i++) {
			ix[i] = plan.indices[i].x;
			iy[i] = plan.indices[i].y;
			iz[i] = plan.indices[i].z;
			px[i] = plan.positions[i].x;
			py[i] = plan.positions[i].y;
			pz[i] = plan.positions[i].z;
			rx[i] = plan.positionsRelative[i].x;
			ry[i] = plan.positionsRelative[i].y;
			rz[i] = plan.positionsRelative[i].z;
			allowed[i] = plan.calibrationAllowed[i] ? 1.0 : 0.0;
		}
		storage->setResumeNumbers("plan-index-x", ix);
		storage->setResumeNumbers("plan-index-y", iy);
		storage->setResumeNumbers("plan-index-z", iz);
		storage->setResumeNumbers("plan-pos-x", px);
		storage->setResumeNumbers("plan-pos-y", py);
		storage->setResumeNumbers("plan-pos-z", pz);
		storage->setResumeNumbers("plan-posrel-x", rx);
		storage->setResumeNumbers("plan-posrel-y", ry);
		storage->setResumeNumbers("plan-posrel-z", rz);
		storage->setResumeNumbers("plan-calibration-allowed", allowed);
	}

	// [medium reference, drop fraction, columns in x, columns in y]
	template <class Storage>
	void writeSurfaceReference(Storage* storage, double mediumReference, double dropFraction, size_t columnsX, size_t columnsY) {
		storage->setResumeNumbers("surface-reference", std::vector<double>{ mediumReference, dropFraction, (double)columnsX, (double)columnsY });
	}

	// [found, z, order, metric, number of samples, curve z..., curve metric...]
	template <class Storage>
	void writeSurfaceColumn(Storage* storage, int xi, int yi, bool valid, double z, int order, double metric,
		const std::vector<double>& curveZ, const std::vector<double>& curveMetric) {
		std::vector<double> values{ valid ? 1.0 : 0.0, z, (double)order, metric, (double)curveZ.size() };
		values.insert(values.end(), curveZ.begin(), curveZ.end());
		values.insert(values.end(), curveMetric.begin(), curveMetric.end());
		storage->setResumeNumbers("surface-col-" + std::to_string(yi) + "-" + std::to_string(xi), values);
	}

	// The note in the file that a measurement was continued: where it had stopped and how
	template <class Storage>
	void writeResumeEvent(Storage* storage, int number, int pointsDone, int pointsTotal, double resumedEpochSeconds, bool wasCancelled,
		int incompletePointsRemoved, const std::string& previousState, const std::string& lastPointDate, const std::string& resumedDate) {
		const auto k = padded(number, 4);
		storage->setResumeNumbers("event-" + k + "-point-index", std::vector<double>{ (double)pointsDone });
		storage->setResumeNumbers("event-" + k + "-points-total", std::vector<double>{ (double)pointsTotal });
		storage->setResumeNumbers("event-" + k + "-resumed-epoch-seconds", std::vector<double>{ resumedEpochSeconds });
		// 0: nothing was written when the software stopped (crash, power loss), 1: the measurement was cancelled
		storage->setResumeNumbers("event-" + k + "-was-cancelled", std::vector<double>{ wasCancelled ? 1.0 : 0.0 });
		storage->setResumeNumbers("event-" + k + "-incomplete-points-removed", std::vector<double>{ (double)incompletePointsRemoved });
		storage->setResumeText("event-" + k + "-previous-state", previousState);
		storage->setResumeText("event-" + k + "-last-point-date", lastPointDate);
		storage->setResumeText("event-" + k + "-resumed-date", resumedDate);
	}

	// ------------------------------------------------------------------------------------------------------------------------ reading

	// Reads what a continue needs from the last Brillouin repetition of the file and opens that repetition (no data is changed). False, with the
	// reason in *problem, if the file holds nothing that can be continued.
	template <class Storage, class Mode>
	bool readInfo(Storage* storage, Mode mode, const std::vector<std::string>& settingNumberNames, const std::vector<std::string>& settingTextNames,
		Info* info, std::string* problem) {
		auto fail = [&](const std::string& text) {
			if (problem) {
				*problem = text;
			}
			return false;
		};
		if (!storage || !storage->isWritable()) {
			return fail("No acquisition file is open for writing.");
		}
		const auto repetition = storage->lastRepetition(mode);
		if (repetition < 0) {
			return fail("The open file holds no Brillouin measurement.");
		}
		const auto state = storage->getRepetitionState(mode, repetition);
		if (state.empty()) {
			return fail("The last Brillouin measurement of this file was written by a version that cannot be continued.");
		}
		if (state == "finished") {
			return fail("The last Brillouin measurement of this file is finished.");
		}
		if (state == "preparing") {
			return fail("The last Brillouin measurement was stopped before it started measuring (during the surface scan or before it), so there is nothing to continue; start it again.");
		}
		const auto surfacePhase = state == "surface-scan" || state == "surface-scan-stopped";
		if (state != "measuring" && state != "stopped" && !surfacePhase) {
			return fail("The last Brillouin measurement of this file is in the state \"" + state + "\", which cannot be continued.");
		}
		if (!storage->openRepetition(mode, repetition)) {
			return fail("The last Brillouin measurement of this file cannot be opened.");
		}

		Info result;
		result.repetition = repetition;
		result.state = state;

		std::vector<double> values;
		const auto number = [&](const std::string& name, size_t count) {
			return storage->getResumeNumbers(name, &values) && values.size() >= count;
		};
		if (!number("version", 1)) {
			return fail("The file has no continue information for its last Brillouin measurement.");
		}
		if (!number("start-position", 3)) {
			return fail("The start position of the measurement is not in the file.");
		}
		result.startPosition = POINT3{ values[0], values[1], values[2] };
		if (number("objective-slot", 1)) {
			result.objectiveSlot = (int)std::llround(values[0]);
		}
		if (number("series", 1)) {
			result.seriesRepetition = (int)std::llround(values[0]);
		}
		if (number("fluorescence-repetition", 1)) {
			result.fluorescenceRepetition = (int)std::llround(values[0]);
		}
		if (number("scan-order", 4)) {
			result.hasScanOrder = true;
			result.scanAutomatic = values[0] != 0.0;
			result.scanX = (int)std::llround(values[1]);
			result.scanY = (int)std::llround(values[2]);
			result.scanZ = (int)std::llround(values[3]);
		}

		// the settings
		for (const auto& name : settingNumberNames) {
			if (storage->getResumeNumbers("s." + name, &values)) {
				result.settings.numbers[name] = values;
			}
		}
		for (const auto& name : settingTextNames) {
			std::string text;
			if (storage->getResumeText("s." + name, &text)) {
				result.settings.strings[name] = text;
			}
		}
		if (result.settings.numbers.empty()) {
			return fail("The settings of the measurement are not in the file.");
		}
		const auto absolute = result.settings.numbers.find("gridCoordinatesAbsolute");
		result.gridAbsolute = absolute != result.settings.numbers.end() && !absolute->second.empty() && absolute->second.front() != 0.0;

		// earlier continues
		for (auto k = 0; storage->getResumeNumbers("event-" + padded(k, 4) + "-point-index", &values); k++) {
			result.resumeEvents++;
		}

		if (surfacePhase) {
			// stopped during the surface scan: the checkpoint (medium reference, every column that was finished), no plan yet
			result.surfaceScanPhase = true;
			std::vector<double> reference;
			if (!storage->getResumeNumbers("surface-reference", &reference) || reference.size() < 4) {
				return fail("The measurement stopped during its surface scan before the medium reference was measured; start it again.");
			}
			result.surfaceReference = reference[0];
			const auto columnsX = (int)std::llround(reference[2]);
			const auto columnsY = (int)std::llround(reference[3]);
			for (auto yi = 0; yi < columnsY; yi++) {
				for (auto xi = 0; xi < columnsX; xi++) {
					std::vector<double> column;
					if (!storage->getResumeNumbers("surface-col-" + std::to_string(yi) + "-" + std::to_string(xi), &column) || column.size() < 5) {
						continue;
					}
					SurfaceColumn entry;
					entry.xi = xi;
					entry.yi = yi;
					entry.valid = column[0] != 0.0;
					entry.z = column[1];
					entry.order = (int)std::llround(column[2]);
					entry.metric = column[3];
					const auto samples = (size_t)std::llround(column[4]);
					if (column.size() >= 5 + 2 * samples) {
						entry.curveZ.assign(column.begin() + 5, column.begin() + 5 + samples);
						entry.curveMetric.assign(column.begin() + 5 + samples, column.begin() + 5 + 2 * samples);
					}
					result.surfaceColumns.push_back(std::move(entry));
				}
			}
			*info = std::move(result);
			return true;
		}

		// the plan
		std::vector<double> ix, iy, iz, px, py, pz, rx, ry, rz, allowed;
		if (!storage->getResumeNumbers("plan-index-x", &ix) || !storage->getResumeNumbers("plan-index-y", &iy) || !storage->getResumeNumbers("plan-index-z", &iz)
			|| !storage->getResumeNumbers("plan-pos-x", &px) || !storage->getResumeNumbers("plan-pos-y", &py) || !storage->getResumeNumbers("plan-pos-z", &pz)
			|| !storage->getResumeNumbers("plan-posrel-x", &rx) || !storage->getResumeNumbers("plan-posrel-y", &ry) || !storage->getResumeNumbers("plan-posrel-z", &rz)
			|| !storage->getResumeNumbers("plan-calibration-allowed", &allowed)) {
			return fail("The plan of the measurement (its points) is not in the file.");
		}
		const auto total = ix.size();
		if (total == 0 || iy.size() != total || iz.size() != total || px.size() != total || py.size() != total || pz.size() != total || rx.size() != total
			|| ry.size() != total || rz.size() != total || allowed.size() != total) {
			return fail("The plan of the measurement in the file is inconsistent.");
		}
		for (size_t i = 0; i < total; i++) {
			result.indices.push_back(INDEX3{ (int)std::llround(ix[i]), (int)std::llround(iy[i]), (int)std::llround(iz[i]) });
			result.positions.push_back(POINT3{ px[i], py[i], pz[i] });
			result.positionsRelative.push_back(POINT3{ rx[i], ry[i], rz[i] });
			result.calibrationAllowed.push_back(allowed[i] != 0.0);
		}
		result.pointsTotal = (int)total;

		// which points are in the file
		std::vector<int> complete;
		storage->listPayloadPoints(&complete, &result.incompletePoints);
		const std::set<int> completeSet(complete.begin(), complete.end());
		std::vector<bool> done(total, false);
		for (size_t i = 0; i < total; i++) {
			done[i] = completeSet.count(storage->payloadIndex(result.indices[i].x, result.indices[i].y, result.indices[i].z)) > 0;
		}
		result.pointsDone = 0;
		while ((size_t)result.pointsDone < total && done[(size_t)result.pointsDone]) {
			result.pointsDone++;
		}
		for (size_t i = (size_t)result.pointsDone; i < total; i++) {
			if (done[i]) {
				result.orderedPrefix = false;
			}
		}
		if (result.pointsDone > 0) {
			const auto& last = result.indices[(size_t)result.pointsDone - 1];
			result.lastDataDate = storage->getPayloadDateByIndex(storage->payloadIndex(last.x, last.y, last.z));
		}
		result.calibrations = storage->countCalibrationData();
		result.backgrounds = storage->countBackgroundData();

		// the engine: overview events (with the grid offset from them on) and surface rescans
		for (auto k = 0;; k++) {
			std::vector<double> point, offset;
			const auto prefix = "overview-event-" + padded(k, 4) + "-";
			if (!storage->readPositions(prefix + "point-index", &point) || point.empty() || !storage->readPositions(prefix + "grid-offset-after-um", &offset)
				|| offset.size() < 2) {
				break;
			}
			result.eventPointIndex.push_back((int)std::llround(point[0]));
			result.eventOffsetAfter.push_back(POINT2{ offset[0], offset[1] });
		}
		result.overviewEvents = (int)result.eventPointIndex.size();
		for (auto k = 0; storage->readPositions("surface-rescan-" + padded(k, 4) + "-success", &values); k++) {
			result.surfaceRescans++;
		}
		std::vector<double> carriedX, carriedY;
		if (storage->readPositions("drift-carried-over-x-um", &carriedX) && storage->readPositions("drift-carried-over-y-um", &carriedY) && !carriedX.empty()
			&& !carriedY.empty()) {
			result.carriedOffset = POINT2{ carriedX[0], carriedY[0] };
		}

		// the last surface scan (the prior of a later rescan)
		storage->readPositions("surface-prescan-z-um", &result.prescanZ);
		storage->readPositions("surface-prescan-found-mask", &result.prescanFound);
		if (storage->readPositions("surface-reference-threshold-computed", &values) && !values.empty()) {
			result.referenceThreshold = values[0];
		}

		*info = std::move(result);
		return true;
	}

	// The surface scan of the latest repetition before `beforeRepetition` that has one (and, if it was written with its plan, the plan)
	template <class Storage, class Mode>
	bool readPreviousSurface(Storage* storage, Mode mode, int beforeRepetition, PreviousSurface* previous) {
		for (auto repetition = beforeRepetition - 1; repetition >= 0; repetition--) {
			std::vector<double> z, found;
			if (!storage->readPositionsOfRepetition(mode, repetition, "surface-prescan-z-um", &z)
				|| !storage->readPositionsOfRepetition(mode, repetition, "surface-prescan-found-mask", &found) || z.empty() || z.size() != found.size()) {
				continue;
			}
			PreviousSurface result;
			result.repetition = repetition;
			result.z = std::move(z);
			result.found = std::move(found);
			std::vector<double> ix, iy, iz, pz, start;
			// the resume vectors carry their length in front (see H5BM::setResumeNumbers())
			const auto resumeVector = [&](const std::string& name, std::vector<double>* out) {
				if (!storage->readPositionsOfRepetition(mode, repetition, "resume-" + name, out) || out->empty()) {
					return false;
				}
				out->erase(out->begin());
				return true;
			};
			if (resumeVector("plan-index-x", &ix) && resumeVector("plan-index-y", &iy) && resumeVector("plan-index-z", &iz) && resumeVector("plan-pos-z", &pz)
				&& resumeVector("start-position", &start) && ix.size() == iy.size() && ix.size() == iz.size() && ix.size() == pz.size() && !ix.empty()
				&& start.size() >= 3) {
				result.hasPlan = true;
				for (size_t i = 0; i < ix.size(); i++) {
					result.planIndices.push_back(INDEX3{ (int)std::llround(ix[i]), (int)std::llround(iy[i]), (int)std::llround(iz[i]) });
				}
				result.planZ = std::move(pz);
				result.startPosition = POINT3{ start[0], start[1], start[2] };
			}
			// the grid and the surface scan settings of that repetition (a surface is only reused for the same grid)
			for (const char* name : { "xMin", "xMax", "xSteps", "yMin", "yMax", "ySteps", "zMin", "zMax", "zSteps", "gridCoordinatesAbsolute", "preScanXYBin",
				"absoluteGridOriginUm.x", "absoluteGridOriginUm.y", "absoluteGridOriginUm.z" }) {
				std::vector<double> value;
				if (resumeVector(std::string("s.") + name, &value) && !value.empty()) {
					result.settings[name] = value[0];
				}
			}
			*previous = std::move(result);
			return true;
		}
		return false;
	}

	// ------------------------------------------------------------------------------------------------------------------------ decisions

	enum class ObjectiveAction {
		NONE,		// the objective of the measurement is in the beam
		SWITCH,		// another one is: turn the nosepiece back (z is retracted first)
		REFUSE		// another one is, and it cannot (or may not) be turned automatically: the operator does it by hand
	};

	inline ObjectiveAction decideObjective(int activeSlot, int storedSlot, bool operatorAllowsSwitch, double retractUm, bool changerAllowsSwitch) {
		if (storedSlot < 0 || activeSlot == storedSlot) {
			return ObjectiveAction::NONE;
		}
		if (operatorAllowsSwitch && retractUm != 0.0 && changerAllowsSwitch) {
			return ObjectiveAction::SWITCH;
		}
		return ObjectiveAction::REFUSE;
	}

	// The grid of a measurement, as far as a surface scan depends on it
	struct GridSignature {
		double xMin{ 0.0 };
		double xMax{ 0.0 };
		double xSteps{ 0.0 };
		double yMin{ 0.0 };
		double yMax{ 0.0 };
		double ySteps{ 0.0 };
		double zMin{ 0.0 };
		double zMax{ 0.0 };
		double zSteps{ 0.0 };
		bool absolute{ false };
		double preScanXYBin{ 0.0 };
		double originX{ 0.0 };
		double originY{ 0.0 };
		// [um] how far the software itself has moved the grid since that repetition (the drift of the run, see Brillouin::bakeDriftIntoGrid()):
		// a grid moved by exactly this is still the same grid on the sample
		double shiftX{ 0.0 };
		double shiftY{ 0.0 };
	};

	struct SurfaceOptions {
		bool xyGridSame{ false };
		bool zGridSame{ false };
		bool priorCompatible{ false };		// the earlier scan can be the start of the new one
		bool canReuse{ false };				// its surface can be used without a new scan
		std::string differences;			// the grid settings that differ from that repetition (for the log)
	};

	// A surface of an earlier repetition only fits the same grid: its columns are where this repetition's columns are only if the extent, steps
	// and surface scan bin are the same (and in absolute coordinates the same origin); its plan only if the z planes are the same too.
	inline SurfaceOptions surfaceOptions(const PreviousSurface& previous, const GridSignature& now, size_t currentCoarseColumns) {
		// counts and switches: exactly
		const auto same = [&](const char* name, double current) {
			const auto found = previous.settings.find(name);
			return found != previous.settings.end() && std::abs(found->second - current) < 1e-9;
		};
		// Positions [um]: within 0.05 um. The GUI keeps one decimal of the limits (a grid moved by the drift is rounded to it), and a switch
		// between absolute and relative coordinates converts them in floating point - neither is another grid on the sample.
		const auto shifted = [&](const char* name, double current, double shift) {
			const auto found = previous.settings.find(name);
			return found != previous.settings.end() && std::abs(found->second + shift - current) < 0.051;
		};
		SurfaceOptions options;
		options.xyGridSame = shifted("xMin", now.xMin, now.shiftX) && shifted("xMax", now.xMax, now.shiftX) && same("xSteps", now.xSteps)
			&& shifted("yMin", now.yMin, now.shiftY) && shifted("yMax", now.yMax, now.shiftY)
			&& same("ySteps", now.ySteps) && same("gridCoordinatesAbsolute", now.absolute ? 1.0 : 0.0) && same("preScanXYBin", now.preScanXYBin)
			&& (!now.absolute || (shifted("absoluteGridOriginUm.x", now.originX, 0.0) && shifted("absoluteGridOriginUm.y", now.originY, 0.0)));
		options.zGridSame = shifted("zMin", now.zMin, 0.0) && shifted("zMax", now.zMax, 0.0) && same("zSteps", now.zSteps);
		// which settings differ, for the log (empty if the grid is the same)
		for (const char* name : { "xSteps", "ySteps", "gridCoordinatesAbsolute", "preScanXYBin" }) {
			const auto found = previous.settings.find(name);
			if (found == previous.settings.end()) {
				options.differences += std::string(options.differences.empty() ? "" : ", ") + name + " (not in the file)";
			}
		}
		const auto describe = [&](const char* name, double current, double shift) {
			const auto found = previous.settings.find(name);
			if (found != previous.settings.end() && std::abs(found->second + shift - current) >= 0.051) {
				options.differences += std::string(options.differences.empty() ? "" : ", ") + name + " " + std::to_string(found->second)
					+ (shift != 0.0 ? " (+ " + std::to_string(shift) + " drift)" : std::string{}) + " -> " + std::to_string(current);
			}
		};
		describe("xMin", now.xMin, now.shiftX);
		describe("xMax", now.xMax, now.shiftX);
		describe("yMin", now.yMin, now.shiftY);
		describe("yMax", now.yMax, now.shiftY);
		describe("xSteps", now.xSteps, 0.0);
		describe("ySteps", now.ySteps, 0.0);
		describe("preScanXYBin", now.preScanXYBin, 0.0);
		describe("gridCoordinatesAbsolute", now.absolute ? 1.0 : 0.0, 0.0);
		if (now.absolute) {
			describe("absoluteGridOriginUm.x", now.originX, 0.0);
			describe("absoluteGridOriginUm.y", now.originY, 0.0);
		}
		options.priorCompatible = options.xyGridSame && previous.z.size() == currentCoarseColumns;
		options.canReuse = previous.hasPlan && options.xyGridSame && options.zGridSame;
		return options;
	}

	// What is done with the operator's answer (0 new scan, 1 start from the earlier scan, 2 reuse it; -1 none at all): the default is to start
	// from the earlier scan if that is possible, and an answer that is not possible is a new scan.
	inline int resolveSurfaceAnswer(int answer, const SurfaceOptions& options) {
		const auto chosen = answer >= 0 ? answer : (options.priorCompatible ? 1 : 0);
		if (chosen == 1 && options.priorCompatible) {
			return 1;
		}
		if (chosen == 2 && options.canReuse) {
			return 2;
		}
		return 0;
	}

}

#endif // RESUMEFILE_H

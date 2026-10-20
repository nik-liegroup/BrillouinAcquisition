#include "stdafx.h"
#include "BrillouinResume.h"

#include <cmath>
#include <type_traits>

/*
 * The settings of a Brillouin measurement as a flat record (names -> numbers / text), so that every measurement can write ALL of its settings
 * to the file and a later "continue" can restore exactly them. One visitor walks the settings: the writer collects them into the record, the
 * reader sets them from it. A name missing from the record leaves the setting as it is (files written by another version).
 */

namespace resume {

	std::string toUtf8(const std::wstring& text) {
		std::string out;
		for (const auto c : text) {
			const auto code = (unsigned int)c;
			if (code < 0x80) {
				out.push_back((char)code);
			} else if (code < 0x800) {
				out.push_back((char)(0xC0 | (code >> 6)));
				out.push_back((char)(0x80 | (code & 0x3F)));
			} else {
				out.push_back((char)(0xE0 | (code >> 12)));
				out.push_back((char)(0x80 | ((code >> 6) & 0x3F)));
				out.push_back((char)(0x80 | (code & 0x3F)));
			}
		}
		return out;
	}

	std::wstring fromUtf8(const std::string& text) {
		std::wstring out;
		for (size_t i = 0; i < text.size();) {
			const auto b = (unsigned char)text[i];
			if (b < 0x80) {
				out.push_back((wchar_t)b);
				i += 1;
			} else if ((b & 0xE0) == 0xC0 && i + 1 < text.size()) {
				out.push_back((wchar_t)(((b & 0x1F) << 6) | ((unsigned char)text[i + 1] & 0x3F)));
				i += 2;
			} else if ((b & 0xF0) == 0xE0 && i + 2 < text.size()) {
				out.push_back((wchar_t)(((b & 0x0F) << 12) | (((unsigned char)text[i + 1] & 0x3F) << 6) | ((unsigned char)text[i + 2] & 0x3F)));
				i += 3;
			} else {
				out.push_back(L'?');
				i += 1;
			}
		}
		return out;
	}

	namespace {

		template <typename T>
		void assignNumber(T& target, double value) {
			if constexpr (std::is_same_v<T, bool>) {
				target = value != 0.0;
			} else if constexpr (std::is_enum_v<T>) {
				target = static_cast<T>((long long)std::llround(value));
			} else if constexpr (std::is_integral_v<T>) {
				target = static_cast<T>(std::llround(value));
			} else {
				target = static_cast<T>(value);
			}
		}

		class Writer {
		public:
			explicit Writer(Record& record) : m_record(record) {}
			template <typename Setter>
			void number(const char* name, double value, Setter&&) { m_record.numbers[name] = { value }; }
			template <typename Setter>
			void numbers(const char* name, const std::vector<double>& values, Setter&&) { m_record.numbers[name] = values; }
			template <typename Setter>
			void text(const char* name, const std::string& value, Setter&&) { m_record.strings[name] = value; }
		private:
			Record& m_record;
		};

		class Reader {
		public:
			explicit Reader(const Record& record) : m_record(record) {}
			template <typename Setter>
			void number(const char* name, double, Setter&& setter) {
				const auto found = m_record.numbers.find(name);
				if (found != m_record.numbers.end() && !found->second.empty()) {
					setter(found->second.front());
					m_applied++;
				}
			}
			template <typename Setter>
			void numbers(const char* name, const std::vector<double>&, Setter&& setter) {
				const auto found = m_record.numbers.find(name);
				if (found != m_record.numbers.end()) {
					setter(found->second);
					m_applied++;
				}
			}
			template <typename Setter>
			void text(const char* name, const std::string&, Setter&& setter) {
				const auto found = m_record.strings.find(name);
				if (found != m_record.strings.end()) {
					setter(found->second);
					m_applied++;
				}
			}
			int applied() const { return m_applied; }
		private:
			const Record& m_record;
			int m_applied{ 0 };
		};

		// the names of the settings of one overview objective: stored as one array per property over all objectives
		struct OverviewArrays {
			std::vector<double> slot, active, exposureMs, gain, fullGrid, fullStack, stackSteps, stackRangeUm;
		};

		template <typename V>
		void visitSettings(BRILLOUIN_SETTINGS& s, V& v) {
#define FIELD(member) v.number(#member, (double)s.member, [&](double x) { assignNumber(s.member, x); })
			v.number("xMin", s.xMin, [&](double x) { s.setXMin(x); });
			v.number("xMax", s.xMax, [&](double x) { s.setXMax(x); });
			v.number("xSteps", s.xSteps, [&](double x) { s.setXSteps((int)std::llround(x)); });
			v.number("yMin", s.yMin, [&](double x) { s.setYMin(x); });
			v.number("yMax", s.yMax, [&](double x) { s.setYMax(x); });
			v.number("ySteps", s.ySteps, [&](double x) { s.setYSteps((int)std::llround(x)); });
			v.number("zMin", s.zMin, [&](double x) { s.setZMin(x); });
			v.number("zMax", s.zMax, [&](double x) { s.setZMax(x); });
			v.number("zSteps", s.zSteps, [&](double x) { s.setZSteps((int)std::llround(x)); });
			v.text("sample", s.sample, [&](const std::string& x) { s.sample = x; });
	FIELD(preCalibration);
	FIELD(postCalibration);
	FIELD(conCalibration);
	FIELD(conCalibrationInterval);
	FIELD(nrCalibrationImages);
	FIELD(calibrationExposureTime);
	FIELD(useRoiMask);
	FIELD(useBackgroundRoiMask);
	FIELD(useSurfaceFollow);
	FIELD(surfaceFollowHalfRangeUm);
	FIELD(surfaceMaxRewindUm);
	FIELD(surfaceSearchBelowZeroUm);
	FIELD(surfaceVerificationSteps);
	FIELD(surfaceVerificationFrameAverage);
	FIELD(surfaceVerificationToleranceFraction);
	FIELD(preScanXYBin);
	FIELD(additionalBoundaryPoints);
	FIELD(preScanZStepUm);
	FIELD(preScanZTravelRangeUm);
	FIELD(preScanXSteps);
	FIELD(preScanYSteps);
	FIELD(preScanZSteps);
	FIELD(preScanZMin);
	FIELD(preScanZMax);
	FIELD(surfaceMetricThreshold);
	FIELD(surfaceDropFraction);
	FIELD(surfaceScanDirection);
	FIELD(surfaceProxyRoiLeft);
	FIELD(surfaceProxyRoiTop);
	FIELD(surfaceProxyRoiWidth);
	FIELD(surfaceProxyRoiHeight);
	FIELD(surfaceProxyRoi2Left);
	FIELD(surfaceProxyRoi2Top);
	FIELD(surfaceProxyRoi2Width);
	FIELD(surfaceProxyRoi2Height);
	FIELD(surfaceProxyRoiFrameWidth);
	FIELD(surfaceProxyRoiFrameHeight);
	FIELD(surfaceProxyRoiFrameOriginLeft);
	FIELD(surfaceProxyRoiFrameOriginBottom);
	FIELD(surfaceProxyRoiFrameWidthPhysical);
	FIELD(surfaceProxyRoiFrameHeightPhysical);
	FIELD(surfaceProxyRoi2FrameWidth);
	FIELD(surfaceProxyRoi2FrameHeight);
	FIELD(surfaceProxyRoi2FrameOriginLeft);
	FIELD(surfaceProxyRoi2FrameOriginBottom);
	FIELD(surfaceProxyRoi2FrameWidthPhysical);
	FIELD(surfaceProxyRoi2FrameHeightPhysical);
	FIELD(mediumReferenceValue);
	FIELD(mediumReferenceFrameCount);
	FIELD(gridCoordinatesAbsolute);
	FIELD(saveOverviewBrightfieldPerZ);
	FIELD(overviewBrightfieldExposureMs);
	FIELD(overviewBrightfieldGain);
	FIELD(overviewBrightfieldFullGrid);
	FIELD(overviewBrightfieldFullStackSingle);
	FIELD(overviewBrightfieldFullStackMosaic);
	FIELD(overviewTrigger);
	FIELD(overviewBeforeMeasurement);
	FIELD(overviewAfterMeasurement);
	FIELD(overviewObjectiveRetractUm);
	FIELD(driftCorrectionEnabled);
	FIELD(driftObjectiveSlot);
	FIELD(driftMinNcc);
	FIELD(driftMaxStepUm);
	FIELD(driftMaxSpeedUmPerMin);
	FIELD(driftExtrapolate);
	FIELD(surfaceRescan);
	FIELD(capturePerPointBrightfield);
	FIELD(perPointBrightfieldEveryN);
	FIELD(perPointBrightfieldDuringAcquisition);
	FIELD(useGridHysteresisCompensation);
	FIELD(useDoseProtection);
	FIELD(extendFramesOnLowSignal);
	FIELD(proxyRoiMetricThreshold);
	FIELD(maxAdditionalFrames);
	FIELD(livePreviews);
			v.number("repetitions.count", s.repetitions.count, [&](double x) { s.repetitions.count = (int)std::llround(x); });
			v.number("repetitions.interval", s.repetitions.interval, [&](double x) { s.repetitions.interval = x; });
			v.number("repetitions.filePerRepetition", s.repetitions.filePerRepetition, [&](double x) { s.repetitions.filePerRepetition = x != 0.0; });
			v.number("absoluteGridOriginUm.x", s.absoluteGridOriginUm.x, [&](double x) { s.absoluteGridOriginUm.x = x; });
			v.number("absoluteGridOriginUm.y", s.absoluteGridOriginUm.y, [&](double x) { s.absoluteGridOriginUm.y = x; });
			v.number("absoluteGridOriginUm.z", s.absoluteGridOriginUm.z, [&](double x) { s.absoluteGridOriginUm.z = x; });

			const auto polygon = [&v](const char* name, std::vector<POINT2>& points) {
				std::vector<double> flat;
				for (const auto& point : points) {
					flat.push_back(point.x);
					flat.push_back(point.y);
				}
				v.numbers(name, flat, [&points](const std::vector<double>& values) {
					points.clear();
					for (size_t i = 0; i + 1 < values.size(); i += 2) {
						points.push_back(POINT2{ values[i], values[i + 1] });
					}
				});
			};
			polygon("roiPolygonUm", s.roiPolygonUm);
			polygon("backgroundRoiPolygonUm", s.backgroundRoiPolygonUm);

			OverviewArrays arrays;
			for (const auto& entry : s.overviewObjectives) {
				arrays.slot.push_back(entry.slot);
				arrays.active.push_back(entry.active ? 1.0 : 0.0);
				arrays.exposureMs.push_back(entry.exposureMs);
				arrays.gain.push_back(entry.gain);
				arrays.fullGrid.push_back(entry.fullGrid ? 1.0 : 0.0);
				arrays.fullStack.push_back(entry.fullStack ? 1.0 : 0.0);
				arrays.stackSteps.push_back(entry.stackSteps);
				arrays.stackRangeUm.push_back(entry.stackRangeUm);
			}
			// the arrays are applied together when the slot array arrives (it defines how many objectives there are)
			v.numbers("overviewObjectives.slot", arrays.slot, [&](const std::vector<double>& values) {
				s.overviewObjectives.assign(values.size(), OVERVIEW_OBJECTIVE_SETTINGS{});
				for (size_t i = 0; i < values.size(); i++) {
					s.overviewObjectives[i].slot = (int)std::llround(values[i]);
				}
			});
			const auto objectiveProperty = [&](const char* name, const std::vector<double>& values, auto assign) {
				v.numbers(name, values, [&s, assign](const std::vector<double>& loaded) {
					for (size_t i = 0; i < loaded.size() && i < s.overviewObjectives.size(); i++) {
						assign(s.overviewObjectives[i], loaded[i]);
					}
				});
			};
			objectiveProperty("overviewObjectives.active", arrays.active, [](OVERVIEW_OBJECTIVE_SETTINGS& e, double x) { e.active = x != 0.0; });
			objectiveProperty("overviewObjectives.exposureMs", arrays.exposureMs, [](OVERVIEW_OBJECTIVE_SETTINGS& e, double x) { e.exposureMs = (int)std::llround(x); });
			objectiveProperty("overviewObjectives.gain", arrays.gain, [](OVERVIEW_OBJECTIVE_SETTINGS& e, double x) { e.gain = x; });
			objectiveProperty("overviewObjectives.fullGrid", arrays.fullGrid, [](OVERVIEW_OBJECTIVE_SETTINGS& e, double x) { e.fullGrid = x != 0.0; });
			objectiveProperty("overviewObjectives.fullStack", arrays.fullStack, [](OVERVIEW_OBJECTIVE_SETTINGS& e, double x) { e.fullStack = x != 0.0; });
			objectiveProperty("overviewObjectives.stackSteps", arrays.stackSteps, [](OVERVIEW_OBJECTIVE_SETTINGS& e, double x) { e.stackSteps = (int)std::llround(x); });
			objectiveProperty("overviewObjectives.stackRangeUm", arrays.stackRangeUm, [](OVERVIEW_OBJECTIVE_SETTINGS& e, double x) { e.stackRangeUm = x; });
#undef FIELD

			// the camera of the Brillouin measurement
			auto& c = s.camera;
			v.number("camera.exposureTime", c.exposureTime, [&](double x) { c.exposureTime = x; });
			v.number("camera.frameCount", (double)c.frameCount, [&](double x) { c.frameCount = (long long)std::llround(x); });
			v.number("camera.gain", c.gain, [&](double x) { c.gain = x; });
			v.number("camera.spuriousNoiseFilter", c.spuriousNoiseFilter, [&](double x) { c.spuriousNoiseFilter = x != 0.0; });
#define ROI(member) v.number("camera.roi." #member, (double)c.roi.member, [&](double x) { c.roi.member = (long long)std::llround(x); })
			ROI(left); ROI(right); ROI(width_physical); ROI(width_binned); ROI(top); ROI(bottom);
			ROI(height_physical); ROI(height_binned); ROI(binX); ROI(binY);
#undef ROI
			v.number("camera.roi.bytesPerFrame", c.roi.bytesPerFrame, [&](double x) { c.roi.bytesPerFrame = (int)std::llround(x); });
			v.text("camera.roi.binning", toUtf8(c.roi.binning), [&](const std::string& x) { c.roi.binning = fromUtf8(x); });
#define READOUT(member) v.text("camera.readout." #member, toUtf8(c.readout.member), [&](const std::string& x) { c.readout.member = fromUtf8(x); })
			READOUT(pixelReadoutRate); READOUT(pixelEncoding); READOUT(cycleMode); READOUT(triggerMode); READOUT(preAmpGain);
#undef READOUT
			v.text("camera.readout.dataType", c.readout.dataType, [&](const std::string& x) { c.readout.dataType = x; });
		}

	}

	Record encodeSettings(const BRILLOUIN_SETTINGS& settings) {
		// not copy-constructed: the settings hold references to their own members, only operator= copies correctly
		BRILLOUIN_SETTINGS copy;
		copy = settings;
		Record record;
		Writer writer(record);
		visitSettings(copy, writer);
		return record;
	}

	int decodeSettings(const Record& record, BRILLOUIN_SETTINGS* settings) {
		Reader reader(record);
		visitSettings(*settings, reader);
		return reader.applied();
	}

	std::vector<std::string> settingNames(bool numbers) {
		BRILLOUIN_SETTINGS defaults;
		const auto record = encodeSettings(defaults);
		std::vector<std::string> names;
		if (numbers) {
			for (const auto& entry : record.numbers) {
				names.push_back(entry.first);
			}
		} else {
			for (const auto& entry : record.strings) {
				names.push_back(entry.first);
			}
		}
		return names;
	}

}

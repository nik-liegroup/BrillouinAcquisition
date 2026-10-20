#include "stdafx.h"
#include "drift.h"

#include <algorithm>
#include <limits>

namespace drift {

	StageMap stageMapFromCalibration(POINT2 micrometerToPixX, POINT2 micrometerToPixY) {
		// pixel = x * micrometerToPixX + y * micrometerToPixY in the live view, entries ordered (column, row); the stored image has the
		// rows running the other way, so the row component changes sign
		StageMap map;
		map.colX = micrometerToPixX.x;
		map.colY = micrometerToPixY.x;
		map.rowX = -micrometerToPixX.y;
		map.rowY = -micrometerToPixY.y;
		return map;
	}

	POINT2 shiftToDriftUm(const StageMap& map, double shiftCol, double shiftRow, POINT2 positionMovedUm, POINT2 positionReferenceUm) {
		const auto determinant = map.determinant();
		if (!map.isValid()) {
			return POINT2{ std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN() };
		}
		// map^-1 * (shift)
		const auto x = (map.rowY * shiftCol - map.colY * shiftRow) / determinant;
		const auto y = (-map.rowX * shiftCol + map.colX * shiftRow) / determinant;
		// A frame captured with the stage elsewhere shows the sample shifted by that move even without any drift.
		return POINT2{ x + (positionMovedUm.x - positionReferenceUm.x), y + (positionMovedUm.y - positionReferenceUm.y) };
	}

	namespace {

		double median(std::vector<double> values) {
			std::sort(values.begin(), values.end());
			const auto n = values.size();
			if (n == 0) {
				return 0.0;
			}
			return n % 2 == 1 ? values[n / 2] : 0.5 * (values[n / 2 - 1] + values[n / 2]);
		}

	}

	StepMeasurement measureStep(
		const std::vector<TileFrame>& reference, const std::vector<TileFrame>& moved, const StageMap& map,
		double minNcc, double maxShiftUm, double maxTileDistanceUm, double outlierUm
	) {
		StepMeasurement result;
		if (!map.isValid() || reference.empty() || moved.empty()) {
			return result;
		}
		const auto maxShiftRawPix = maxShiftUm * map.pixelsPerUm();

		struct TileStep {
			POINT2 drift;
			double ncc;
		};
		std::vector<TileStep> steps;
		for (const auto& tile : moved) {
			if (!tile.image.isValid()) {
				continue;
			}
			// the reference tile at (nearly) the same stage position
			const TileFrame* nearest = nullptr;
			auto nearestDistance = std::numeric_limits<double>::infinity();
			for (const auto& candidate : reference) {
				if (!candidate.image.isValid()) {
					continue;
				}
				const auto distance = std::hypot(candidate.pairingPositionUm().x - tile.pairingPositionUm().x,
						candidate.pairingPositionUm().y - tile.pairingPositionUm().y);
				if (distance < nearestDistance) {
					nearestDistance = distance;
					nearest = &candidate;
				}
			}
			if (!nearest || nearestDistance > maxTileDistanceUm || nearest->image.factor != tile.image.factor) {
				continue;
			}
			const auto match = registration::match(nearest->image, tile.image, maxShiftRawPix);
			result.tilesCompared++;
			if (!match.found) {
				continue;
			}
			result.maxNcc = std::max(result.maxNcc, match.ncc);
			if (match.ncc < minNcc) {
				continue;
			}
			steps.push_back(TileStep{
				shiftToDriftUm(map, match.shiftCol, match.shiftRow, tile.positionUm, nearest->positionUm), match.ncc });
		}
		if (steps.empty()) {
			return result;
		}

		std::vector<double> xs;
		std::vector<double> ys;
		for (const auto& step : steps) {
			xs.push_back(step.drift.x);
			ys.push_back(step.drift.y);
		}
		const auto medianX = median(xs);
		const auto medianY = median(ys);
		std::vector<TileStep> kept;
		for (const auto& step : steps) {
			if (std::hypot(step.drift.x - medianX, step.drift.y - medianY) <= outlierUm) {
				kept.push_back(step);
			}
		}
		if (kept.empty()) {
			kept = steps;
		}
		auto sumX = 0.0;
		auto sumY = 0.0;
		auto sumWeight = 0.0;
		auto sumNcc = 0.0;
		for (const auto& step : kept) {
			sumX += step.ncc * step.drift.x;
			sumY += step.ncc * step.drift.y;
			sumWeight += step.ncc;
			sumNcc += step.ncc;
		}
		if (sumWeight <= 0.0) {
			return result;
		}
		result.valid = true;
		result.driftUm = POINT2{ sumX / sumWeight, sumY / sumWeight };
		result.tilesUsed = (int)kept.size();
		result.meanNcc = sumNcc / (double)kept.size();
		return result;
	}

	void DriftController::reset() {
		m_hasReference = false;
		m_referenceTime = 0.0;
		m_total = POINT2{ 0.0, 0.0 };
		m_lastStep = POINT2{ 0.0, 0.0 };
		m_lastStepSeconds = 0.0;
		m_hasLastStep = false;
	}

	DriftUpdate DriftController::setReference(double timeSeconds, POINT2 carriedOverUm) {
		reset();
		m_hasReference = true;
		m_referenceTime = timeSeconds;
		m_total = carriedOverUm;
		DriftUpdate update;
		update.decision = DriftDecision::REFERENCE;
		update.totalUm = m_total;
		return update;
	}

	DriftUpdate DriftController::update(const StepMeasurement& measurement, double timeSeconds) {
		if (!m_hasReference) {
			return setReference(timeSeconds);
		}
		DriftUpdate update;
		update.totalUm = m_total;
		update.stepUm = measurement.valid ? measurement.driftUm : POINT2{ 0.0, 0.0 };
		update.meanNcc = measurement.valid ? measurement.meanNcc : measurement.maxNcc;
		update.tilesUsed = measurement.tilesUsed;
		const auto seconds = timeSeconds - m_referenceTime;
		update.secondsSinceReference = seconds;

		if (!measurement.valid) {
			update.decision = measurement.tilesCompared == 0 ? DriftDecision::REJECTED_NO_MATCH : DriftDecision::REJECTED_LOW_CONFIDENCE;
			return update;
		}
		if (measurement.meanNcc < m_settings.minNcc) {
			update.decision = DriftDecision::REJECTED_LOW_CONFIDENCE;
			return update;
		}
		const auto stepLength = std::hypot(measurement.driftUm.x, measurement.driftUm.y);
		if (!std::isfinite(stepLength) || stepLength > m_settings.maxStepUm) {
			update.decision = DriftDecision::REJECTED_IMPLAUSIBLE;
			return update;
		}

		m_total = POINT2{ m_total.x + measurement.driftUm.x, m_total.y + measurement.driftUm.y };
		m_lastStep = measurement.driftUm;
		m_lastStepSeconds = seconds;
		m_hasLastStep = seconds > 0.0;
		m_referenceTime = timeSeconds;
		update.decision = DriftDecision::APPLIED;
		update.totalUm = m_total;
		return update;
	}

	POINT2 DriftController::offsetAt(double timeSeconds) const {
		if (!m_settings.extrapolate || !m_hasLastStep || m_lastStepSeconds <= 0.0) {
			return m_total;
		}
		const auto elapsed = std::clamp(timeSeconds - m_referenceTime, 0.0, 2.0 * m_lastStepSeconds);
		auto extraX = m_lastStep.x / m_lastStepSeconds * elapsed;
		auto extraY = m_lastStep.y / m_lastStepSeconds * elapsed;
		const auto length = std::hypot(extraX, extraY);
		if (length > m_settings.maxStepUm && length > 0.0) {
			extraX *= m_settings.maxStepUm / length;
			extraY *= m_settings.maxStepUm / length;
		}
		return POINT2{ m_total.x + extraX, m_total.y + extraY };
	}

}

#pragma once

#include "ScaleCalibrationHelper.h"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <array>
#include <limits>

struct FovRegistrationResult {
	POINT2 imageTranslation; // r_target = M_image*r_reference + translation
	double correlation{ 0 };
	cv::Mat overlay;
};

// Refine either a minimum or maximum without quantizing the stage calibration
// to whole camera pixels. At a boundary or a flat/invalid peak keep the integer fit.
inline cv::Point2d subpixelMatchLocation(const cv::Mat& scores, cv::Point peak) {
	auto refine = [](double left, double center, double right) {
		const double curvature = left - 2 * center + right;
		if (!std::isfinite(curvature) || std::abs(curvature) < 1e-12) return 0.0;
		return std::clamp(0.5 * (left - right) / curvature, -0.5, 0.5);
	};
	cv::Point2d point(peak.x, peak.y);
	if (peak.x > 0 && peak.x + 1 < scores.cols) {
		point.x += refine(scores.at<float>(peak.y, peak.x - 1), scores.at<float>(peak.y, peak.x), scores.at<float>(peak.y, peak.x + 1));
	}
	if (peak.y > 0 && peak.y + 1 < scores.rows) {
		point.y += refine(scores.at<float>(peak.y - 1, peak.x), scores.at<float>(peak.y, peak.x), scores.at<float>(peak.y + 1, peak.x));
	}
	return point;
}

// Match in the finer camera's pixel frame. The larger physical FOV supplies the
// search image; warping uses the FULL matrix also used by the grid projection.
inline bool registerObjectiveImages(const cv::Mat& reference, const cv::Mat& target,
	const ScaleCalibrationData& referenceScale, const ScaleCalibrationData& targetScale,
	FovRegistrationResult& result, std::string& error) {
	if (reference.empty() || target.empty()
		|| !ScaleCalibrationHelper::isBasis(referenceScale.pixToMicrometerX, referenceScale.pixToMicrometerY)
		|| !ScaleCalibrationHelper::isBasis(targetScale.pixToMicrometerX, targetScale.pixToMicrometerY)) {
		error = "Both images and valid pixel-scale calibrations are required.";
		return false;
	}
	auto pitchR = ScaleCalibrationHelper::isotropicPixelPitchUm(referenceScale);
	auto pitchT = ScaleCalibrationHelper::isotropicPixelPitchUm(targetScale);
	const bool reverse = reference.total() * pitchR * pitchR < target.total() * pitchT * pitchT;
	const auto& searchSource = reverse ? target : reference;
	const auto& templateSource = reverse ? reference : target;
	const auto& searchScale = reverse ? targetScale : referenceScale;
	const auto& templateScale = reverse ? referenceScale : targetScale;
	auto m = ScaleCalibrationHelper::imageLinearTransform(searchScale, templateScale);
	auto map = [m](double x, double y) { return cv::Point2d(m.a * x + m.b * y, m.c * x + m.d * y); };
	const std::array<cv::Point2d, 4> corners{
		map(0, 0), map(searchSource.cols - 1, 0),
		map(0, searchSource.rows - 1), map(searchSource.cols - 1, searchSource.rows - 1)
	};
	double left = corners[0].x, right = left, top = corners[0].y, bottom = top;
	for (auto p : corners) {
		if (!std::isfinite(p.x) || !std::isfinite(p.y)) {
			error = "The pixel-scale transform contains non-finite values.";
			return false;
		}
		left = std::min(left, p.x); right = std::max(right, p.x);
		top = std::min(top, p.y); bottom = std::max(bottom, p.y);
	}
	left = std::floor(left); top = std::floor(top);
	const auto width = std::ceil(right) - left + 1, height = std::ceil(bottom) - top + 1;
	if (width < 1 || height < 1 || width > 32766 || height > 32766 || width * height > 64000000) {
		error = "The calibrated image transform is too large; check both pixel scales.";
		return false;
	}
	// Canvas shift b=(-left,-top): q = M*r+b. Keep it when recovering translation.
	cv::Mat affine = (cv::Mat_<double>(2, 3) << m.a, m.b, -left, m.c, m.d, -top);
	cv::Mat search, valid;
	cv::warpAffine(searchSource, search, affine, cv::Size((int)width, (int)height), cv::INTER_LINEAR);
	cv::warpAffine(cv::Mat(searchSource.size(), CV_8U, cv::Scalar(255)), valid,
		affine, search.size(), cv::INTER_LINEAR);
	// Exclude interpolation against padding. A high match against black warp borders
	// must never be accepted as a real correspondence.
	cv::compare(valid, 255, valid, cv::CMP_EQ);
	valid /= 255;
	cv::Mat integral;
	cv::integral(valid, integral, CV_32S);
	cv::Point match;
	cv::Rect crop(0, 0, templateSource.cols, templateSource.rows);
	double score = -std::numeric_limits<double>::infinity();
	bool found = false;
	cv::Mat bestScores;
	// Normally use the entire target. A centred crop leaves translation search room
	// for equal-FOV pairs or rotations whose valid footprint cannot contain it whole.
	for (double fraction : { 1.0, 0.75, 0.5 }) {
		const int w = (int)(templateSource.cols * fraction), h = (int)(templateSource.rows * fraction);
		if (w < 16 || h < 16 || w > search.cols || h > search.rows) continue;
		crop = cv::Rect((templateSource.cols - w) / 2, (templateSource.rows - h) / 2, w, h);
		cv::Scalar mean, deviation;
		cv::meanStdDev(templateSource(crop), mean, deviation);
		if (deviation[0] < 1e-6) continue;
		cv::Mat scores;
		cv::matchTemplate(search, templateSource(crop), scores, cv::TM_CCOEFF_NORMED);
		for (int y = 0; y < scores.rows; ++y) {
			const int* row0 = integral.ptr<int>(y);
			const int* row1 = integral.ptr<int>(y + h);
			const float* rowScore = scores.ptr<float>(y);
			for (int x = 0; x < scores.cols; ++x) {
				const int count = row1[x + w] - row1[x] - row0[x + w] + row0[x];
				if (count == w * h && std::isfinite(rowScore[x]) && rowScore[x] > score) {
					score = rowScore[x]; match = cv::Point(x, y); found = true;
				}
			}
		}
		if (found && score >= 0.5) { bestScores = scores; break; }
		found = false;
		score = -std::numeric_limits<double>::infinity();
	}
	if (!found || score < 0.5) {
		error = "No reliable overlapping image match was found. Check focus, image structure and pixel scales.";
		return false;
	}
	cv::Point2d refined(match.x, match.y);
	if (match.x > 0 && match.y > 0 && match.x + 1 < bestScores.cols && match.y + 1 < bestScores.rows) {
		const int x = match.x - 1, y = match.y - 1, w = crop.width + 2, h = crop.height + 2;
		const int count = integral.at<int>(y + h, x + w) - integral.at<int>(y + h, x)
			- integral.at<int>(y, x + w) + integral.at<int>(y, x);
		if (count == w * h) refined = subpixelMatchLocation(bestScores, match);
	}
	POINT2 translation{ -left + crop.x - refined.x, -top + crop.y - refined.y };
	if (reverse) {
		auto forward = ScaleCalibrationHelper::imageLinearTransform(referenceScale, targetScale);
		result.imageTranslation = { -forward.a * translation.x - forward.b * translation.y,
			-forward.c * translation.x - forward.d * translation.y };
	} else {
		result.imageTranslation = translation;
	}
	result.correlation = score;
	// Show the match in the original target frame, including for reversed searches.
	auto forward = ScaleCalibrationHelper::imageLinearTransform(referenceScale, targetScale);
	cv::Mat toTarget = (cv::Mat_<double>(2, 3) << forward.a, forward.b, result.imageTranslation.x,
		forward.c, forward.d, result.imageTranslation.y);
	cv::Mat registered, mask;
	cv::warpAffine(reference, registered, toTarget, target.size(), cv::INTER_LINEAR);
	cv::warpAffine(cv::Mat(reference.size(), CV_8U, cv::Scalar(255)), mask, toTarget, target.size(), cv::INTER_NEAREST);
	result.overlay = target.clone();
	cv::Mat blended;
	cv::addWeighted(registered, 0.5, target, 0.5, 0, blended);
	blended.copyTo(result.overlay, mask);
	return true;
}

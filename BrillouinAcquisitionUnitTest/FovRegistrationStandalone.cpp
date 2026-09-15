// Standalone regression for the production OpenCV registration implementation.
// Build with the project's OpenCV include/library paths (no Qt or microscope SDK).
#include "../BrillouinAcquisition/src/Acquisition/AcquisitionModes/FovRegistration.h"
#include <iostream>
#include <stdexcept>

static void require(bool condition, const char* message) {
	if (!condition) throw std::runtime_error(message);
}

static void exercise(Matrix2 imageMatrix, bool reverseInput, cv::Size targetSize) {
	ScaleCalibrationData reference, target;
	reference.pixToMicrometerX = { 0, -.47 };
	reference.pixToMicrometerY = { .46, 0 };
	// A_target = A_reference * inverse(D*M_image*D).
	auto inverse = ScaleCalibrationHelper::invert({ imageMatrix.a, -imageMatrix.b, -imageMatrix.c, imageMatrix.d });
	target.pixToMicrometerX = { .46 * inverse.c, -.47 * inverse.a };
	target.pixToMicrometerY = { .46 * inverse.d, -.47 * inverse.b };
	ScaleCalibrationHelper::initializeCalibrationFromPixel(&reference);
	ScaleCalibrationHelper::initializeCalibrationFromPixel(&target);
	cv::Mat source(512, 640, CV_8U);
	cv::RNG random(73);
	random.fill(source, cv::RNG::UNIFORM, 0, 255);
	cv::GaussianBlur(source, source, cv::Size(5, 5), 1.1);
	POINT2 translation{ (targetSize.width - 1) / 2.0 - imageMatrix.a * 319.5 - imageMatrix.b * 255.5 + 11.3,
		(targetSize.height - 1) / 2.0 - imageMatrix.c * 319.5 - imageMatrix.d * 255.5 - 7.2 };
	cv::Mat affine = (cv::Mat_<double>(2, 3) << imageMatrix.a, imageMatrix.b, translation.x,
		imageMatrix.c, imageMatrix.d, translation.y);
	cv::Mat observed;
	cv::warpAffine(source, observed, affine, targetSize, cv::INTER_LINEAR);
	FovRegistrationResult result;
	std::string error;
	if (reverseInput) {
		if (!registerObjectiveImages(observed, source, target, reference, result, error)) throw std::runtime_error(error);
		auto inv = ScaleCalibrationHelper::invert(imageMatrix);
		translation = { -inv.a * translation.x - inv.b * translation.y,
			-inv.c * translation.x - inv.d * translation.y };
	} else {
		if (!registerObjectiveImages(source, observed, reference, target, result, error)) throw std::runtime_error(error);
	}
	require(std::abs(result.imageTranslation.x - translation.x) < .6
		&& std::abs(result.imageTranslation.y - translation.y) < .6, "Wrong image translation");
	require(result.correlation > .8, "Synthetic match should have high correlation");
	require(result.overlay.size() == (reverseInput ? source.size() : observed.size()), "Overlay frame mismatch");
}

int main() {
	try {
		for (bool reverse : { false, true }) {
			exercise({ 2, 0, 0, 2 }, reverse, { 320, 256 });
			exercise({ 2.1, .2, -.1, 1.8 }, reverse, { 320, 256 });
			exercise({ 0, -2, 2, 0 }, reverse, { 320, 256 });
			exercise({ -2, 0, 0, 2 }, reverse, { 320, 256 });
			exercise({ 1, 0, 0, 1 }, reverse, { 640, 512 }); // crop fallback
		}
		cv::Mat flat(64, 64, CV_8U, cv::Scalar(100));
		ScaleCalibrationData scale;
		scale.pixToMicrometerX = { 1, 0 };
		scale.pixToMicrometerY = { 0, 1 };
		FovRegistrationResult result;
		std::string error;
		require(!registerObjectiveImages(flat, flat, scale, scale, result, error), "Flat images must fail");
		// Also test a known quadratic minimum: the scale-calibration peak refiner.
		cv::Mat scores(9, 9, CV_32F);
		for (int y = 0; y < scores.rows; ++y)
			for (int x = 0; x < scores.cols; ++x)
				scores.at<float>(y, x) = (float)((x - 4.25) * (x - 4.25) + (y - 3.7) * (y - 3.7));
		auto refined = subpixelMatchLocation(scores, { 4, 4 });
		require(std::abs(refined.x - 4.25) < 1e-5 && std::abs(refined.y - 3.7) < 1e-5, "Subpixel refinement failed");
		std::cout << "PASS: full-matrix registration, reverse order, rotations, mirrors, crop, flat rejection, subpixel fit\n";
	} catch (const std::exception& e) {
		std::cerr << e.what() << '\n';
		return 1;
	}
}

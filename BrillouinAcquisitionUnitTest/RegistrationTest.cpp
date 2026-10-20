#include "stdafx.h"
#include "CppUnitTest.h"
#include "../BrillouinAcquisition/src/lib/math/registration.h"
#include "TestScenes.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace BrillouinAcquisitionUnitTest {

	TEST_CLASS(RegistrationTest) {
	public:
		// Frames are built analytically (no file, no external tool): the scene content is shifted by a known amount and the registration
		// has to find it again.
		static constexpr int kWidth = 192;
		static constexpr int kHeight = 160;

		static registration::Match registerShifted(double shiftCol, double shiftRow, double noise = 0.0, int factor = 1) {
			const auto blobs = testscenes::makeBlobs(kWidth, kHeight, 11, 70, 4.0, 16.0);
			const auto reference = testscenes::pixelScene(blobs, kWidth, kHeight, 0.0, 0.0, noise, 1);
			const auto moved = testscenes::pixelScene(blobs, kWidth, kHeight, shiftCol, shiftRow, noise, 2);
			const auto a = registration::prepare(reference, kWidth, kHeight, factor);
			const auto b = registration::prepare(moved, kWidth, kHeight, factor);
			return registration::match(a, b, 30.0);
		}

		TEST_METHOD(Registration_whole_pixel_shift) {
			const auto result = registerShifted(7.0, -4.0);
			Assert::IsTrue(result.found);
			Assert::AreEqual(7.0, result.shiftCol, 0.1);
			Assert::AreEqual(-4.0, result.shiftRow, 0.1);
			Assert::IsTrue(result.ncc > 0.9);
		}

		TEST_METHOD(Registration_zero_shift) {
			const auto result = registerShifted(0.0, 0.0);
			Assert::IsTrue(result.found);
			Assert::AreEqual(0.0, result.shiftCol, 0.05);
			Assert::AreEqual(0.0, result.shiftRow, 0.05);
			Assert::IsTrue(result.ncc > 0.99);
		}

		TEST_METHOD(Registration_subpixel_shift) {
			const auto result = registerShifted(2.37, -1.62);
			Assert::IsTrue(result.found);
			Assert::AreEqual(2.37, result.shiftCol, 0.1);
			Assert::AreEqual(-1.62, result.shiftRow, 0.1);
		}

		TEST_METHOD(Registration_shift_in_raw_pixels_with_downsampling) {
			// the shift is reported in pixels of the raw frame, not of the downsampled one
			const auto result = registerShifted(9.0, 6.0, 0.0, 2);
			Assert::IsTrue(result.found);
			Assert::AreEqual(9.0, result.shiftCol, 0.4);
			Assert::AreEqual(6.0, result.shiftRow, 0.4);
		}

		TEST_METHOD(Registration_with_noise) {
			const auto result = registerShifted(-5.5, 3.25, 8.0);
			Assert::IsTrue(result.found);
			Assert::AreEqual(-5.5, result.shiftCol, 0.3);
			Assert::AreEqual(3.25, result.shiftRow, 0.3);
			Assert::IsTrue(result.ncc > 0.5);
		}

		TEST_METHOD(Registration_of_unrelated_scenes_has_low_correlation) {
			const auto blobsA = testscenes::makeBlobs(kWidth, kHeight, 11, 70, 4.0, 16.0);
			const auto blobsB = testscenes::makeBlobs(kWidth, kHeight, 987, 70, 4.0, 16.0);
			const auto a = registration::prepare(testscenes::pixelScene(blobsA, kWidth, kHeight, 0.0, 0.0), kWidth, kHeight, 1);
			const auto b = registration::prepare(testscenes::pixelScene(blobsB, kWidth, kHeight, 0.0, 0.0), kWidth, kHeight, 1);
			const auto result = registration::match(a, b, 30.0);
			Assert::IsTrue(!result.found || result.ncc < 0.5);
		}

		TEST_METHOD(Registration_ignores_uneven_illumination) {
			// the same scene on top of a strong extra gradient (a different lamp position) still registers
			const auto blobs = testscenes::makeBlobs(kWidth, kHeight, 11, 70, 4.0, 16.0);
			auto reference = testscenes::pixelScene(blobs, kWidth, kHeight, 0.0, 0.0);
			auto moved = testscenes::pixelScene(blobs, kWidth, kHeight, 3.0, 2.0);
			for (int row = 0; row < kHeight; row++) {
				for (int col = 0; col < kWidth; col++) {
					moved[(size_t)row * kWidth + col] += 0.3f * (float)col - 0.2f * (float)row;
				}
			}
			const auto result = registration::match(
				registration::prepare(reference, kWidth, kHeight, 1), registration::prepare(moved, kWidth, kHeight, 1), 30.0);
			Assert::IsTrue(result.found);
			Assert::AreEqual(3.0, result.shiftCol, 0.15);
			Assert::AreEqual(2.0, result.shiftRow, 0.15);
		}

		TEST_METHOD(Registration_with_an_invalid_area) {
			// the lower third of both frames has no data (zero mask), as in a mosaic edge
			const auto blobs = testscenes::makeBlobs(kWidth, kHeight, 11, 70, 4.0, 16.0);
			const auto reference = testscenes::pixelScene(blobs, kWidth, kHeight, 0.0, 0.0);
			const auto moved = testscenes::pixelScene(blobs, kWidth, kHeight, 4.0, 1.0);
			std::vector<unsigned char> valid((size_t)kWidth * kHeight, 1);
			for (int row = 2 * kHeight / 3; row < kHeight; row++) {
				for (int col = 0; col < kWidth; col++) {
					valid[(size_t)row * kWidth + col] = 0;
				}
			}
			const auto result = registration::match(
				registration::prepare(reference, valid, kWidth, kHeight, 1), registration::prepare(moved, valid, kWidth, kHeight, 1), 30.0);
			Assert::IsTrue(result.found);
			Assert::AreEqual(4.0, result.shiftCol, 0.2);
			Assert::AreEqual(1.0, result.shiftRow, 0.2);
		}

		TEST_METHOD(Registration_respects_the_search_range) {
			// a shift beyond maxShift cannot be found; the best match inside the range is reported with a poor correlation
			const auto blobs = testscenes::makeBlobs(kWidth, kHeight, 11, 70, 4.0, 16.0);
			const auto a = registration::prepare(testscenes::pixelScene(blobs, kWidth, kHeight, 0.0, 0.0), kWidth, kHeight, 1);
			const auto b = registration::prepare(testscenes::pixelScene(blobs, kWidth, kHeight, 25.0, 0.0), kWidth, kHeight, 1);
			const auto result = registration::match(a, b, 10.0);
			Assert::IsTrue(!result.found || std::hypot(result.shiftCol, result.shiftRow) <= 10.5);
		}

		TEST_METHOD(Registration_refuses_incompatible_input) {
			const auto blobs = testscenes::makeBlobs(kWidth, kHeight, 11, 70, 4.0, 16.0);
			const auto frame = testscenes::pixelScene(blobs, kWidth, kHeight, 0.0, 0.0);
			const auto a = registration::prepare(frame, kWidth, kHeight, 1);
			const auto b = registration::prepare(frame, kWidth, kHeight, 2);
			Assert::IsFalse(registration::match(a, b, 20.0).found);					// different downsampling
			Assert::IsFalse(registration::match(registration::PreparedImage{}, a, 20.0).found);		// empty image
			Assert::IsFalse(registration::prepare(frame, kWidth + 1, kHeight, 1).isValid());		// wrong size
		}

		TEST_METHOD(Registration_automatic_factor) {
			Assert::AreEqual(1, registration::automaticFactor(640, 480));
			Assert::AreEqual(2, registration::automaticFactor(1280, 1024));
			Assert::AreEqual(4, registration::automaticFactor(2448, 2048));
			Assert::AreEqual(1, registration::automaticFactor(100, 100, 0));
		}

		TEST_METHOD(Fft_inverse_restores_the_input) {
			const auto rows = 16;
			const auto cols = 32;
			std::vector<std::complex<float>> data((size_t)rows * cols);
			for (size_t i = 0; i < data.size(); i++) {
				data[i] = std::complex<float>((float)std::sin(0.37 * i), (float)std::cos(0.11 * i));
			}
			const auto original = data;
			registration::detail::fft2d(data, rows, cols, false);
			registration::detail::fft2d(data, rows, cols, true);
			for (size_t i = 0; i < data.size(); i++) {
				Assert::AreEqual((double)original[i].real(), (double)data[i].real(), 1e-4);
				Assert::AreEqual((double)original[i].imag(), (double)data[i].imag(), 1e-4);
			}
		}

		TEST_METHOD(Fft_of_a_delta_is_flat) {
			std::vector<std::complex<float>> data(64, std::complex<float>(0.0f, 0.0f));
			data[0] = std::complex<float>(1.0f, 0.0f);
			registration::detail::fft2d(data, 8, 8, false);
			for (const auto& value : data) {
				Assert::AreEqual(1.0, (double)value.real(), 1e-5);
				Assert::AreEqual(0.0, (double)value.imag(), 1e-5);
			}
		}

		TEST_METHOD(GaussianFilter_keeps_a_constant_and_spreads_a_spike) {
			const auto width = 40;
			const auto height = 30;
			std::vector<float> constant((size_t)width * height, 5.0f);
			registration::detail::gaussianFilter(constant, width, height, 3.0);
			for (const auto value : constant) {
				Assert::AreEqual(5.0, (double)value, 1e-4);
			}
			std::vector<float> spike((size_t)width * height, 0.0f);
			spike[(size_t)15 * width + 20] = 1.0f;
			registration::detail::gaussianFilter(spike, width, height, 2.0);
			auto sum = 0.0;
			for (const auto value : spike) {
				sum += value;
			}
			Assert::AreEqual(1.0, sum, 1e-4);		// the filter conserves the total
			Assert::IsTrue(spike[(size_t)15 * width + 20] < 0.1f && spike[(size_t)15 * width + 20] > spike[(size_t)15 * width + 24]);
		}

		TEST_METHOD(NextPowerOfTwo) {
			Assert::AreEqual(1, registration::detail::nextPowerOfTwo(1));
			Assert::AreEqual(8, registration::detail::nextPowerOfTwo(5));
			Assert::AreEqual(8, registration::detail::nextPowerOfTwo(8));
			Assert::AreEqual(1024, registration::detail::nextPowerOfTwo(1000));
		}
	};

}

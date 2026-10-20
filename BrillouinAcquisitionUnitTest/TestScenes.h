#ifndef TESTSCENES_H
#define TESTSCENES_H

#include <cmath>
#include <random>
#include <vector>

#include "../BrillouinAcquisition/src/lib/math/drift.h"

/*
 * Synthetic brightfield "tissue" for the registration and drift tests: a sum of Gaussian blobs of random size and sign on an uneven
 * illumination ramp, evaluated analytically at any position, so a frame can be generated for any stage position, any sample drift and
 * any sub-pixel shift without interpolation error. Nothing here reads a file.
 */
namespace testscenes {

	struct Blob {
		double x;
		double y;
		double sigma;
		double amplitude;
	};

	// blobs spread over [-margin, extent + margin]^2 (arbitrary units: pixels for the pixel scenes, um for the stage scenes)
	inline std::vector<Blob> makeBlobs(double width, double height, unsigned seed, int count = 70, double sigmaMin = 6.0, double sigmaMax = 30.0) {
		std::mt19937 generator(seed);
		std::uniform_real_distribution<double> unit(0.0, 1.0);
		std::vector<Blob> blobs;
		for (int i = 0; i < count; i++) {
			Blob blob;
			blob.x = -0.3 * width + 1.6 * width * unit(generator);
			blob.y = -0.3 * height + 1.6 * height * unit(generator);
			blob.sigma = sigmaMin + (sigmaMax - sigmaMin) * unit(generator);
			blob.amplitude = (unit(generator) < 0.5 ? -1.0 : 1.0) * (20.0 + 80.0 * unit(generator));
			blobs.push_back(blob);
		}
		return blobs;
	}

	inline double evaluate(const std::vector<Blob>& blobs, double x, double y) {
		// uneven illumination: a ramp across the field
		auto value = 128.0 + 0.04 * x + 0.02 * y;
		for (const auto& blob : blobs) {
			const auto dx = x - blob.x;
			const auto dy = y - blob.y;
			value += blob.amplitude * std::exp(-0.5 * (dx * dx + dy * dy) / (blob.sigma * blob.sigma));
		}
		return value;
	}

	// A frame in which the scene content is shifted by (shiftCol, shiftRow) pixels: pixel (col, row) shows the scene at (col - shiftCol, row - shiftRow).
	inline std::vector<float> pixelScene(
		const std::vector<Blob>& blobs, int width, int height, double shiftCol, double shiftRow, double noise = 0.0, unsigned noiseSeed = 1
	) {
		std::vector<float> frame((size_t)width * (size_t)height);
		std::mt19937 generator(noiseSeed);
		std::normal_distribution<double> gauss(0.0, 1.0);
		for (int row = 0; row < height; row++) {
			for (int col = 0; col < width; col++) {
				auto value = evaluate(blobs, col - shiftCol, row - shiftRow);
				if (noise > 0.0) {
					value += noise * gauss(generator);
				}
				frame[(size_t)row * width + col] = (float)value;
			}
		}
		return frame;
	}

	// A camera frame of a scene defined in stage micrometers, taken with the stage at positionUm, after the sample drifted by driftUm
	// (the scene content at stage position s sits at s + driftUm). The frame centre shows the stage position (as in the real software:
	// bmlab's get_point_to_pixel_matrix()), and a stage offset (x, y) moves the content by map * (x, y) pixels.
	inline std::vector<float> stageScene(
		const std::vector<Blob>& blobsUm, int width, int height, const drift::StageMap& map, POINT2 positionUm, POINT2 driftUm,
		double noise = 0.0, unsigned noiseSeed = 1
	) {
		std::vector<float> frame((size_t)width * (size_t)height);
		std::mt19937 generator(noiseSeed);
		std::normal_distribution<double> gauss(0.0, 1.0);
		const auto determinant = map.determinant();
		const auto centreCol = 0.5 * (width - 1);
		const auto centreRow = 0.5 * (height - 1);
		for (int row = 0; row < height; row++) {
			for (int col = 0; col < width; col++) {
				// stage offset of this pixel from the frame centre: inverse of the map
				const auto dCol = col - centreCol;
				const auto dRow = row - centreRow;
				const auto ox = (map.rowY * dCol - map.colY * dRow) / determinant;
				const auto oy = (-map.rowX * dCol + map.colX * dRow) / determinant;
				// the stage position this pixel looks at, and the scene position that is now there (the content moved by drift)
				const auto sx = positionUm.x + ox - driftUm.x;
				const auto sy = positionUm.y + oy - driftUm.y;
				auto value = evaluate(blobsUm, sx, sy);
				if (noise > 0.0) {
					value += noise * gauss(generator);
				}
				frame[(size_t)row * width + col] = (float)value;
			}
		}
		return frame;
	}

}

#endif // TESTSCENES_H

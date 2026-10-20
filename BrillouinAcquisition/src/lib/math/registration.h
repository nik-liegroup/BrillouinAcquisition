#ifndef REGISTRATION_H
#define REGISTRATION_H

#include <complex>
#include <vector>

/*
 * Translation-only registration of two brightfield frames by masked, normalised cross-correlation (no external library: plain C++).
 *
 * This is the same algorithm the offline drift correction (brillouin-drift-correction, registration.py) uses on the exported
 * overviews, so the drift measured during an acquisition and the drift measured afterwards agree: each frame is downsampled by
 * block means, a smooth background (a Gaussian-weighted mean over the valid pixels) is subtracted and the result smoothed slightly
 * (a band-pass: it removes uneven illumination and pixel noise); the normalised cross-correlation of two such frames is computed
 * for every shift within a given range with FFTs (the sums over the overlapping valid area are correlations too), and the maximum
 * is refined to sub-pixel precision with a parabola through its neighbours.
 *
 * A frame is prepared once (prepare()) and can then be matched against any number of other frames (match()): during an
 * acquisition the prepared form of the last accepted overview is all that has to be kept.
 */
namespace registration {

	// A frame as it enters the correlation: downsampled by `factor`, background-subtracted and smoothed, 0 outside the mask.
	struct PreparedImage {
		int width{ 0 };					// [pix]	of the downsampled image
		int height{ 0 };				// [pix]
		int factor{ 1 };				// [1]		raw pixels per downsampled pixel (along each axis)
		int rawWidth{ 0 };				// [pix]	of the frame the image was made from
		int rawHeight{ 0 };				// [pix]
		std::vector<float> data;		// band-passed values, row-major, 0 where mask is 0
		std::vector<float> mask;		// 1 where the pixel has valid data, 0 elsewhere

		bool isValid() const {
			return width > 0 && height > 0 && data.size() == (size_t)width * (size_t)height && mask.size() == data.size();
		}
	};

	// Outcome of match(). found is false if the frames cannot be compared at all (no overlap, invalid input); a found match can still
	// be a bad one - ncc says how good: the caller decides with its own threshold.
	struct Match {
		bool found{ false };
		// [raw pix] by how many pixels (columns, rows) the content of the reference frame appears shifted in the moved frame
		double shiftCol{ 0.0 };
		double shiftRow{ 0.0 };
		double ncc{ 0.0 };				// [-1..1]	normalised cross-correlation at the best shift
		double overlap{ 0.0 };			// [0..1]	valid overlap at the best shift, relative to the smaller of the two valid areas
	};

	// Downsampling factor that brings the longer side of a frame to at most maxSide pixels (at least 1): keeps the cost of the
	// correlation bounded for large sensors without changing the behaviour for frames that are already small.
	int automaticFactor(int width, int height, int maxSide = 640);

	// Pixels of the downsampled image within this distance of the frame edge are left out of the correlation (the band-pass is distorted
	// there, and that distortion does not move with the scene: it would pull the shift towards zero). Twice the background sigma.
	constexpr int kDefaultEdgeMargin = 30;

	// Prepares a raw frame (row-major, width x height) in which every pixel is valid.
	PreparedImage prepare(const std::vector<float>& frame, int width, int height, int factor, int edgeMargin = kDefaultEdgeMargin);

	// The same with a validity mask (non-zero = valid) of the same size, for images that have areas without data.
	PreparedImage prepare(const std::vector<float>& frame, const std::vector<unsigned char>& valid, int width, int height, int factor, int edgeMargin = kDefaultEdgeMargin);

	// Registers `moved` against `reference`: the shift (of the reference's content in the moved frame, in pixels of the raw frames)
	// that maximises the normalised cross-correlation, searched within maxShiftRawPix of zero. Shifts whose overlap is smaller than
	// minOverlap (relative to the smaller valid area) are not considered. Both images must have the same factor.
	Match match(const PreparedImage& reference, const PreparedImage& moved, double maxShiftRawPix, double minOverlap = 0.4);

	// Pieces of the above, exposed for testing.
	namespace detail {
		// 2D FFT of a rows x cols array (both powers of two), in place; inverse includes the 1/(rows*cols) scaling.
		void fft2d(std::vector<std::complex<float>>& data, int rows, int cols, bool inverse);
		// Gaussian filter, separable, scipy's convention (truncated at 4 sigma, 'reflect' boundary), in place on a row-major array.
		void gaussianFilter(std::vector<float>& data, int width, int height, double sigma);
		int nextPowerOfTwo(int n);
	}

}

#endif // REGISTRATION_H

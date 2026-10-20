#include "stdafx.h"
#include "registration.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace registration {

	namespace detail {

		int nextPowerOfTwo(int n) {
			auto result = 1;
			while (result < n) {
				result <<= 1;
			}
			return result;
		}

		namespace {

			// 1D radix-2 FFT of `count` values (power of two) in place; inverse includes no scaling.
			void fft1d(std::complex<float>* values, int count, bool inverse) {
				// bit reversal
				for (int i = 1, j = 0; i < count; i++) {
					auto bit = count >> 1;
					for (; j & bit; bit >>= 1) {
						j ^= bit;
					}
					j ^= bit;
					if (i < j) {
						std::swap(values[i], values[j]);
					}
				}
				for (int length = 2; length <= count; length <<= 1) {
					const auto angle = 2.0 * 3.14159265358979323846 / length * (inverse ? 1.0 : -1.0);
					const auto half = length / 2;
					// twiddle factors of this stage, computed in double and rounded once
					std::vector<std::complex<float>> twiddle((size_t)half);
					for (int k = 0; k < half; k++) {
						twiddle[(size_t)k] = std::complex<float>((float)std::cos(angle * k), (float)std::sin(angle * k));
					}
					for (int start = 0; start < count; start += length) {
						for (int k = 0; k < half; k++) {
							const auto even = values[start + k];
							const auto odd = values[start + k + half] * twiddle[(size_t)k];
							values[start + k] = even + odd;
							values[start + k + half] = even - odd;
						}
					}
				}
			}

			// Reflects an index into [0, count) like scipy's 'reflect' boundary mode (d c b a | a b c d | d c b a).
			int reflectIndex(int index, int count) {
				if (count == 1) {
					return 0;
				}
				const auto period = 2 * count;
				index %= period;
				if (index < 0) {
					index += period;
				}
				return index < count ? index : period - index - 1;
			}

			// Applies a symmetric kernel (radius+1 weights, centre first) along one line of `count` values spaced `stride` apart.
			void filterLine(float* line, int count, int stride, const std::vector<double>& kernel, std::vector<double>& padded) {
				const auto radius = (int)kernel.size() - 1;
				for (int i = -radius; i < count + radius; i++) {
					padded[(size_t)(i + radius)] = line[(size_t)reflectIndex(i, count) * stride];
				}
				for (int i = 0; i < count; i++) {
					auto sum = kernel[0] * padded[(size_t)(i + radius)];
					for (int k = 1; k <= radius; k++) {
						sum += kernel[(size_t)k] * (padded[(size_t)(i + radius - k)] + padded[(size_t)(i + radius + k)]);
					}
					line[(size_t)i * stride] = (float)sum;
				}
			}

		}

		void fft2d(std::vector<std::complex<float>>& data, int rows, int cols, bool inverse) {
			for (int row = 0; row < rows; row++) {
				fft1d(&data[(size_t)row * cols], cols, inverse);
			}
			std::vector<std::complex<float>> column((size_t)rows);
			for (int col = 0; col < cols; col++) {
				for (int row = 0; row < rows; row++) {
					column[(size_t)row] = data[(size_t)row * cols + col];
				}
				fft1d(column.data(), rows, inverse);
				for (int row = 0; row < rows; row++) {
					data[(size_t)row * cols + col] = column[(size_t)row];
				}
			}
			if (inverse) {
				const auto scale = 1.0f / ((float)rows * (float)cols);
				for (auto& value : data) {
					value *= scale;
				}
			}
		}

		void gaussianFilter(std::vector<float>& data, int width, int height, double sigma) {
			if (sigma <= 0.0 || width <= 0 || height <= 0) {
				return;
			}
			const auto radius = (int)(4.0 * sigma + 0.5);
			std::vector<double> kernel((size_t)radius + 1);
			auto total = 0.0;
			for (int k = 0; k <= radius; k++) {
				kernel[(size_t)k] = std::exp(-0.5 * (double)k * (double)k / (sigma * sigma));
				total += k == 0 ? kernel[0] : 2.0 * kernel[(size_t)k];
			}
			for (auto& weight : kernel) {
				weight /= total;
			}
			std::vector<double> padded((size_t)std::max(width, height) + 2 * (size_t)radius);
			for (int row = 0; row < height; row++) {
				filterLine(&data[(size_t)row * width], width, 1, kernel, padded);
			}
			for (int col = 0; col < width; col++) {
				filterLine(&data[(size_t)col], height, width, kernel, padded);
			}
		}

	}

	int automaticFactor(int width, int height, int maxSide) {
		const auto longest = std::max(width, height);
		if (maxSide <= 0 || longest <= maxSide) {
			return 1;
		}
		return (longest + maxSide - 1) / maxSide;
	}

	PreparedImage prepare(const std::vector<float>& frame, int width, int height, int factor, int edgeMargin) {
		return prepare(frame, std::vector<unsigned char>(frame.size(), 1), width, height, factor, edgeMargin);
	}

	PreparedImage prepare(const std::vector<float>& frame, const std::vector<unsigned char>& valid, int width, int height, int factor, int edgeMargin) {
		PreparedImage result;
		if (width <= 0 || height <= 0 || frame.size() != (size_t)width * (size_t)height || valid.size() != frame.size()) {
			return result;
		}
		factor = std::max(1, factor);
		const auto smallWidth = (width + factor - 1) / factor;
		const auto smallHeight = (height + factor - 1) / factor;
		const auto count = (size_t)smallWidth * (size_t)smallHeight;

		// Block means, a partial block at the edge counting its missing pixels as 0 (the convention of skimage's
		// downscale_local_mean, which the offline correction uses); such blocks have a mask mean below 1 and drop out below.
		std::vector<float> image(count, 0.0f);
		std::vector<float> maskFraction(count, 0.0f);
		const auto blockArea = (double)factor * (double)factor;
		for (int row = 0; row < smallHeight; row++) {
			for (int col = 0; col < smallWidth; col++) {
				auto sum = 0.0;
				auto sumValid = 0.0;
				for (int dy = 0; dy < factor; dy++) {
					const auto y = row * factor + dy;
					if (y >= height) {
						break;
					}
					for (int dx = 0; dx < factor; dx++) {
						const auto x = col * factor + dx;
						if (x >= width) {
							break;
						}
						const auto index = (size_t)y * width + x;
						sum += frame[index];
						sumValid += valid[index] ? 1.0 : 0.0;
					}
				}
				image[(size_t)row * smallWidth + col] = (float)(sum / blockArea);
				maskFraction[(size_t)row * smallWidth + col] = (float)(sumValid / blockArea);
			}
		}
		std::vector<float> mask(count);
		for (size_t i = 0; i < count; i++) {
			mask[i] = maskFraction[i] > 0.99f ? 1.0f : 0.0f;
		}

		// Band-pass: remove a smooth, mask-aware background, then smooth slightly.
		auto weight = mask;
		detail::gaussianFilter(weight, smallWidth, smallHeight, 15.0);
		std::vector<float> background(count);
		for (size_t i = 0; i < count; i++) {
			background[i] = mask[i] > 0.0f ? image[i] : 0.0f;
		}
		detail::gaussianFilter(background, smallWidth, smallHeight, 15.0);
		std::vector<float> filtered(count);
		for (size_t i = 0; i < count; i++) {
			const auto bg = background[i] / std::max(weight[i], 1e-3f);
			filtered[i] = mask[i] > 0.0f ? image[i] - bg : 0.0f;
		}
		detail::gaussianFilter(filtered, smallWidth, smallHeight, 1.0);
		// The band-pass is distorted near the frame edge (the background estimate there rests on reflected pixels), and that distortion
		// stays in place while the scene moves: left in, it correlates at zero shift and pulls the result towards it. Pixels within
		// edgeMargin of the frame edge therefore do not take part in the correlation.
		for (int row = 0; row < smallHeight; row++) {
			for (int col = 0; col < smallWidth; col++) {
				const auto edge = std::min(std::min(col, smallWidth - 1 - col), std::min(row, smallHeight - 1 - row));
				if (edge < edgeMargin) {
					mask[(size_t)row * smallWidth + col] = 0.0f;
				}
			}
		}
		for (size_t i = 0; i < count; i++) {
			if (mask[i] <= 0.0f) {
				filtered[i] = 0.0f;
			}
		}

		result.width = smallWidth;
		result.height = smallHeight;
		result.factor = factor;
		result.rawWidth = width;
		result.rawHeight = height;
		result.data = std::move(filtered);
		result.mask = std::move(mask);
		return result;
	}

	Match match(const PreparedImage& reference, const PreparedImage& moved, double maxShiftRawPix, double minOverlap) {
		Match result;
		if (!reference.isValid() || !moved.isValid() || reference.factor != moved.factor || maxShiftRawPix < 0.0) {
			return result;
		}
		const auto radius = maxShiftRawPix / reference.factor;		// [downsampled pix]
		const auto window = (int)std::ceil(radius);
		// A circular correlation of zero-padded images is a linear one for every lag u with |u| <= window as long as the array is at
		// least (image size + window) long along each axis: nothing wraps around into the lags that are used.
		const auto rows = detail::nextPowerOfTwo(std::max(reference.height, moved.height) + window);
		const auto cols = detail::nextPowerOfTwo(std::max(reference.width, moved.width) + window);
		const auto size = (size_t)rows * (size_t)cols;

		using Complex = std::complex<float>;
		auto transform = [&](const PreparedImage& image, int kind) {
			// kind 0: the values, 1: the squared values, 2: the mask
			std::vector<Complex> spectrum(size, Complex(0.0f, 0.0f));
			for (int row = 0; row < image.height; row++) {
				for (int col = 0; col < image.width; col++) {
					const auto index = (size_t)row * image.width + col;
					const auto value = kind == 2 ? image.mask[index] : (kind == 1 ? image.data[index] * image.data[index] : image.data[index]);
					spectrum[(size_t)row * cols + col] = Complex(value, 0.0f);
				}
			}
			detail::fft2d(spectrum, rows, cols, false);
			return spectrum;
		};
		const auto fa = transform(reference, 0);
		const auto fa2 = transform(reference, 1);
		const auto fma = transform(reference, 2);
		const auto fb = transform(moved, 0);
		const auto fb2 = transform(moved, 1);
		const auto fmb = transform(moved, 2);

		const auto span = 2 * window + 1;
		// out[u] = sum_p x(p) y(p - u) for the lags u = (row, col) in [-window, window]^2, stored at (row + window, col + window)
		auto correlate = [&](const std::vector<Complex>& x, const std::vector<Complex>& y) {
			std::vector<Complex> product(size);
			for (size_t i = 0; i < size; i++) {
				product[i] = x[i] * std::conj(y[i]);
			}
			detail::fft2d(product, rows, cols, true);
			std::vector<double> out((size_t)span * (size_t)span);
			for (int ur = -window; ur <= window; ur++) {
				const auto r = (size_t)((ur + rows) % rows);
				for (int uc = -window; uc <= window; uc++) {
					const auto c = (size_t)((uc + cols) % cols);
					out[(size_t)(ur + window) * span + (uc + window)] = product[r * cols + c].real();
				}
			}
			return out;
		};
		const auto n = correlate(fma, fmb);
		const auto sa = correlate(fa, fmb);
		const auto sb = correlate(fma, fb);
		const auto ab = correlate(fa, fb);
		const auto a2 = correlate(fa2, fmb);
		const auto b2 = correlate(fma, fb2);

		auto sumMaskA = 0.0;
		for (const auto value : reference.mask) {
			sumMaskA += value;
		}
		auto sumMaskB = 0.0;
		for (const auto value : moved.mask) {
			sumMaskB += value;
		}
		const auto smallerArea = std::min(sumMaskA, sumMaskB);
		if (smallerArea <= 0.0) {
			return result;
		}

		const auto negativeInfinity = -std::numeric_limits<double>::infinity();
		std::vector<double> score((size_t)span * (size_t)span, negativeInfinity);
		std::vector<double> coverage((size_t)span * (size_t)span, 0.0);
		auto best = negativeInfinity;
		auto bestRow = -1;
		auto bestCol = -1;
		for (int ur = -window; ur <= window; ur++) {
			for (int uc = -window; uc <= window; uc++) {
				if (std::hypot((double)ur, (double)uc) > radius) {
					continue;
				}
				const auto index = (size_t)(ur + window) * span + (uc + window);
				if (n[index] < minOverlap * smallerArea) {
					continue;
				}
				const auto count = std::max(n[index], 1.0);
				const auto covariance = ab[index] - sa[index] * sb[index] / count;
				const auto varianceA = std::max(a2[index] - sa[index] * sa[index] / count, 0.0);
				const auto varianceB = std::max(b2[index] - sb[index] * sb[index] / count, 0.0);
				const auto denominator = std::sqrt(varianceA * varianceB);
				const auto ncc = denominator > 1e-6 * count ? covariance / std::max(denominator, 1e-12) : 0.0;
				score[index] = ncc;
				coverage[index] = n[index] / smallerArea;
				if (ncc > best) {
					best = ncc;
					bestRow = ur + window;
					bestCol = uc + window;
				}
			}
		}
		if (bestRow < 0) {
			return result;
		}

		// Sub-pixel position of the peak: a parabola through the maximum and its two neighbours along each axis (0 if a neighbour
		// is outside the searched area or was excluded).
		auto parabola = [](double left, double centre, double right) {
			const auto denominator = left - 2.0 * centre + right;
			if (!std::isfinite(denominator) || std::abs(denominator) < 1e-12) {
				return 0.0;
			}
			return 0.5 * (left - right) / denominator;
		};
		auto at = [&](int r, int c) {
			if (r < 0 || c < 0 || r >= span || c >= span) {
				return negativeInfinity;
			}
			return score[(size_t)r * span + c];
		};
		const auto lagCol = (double)(bestCol - window) + parabola(at(bestRow, bestCol - 1), best, at(bestRow, bestCol + 1));
		const auto lagRow = (double)(bestRow - window) + parabola(at(bestRow - 1, bestCol), best, at(bestRow + 1, bestCol));

		result.found = true;
		result.shiftCol = -lagCol * reference.factor;
		result.shiftRow = -lagRow * reference.factor;
		result.ncc = best;
		result.overlap = coverage[(size_t)bestRow * span + bestCol];
		return result;
	}

}

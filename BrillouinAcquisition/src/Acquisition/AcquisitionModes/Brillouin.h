#ifndef BRILLOUIN_H
#define BRILLOUIN_H

#include "AcquisitionMode.h"
#include "../../Devices/Cameras/Camera.h"
#include "../../helper/thread.h"
#include "src/lib/buffer_circular.h"
#include "src/lib/math/drift.h"
#include "src/Acquisition/Planning/OverviewSchedule.h"
#include "ResumeRecord.h"
#include "ResumeFile.h"
#include <functional>
#include <map>
#include <limits>
#include <optional>
#include <set>
#include <utility>


// Loop level of each axis: 0 = innermost (fastest), 2 = outermost (slowest).
// Automatic: z is always the outermost loop (plane by plane), and x/y are ordered by step
// count (more steps = more inner) - see Brillouin::determineScanOrder(). Manual: any
// permutation, but only with z outermost do the per-z overview brightfield images follow
// finished planes; otherwise they're spread evenly across the scan (see
// Brillouin::runMeasurementPhase()).
struct SCAN_ORDER {
	bool automatical{ true };
	int x{ 0 };	// first scan in x-direction
	int y{ 1 };	// then in y-direction
	int z{ 2 };	// scan in z-direction last
};

// Writes the live previews of the overviews on a thread of its own (Brillouin_Overview.cpp)
class LivePreviewWorker;

// What an overview with one objective looks like (the settings of the objective that measures are the BF overview settings of
// BRILLOUIN_SETTINGS below; this is for the others, see Brillouin_Overview.cpp). Each objective has its own, independent of the others.
struct OVERVIEW_OBJECTIVE_SETTINGS {
	int slot{ -1 };					// nosepiece slot of the objective
	bool active{ false };			// take overviews with this objective
	int exposureMs{ 4 };			// [ms]	brightfield exposure
	double gain{ 0.0 };				// camera gain
	bool fullGrid{ false };			// tile the whole grid (mosaic of camera-sized images, 20 % overlap) instead of one image at the grid centre
	bool fullStack{ false };		// a z stack instead of one image (the centre slice is used for drift correction)
	int stackSteps{ 5 };			// [1]	images of the stack
	double stackRangeUm{ 20.0 };	// [um]	z range of the stack, centred on the objective's focus
};

struct BRILLOUIN_SETTINGS {
	private:
		// ROI parameters
		double m_xMin{ 0 };		// [µm]	x minimum value
		double m_xMax{ 10 };	// [µm]	x maximum value
		int m_xSteps{ 3 };		// [1]	x steps
		double m_yMin{ 0 };		// [µm]	y minimum value
		double m_yMax{ 10 };	// [µm]	y maximum value
		int m_ySteps{ 3 };		// [1]	y steps
		double m_zMin{ 0 };		// [µm]	z minimum value
		double m_zMax{ 0 };		// [µm]	z maximum value
		int m_zSteps{ 1 };		// [1]	z steps

		// ROI limits
		std::vector<double> m_xyzLim{ -1000000, 1000000 };
		std::vector<int> m_stepsLim{ 1, 100000 };

		template <typename T>
		void checkLimits(T &value, std::vector<T> limits) {
			if (value < limits[0]) {
				value = limits[0];
			}
			if (value > limits[1]) {
				value = limits[1];
			}
		};

	public:
		BRILLOUIN_SETTINGS& operator=(const BRILLOUIN_SETTINGS& settings) {
			m_xMin = settings.xMin;
			m_xMax = settings.xMax;
			m_xSteps = settings.xSteps;
			m_yMin = settings.yMin;
			m_yMax = settings.yMax;
			m_ySteps = settings.ySteps;
			m_zMin = settings.zMin;
			m_zMax = settings.zMax;
			m_zSteps = settings.zSteps;
			sample = settings.sample;
			preCalibration = settings.preCalibration;
			postCalibration = settings.postCalibration;
			conCalibration = settings.conCalibration;
			conCalibrationInterval = settings.conCalibrationInterval;
			nrCalibrationImages = settings.nrCalibrationImages;
			calibrationExposureTime = settings.calibrationExposureTime;
			repetitions = settings.repetitions;
			useRoiMask = settings.useRoiMask;
			roiPolygonUm = settings.roiPolygonUm;
			useBackgroundRoiMask = settings.useBackgroundRoiMask;
			backgroundRoiPolygonUm = settings.backgroundRoiPolygonUm;
			useSurfaceFollow = settings.useSurfaceFollow;
			surfaceFollowHalfRangeUm = settings.surfaceFollowHalfRangeUm;
			surfaceMaxRewindUm = settings.surfaceMaxRewindUm;
			surfaceSearchBelowZeroUm = settings.surfaceSearchBelowZeroUm;
			surfaceVerificationSteps = settings.surfaceVerificationSteps;
			surfaceVerificationFrameAverage = settings.surfaceVerificationFrameAverage;
			surfaceVerificationToleranceFraction = settings.surfaceVerificationToleranceFraction;
			preScanXYBin = settings.preScanXYBin;
			additionalBoundaryPoints = settings.additionalBoundaryPoints;
			preScanZStepUm = settings.preScanZStepUm;
			preScanZTravelRangeUm = settings.preScanZTravelRangeUm;
			preScanXSteps = settings.preScanXSteps;
			preScanYSteps = settings.preScanYSteps;
			preScanZSteps = settings.preScanZSteps;
			preScanZMin = settings.preScanZMin;
			preScanZMax = settings.preScanZMax;
			surfaceMetricThreshold = settings.surfaceMetricThreshold;
			surfaceDropFraction = settings.surfaceDropFraction;
			surfaceScanDirection = settings.surfaceScanDirection;
			surfaceProxyRoiLeft = settings.surfaceProxyRoiLeft;
			surfaceProxyRoiTop = settings.surfaceProxyRoiTop;
			surfaceProxyRoiWidth = settings.surfaceProxyRoiWidth;
			surfaceProxyRoiHeight = settings.surfaceProxyRoiHeight;
			surfaceProxyRoi2Left = settings.surfaceProxyRoi2Left;
			surfaceProxyRoi2Top = settings.surfaceProxyRoi2Top;
			surfaceProxyRoi2Width = settings.surfaceProxyRoi2Width;
			surfaceProxyRoi2Height = settings.surfaceProxyRoi2Height;
			surfaceProxyRoiFrameWidth = settings.surfaceProxyRoiFrameWidth;
			surfaceProxyRoiFrameHeight = settings.surfaceProxyRoiFrameHeight;
			surfaceProxyRoiFrameOriginLeft = settings.surfaceProxyRoiFrameOriginLeft;
			surfaceProxyRoiFrameOriginBottom = settings.surfaceProxyRoiFrameOriginBottom;
			surfaceProxyRoiFrameWidthPhysical = settings.surfaceProxyRoiFrameWidthPhysical;
			surfaceProxyRoiFrameHeightPhysical = settings.surfaceProxyRoiFrameHeightPhysical;
			surfaceProxyRoi2FrameWidth = settings.surfaceProxyRoi2FrameWidth;
			surfaceProxyRoi2FrameHeight = settings.surfaceProxyRoi2FrameHeight;
			surfaceProxyRoi2FrameOriginLeft = settings.surfaceProxyRoi2FrameOriginLeft;
			surfaceProxyRoi2FrameOriginBottom = settings.surfaceProxyRoi2FrameOriginBottom;
			surfaceProxyRoi2FrameWidthPhysical = settings.surfaceProxyRoi2FrameWidthPhysical;
			surfaceProxyRoi2FrameHeightPhysical = settings.surfaceProxyRoi2FrameHeightPhysical;
			mediumReferenceValue = settings.mediumReferenceValue;
			mediumReferenceFrameCount = settings.mediumReferenceFrameCount;
			gridCoordinatesAbsolute = settings.gridCoordinatesAbsolute;
			absoluteGridOriginUm = settings.absoluteGridOriginUm;
			saveOverviewBrightfieldPerZ = settings.saveOverviewBrightfieldPerZ;
			overviewBrightfieldExposureMs = settings.overviewBrightfieldExposureMs;
			overviewBrightfieldGain = settings.overviewBrightfieldGain;
			overviewBrightfieldFullGrid = settings.overviewBrightfieldFullGrid;
			overviewBrightfieldFullStackSingle = settings.overviewBrightfieldFullStackSingle;
			overviewBrightfieldFullStackMosaic = settings.overviewBrightfieldFullStackMosaic;
			overviewTrigger = settings.overviewTrigger;
			overviewBeforeMeasurement = settings.overviewBeforeMeasurement;
			overviewAfterMeasurement = settings.overviewAfterMeasurement;
			overviewObjectives = settings.overviewObjectives;
			overviewObjectiveRetractUm = settings.overviewObjectiveRetractUm;
			driftCorrectionEnabled = settings.driftCorrectionEnabled;
			driftObjectiveSlot = settings.driftObjectiveSlot;
			driftMinNcc = settings.driftMinNcc;
			driftMaxStepUm = settings.driftMaxStepUm;
			driftMaxSpeedUmPerMin = settings.driftMaxSpeedUmPerMin;
			driftExtrapolate = settings.driftExtrapolate;
			surfaceRescan = settings.surfaceRescan;
			livePreviews = settings.livePreviews;
			capturePerPointBrightfield = settings.capturePerPointBrightfield;
			perPointBrightfieldEveryN = settings.perPointBrightfieldEveryN;
			perPointBrightfieldDuringAcquisition = settings.perPointBrightfieldDuringAcquisition;
			useGridHysteresisCompensation = settings.useGridHysteresisCompensation;
			useDoseProtection = settings.useDoseProtection;
			extendFramesOnLowSignal = settings.extendFramesOnLowSignal;
			proxyRoiMetricThreshold = settings.proxyRoiMetricThreshold;
			maxAdditionalFrames = settings.maxAdditionalFrames;
			camera = settings.camera;
			return *this;
		}
		// calibration parameters
		std::string sample{ "Methanol & Water" };
		bool preCalibration{ true };				// do pre calibration
		bool postCalibration{ true };				// do post calibration
		bool conCalibration{ true };				// do continuous calibration
		double conCalibrationInterval{ 10 };		// interval of continuous calibrations
		int nrCalibrationImages{ 10 };				// number of calibration images
		double calibrationExposureTime{ 1 };		// exposure time for calibration images

		// repetition parameters
		REPETITIONS repetitions;

		// Advanced scan planning (future extensions, disabled by default)
		bool useRoiMask{ false };
		std::vector<POINT2> roiPolygonUm;
		// A second, independent ROI polygon - drawn/edited/plotted the same way roiPolygonUm is,
		// but with its own points, own on-screen overlay and no bearing on the main ROI mask.
		// Marks an extra region (e.g. water beside the sample) sampled for reference points
		// alongside the main grid - deliberately NOT clipped to xMin/xMax/yMin/yMax like the main
		// ROI is, since the whole point is a region outside the main grid's own extent. Surface
		// pre-scan (runSurfacePreScan(), additionalBoundaryXYPoints()) never looks at this -
		// only the main roiPolygonUm/useRoiMask.
		bool useBackgroundRoiMask{ false };
		std::vector<POINT2> backgroundRoiPolygonUm;
		bool useSurfaceFollow{ false };
		double surfaceFollowHalfRangeUm{ 10.0 };
		// Neighbor-seeded surface search: each coarse column starts its search from the
		// nearest already-found neighbor's z instead of a blind full-range sweep.
		double surfaceMaxRewindUm{ 15.0 };			// max distance allowed stepping back towards water before giving up
		double surfaceSearchBelowZeroUm{ 0.0 };		// how far below the grid z origin (z = 0) the surface search may go; 0 = never below 0
		int surfaceVerificationSteps{ 3 };			// K: extra steps checked after a threshold crossing before accepting it
		int surfaceVerificationFrameAverage{ 1 };	// M: frames averaged per point during verification (1 = off)
		double surfaceVerificationToleranceFraction{ 0.05 }; // allowed relative rebound in the verification window
		int preScanXYBin{ 3 };
		// Extra anchor points placed directly on the active outline (the ROI clipped to the
		// grid rectangle, or the grid rectangle itself if no ROI is set) - on top of, not
		// instead of, the uniform coarse grid above. 0 = off (default). Chosen by greedy
		// farthest-point selection against the uniform grid's own kept anchors (see
		// Brillouin::additionalBoundaryXYPoints()), so each one lands where coverage is
		// thinnest along the outline rather than at a fixed stride.
		int additionalBoundaryPoints{ 0 };
		double preScanZStepUm{ 3.0 };
		double preScanZTravelRangeUm{ 30.0 };
		int preScanXSteps{ 8 };
		int preScanYSteps{ 8 };
		int preScanZSteps{ 20 };
		double preScanZMin{ -30.0 };
		double preScanZMax{ 30.0 };
		double surfaceMetricThreshold{ 0.1 };
		double surfaceDropFraction{ 0.6 };
		int surfaceScanDirection{ 1 }; // +1 increasing z, -1 decreasing z
		int surfaceProxyRoiLeft{ 0 };
		int surfaceProxyRoiTop{ 0 };
		int surfaceProxyRoiWidth{ 0 };
		int surfaceProxyRoiHeight{ 0 };
		int surfaceProxyRoi2Left{ 0 };
		int surfaceProxyRoi2Top{ 0 };
		int surfaceProxyRoi2Width{ 0 };
		int surfaceProxyRoi2Height{ 0 };
		// Frame (crop + binning) the two ROIs above were drawn against: the binned frame
		// size, the absolute sensor position of that frame's origin (camera.roi.left/bottom,
		// in physical/pre-binning pixels), and that frame's physical size. If the camera's
		// actual ROI at measurement time (surface pre-scan, or a live preview reconfigured
		// since) differs from this - e.g. the view was zoomed/cropped differently, or
		// binning changed - the stored rectangle is remapped through its absolute sensor
		// position onto the current frame before use. Remapping only via a frame-size ratio
		// (ignoring the origin) is wrong whenever the two frames don't share the same sensor
		// origin - it silently lands the ROI on the wrong physical location instead of
		// raising an error, which is what "surface never found despite a visible drop" or a
		// scattered calibration fit looks like. 0 means "never drawn yet", in which case no
		// remapping is attempted.
		int surfaceProxyRoiFrameWidth{ 0 };
		int surfaceProxyRoiFrameHeight{ 0 };
		long long surfaceProxyRoiFrameOriginLeft{ 0 };
		long long surfaceProxyRoiFrameOriginBottom{ 0 };
		long long surfaceProxyRoiFrameWidthPhysical{ 0 };
		long long surfaceProxyRoiFrameHeightPhysical{ 0 };
		int surfaceProxyRoi2FrameWidth{ 0 };
		int surfaceProxyRoi2FrameHeight{ 0 };
		long long surfaceProxyRoi2FrameOriginLeft{ 0 };
		long long surfaceProxyRoi2FrameOriginBottom{ 0 };
		long long surfaceProxyRoi2FrameWidthPhysical{ 0 };
		long long surfaceProxyRoi2FrameHeightPhysical{ 0 };
		// Medium reference is always measured before a surface scan - there is no other
		// threshold source, so this isn't user-optional.
		double mediumReferenceValue{ 0.0 };
		int mediumReferenceFrameCount{ 5 };
		bool gridCoordinatesAbsolute{ false };
		POINT3 absoluteGridOriginUm{ 0.0, 0.0, 0.0 };
		bool saveOverviewBrightfieldPerZ{ false };
		int overviewBrightfieldExposureMs{ 4 };
		double overviewBrightfieldGain{ 0.0 };
		// The overview image itself: instead of one image per z slice at the grid center
		// (false), tile enough camera-FOV-sized images (20% overlap) to cover the whole
		// grid extent (true). Works in both absolute and relative grid coordinate mode.
		bool overviewBrightfieldFullGrid{ false };
		// Independent full-z-stack toggles, one per coverage mode (overviewBrightfieldFullGrid
		// above chooses which one is actually in effect at any time - only ever one of the
		// two, never both at once, but each remembers its own on/off state so switching
		// coverage mode doesn't silently carry the other mode's choice over). If the active
		// one is false, one flat-z overview image per finished z slice (legacy behaviour: the
		// plane's own zMin..zMax offset from the grid origin, no surface-follow adjustment).
		// If true, one full z-stack per finished z slice instead - spanning zMin..zMax
		// (surface-follow off) or the global lowest-to-highest found surface, offset by
		// zMin/zMax (surface-follow on), always sampled at zSteps points.
		bool overviewBrightfieldFullStackSingle{ false };
		bool overviewBrightfieldFullStackMosaic{ false };
		// When overviews are taken during a measurement (Brillouin_Overview.cpp). PER_PLANE (default) is the behaviour of every version so
		// far: one overview after each finished z plane. The two flags add an overview before the first point (after the surface scan) and
		// after the last one (before the background spectra); they are additional to the trigger and never double an overview that is
		// already due at that moment. Everything is off or at its default unless it is changed, and then nothing differs from before.
		OVERVIEW_TRIGGER overviewTrigger{ OVERVIEW_TRIGGER::PER_PLANE };
		bool overviewBeforeMeasurement{ false };
		bool overviewAfterMeasurement{ false };
		// Overviews with objectives other than the one that measures: one entry per objective that was configured, each with its own
		// settings. (The measuring objective's overview is the one described by the settings above.)
		std::vector<OVERVIEW_OBJECTIVE_SETTINGS> overviewObjectives;
		// [um] relative z move (sign as entered, like the retract of the automated objective calibration) that takes the objective away
		// from the sample before the nosepiece turns. Must not be 0 when another objective takes overviews.
		double overviewObjectiveRetractUm{ 0.0 };
		// Drift correction: the overviews of the objective in driftObjectiveSlot (one of the objectives that take overviews) are
		// registered against the previous accepted overview, and the grid, the overviews and everything else that targets the sample
		// follow it by the measured drift (x and y only; the centre slice of a z stack is used). A step that is not believed (correlation
		// below driftMinNcc, larger than driftMaxStepUm, faster than driftMaxSpeedUmPerMin) leaves the positions where they are.
		bool driftCorrectionEnabled{ false };
		int driftObjectiveSlot{ -1 };
		double driftMinNcc{ 0.3 };
		double driftMaxStepUm{ 150.0 };
		double driftMaxSpeedUmPerMin{ 30.0 };
		// continue with the last measured drift speed between two overviews
		bool driftExtrapolate{ false };
		// Whether the surface scan is repeated during the measurement (surface follow must be on): with every overview, or after every
		// z plane. A repeated scan measures a new medium reference and threshold at the same start position and moves the remaining
		// points to the new surface.
		SURFACE_RESCAN surfaceRescan{ SURFACE_RESCAN::NEVER };
		// Overview images are also saved as PNG previews (the centre slice of every stack) in a folder next to the file, with a log of
		// the overview events (events.csv), so a running measurement can be looked at without opening the file
		bool livePreviews{ true };
		// Captures a brightfield image at every perPointBrightfieldEveryN-th *measured* grid
		// point during the main acquisition loop (runMeasurementPhase()) - independent of,
		// and additive with, saveOverviewBrightfieldPerZ above (which only ever captures at
		// the overview image's own xy point(s)/per-z cadence, not per actually-measured
		// point). Meant for post-hoc drift correction against each point's own image, not a
		// sample-wide overview. Off by default; reuses overviewBrightfieldExposureMs/Gain
		// above rather than a second copy of the same two settings. Always captured right
		// after that point's Brillouin spectrum finishes, or overlapped with it - see
		// perPointBrightfieldDuringAcquisition below - never before it, so the spectrum
		// itself is never at risk from a preset switch happening first.
		bool capturePerPointBrightfield{ false };
		// N: capture every Nth measured point (1 = every point), counted in acquisition
		// order (the position's own index in m_orderedPositions), not xy/z grid index.
		int perPointBrightfieldEveryN{ 1 };
		// false (default, "after"): captured with a full, clean preset switch to
		// Brightfield and back immediately after that point's Brillouin spectrum finishes -
		// safest, but adds the switch + exposure + switch-back cost to every captured point.
		// true ("during"): the brightfield camera's acquisition is started before that
		// point's Brillouin exposure begins and only retrieved after it finishes, so the
		// (normally much shorter) brightfield exposure/readout overlaps the Brillouin dwell
		// time instead of adding to it - genuinely zero added wall-clock time on a properly
		// synchronized setup, not just less than "after"'s cost. No preset switch either,
		// since the excitation/detection path is left exactly as the Brillouin preset set
		// it. The tradeoff: the brightfield illumination is then physically present during
		// the actual spectral exposure too, which can contaminate the Brillouin signal
		// depending on how optically isolated your two detection paths really are - verify
		// on your own setup (e.g. compare a spectrum captured with this on vs off) before
		// trusting it for real data, not assumed safe by this code.
		bool perPointBrightfieldDuringAcquisition{ false };
		// Whether stepping from one measurement grid point to the next approaches it from
		// a consistent direction first (ScanControl::setPositionCompensated()) to cancel
		// out stage hysteresis/backlash - accurate but costs an extra move + 100 ms settle
		// per grid point. Off skips straight to the target (ScanControl::setPosition()) for
		// faster stepping, at the cost of potential backlash error. Applies globally -
		// every compensated move in this class (grid-to-grid stepping, the surface
		// pre-scan's own z-stepping, the initial approach to the first point) goes through
		// approachGridPosition(), which is what actually reads this flag; there is no
		// separate "surface pre-scan always compensates" carve-out any more.
		bool useGridHysteresisCompensation{ true };
		// With this on, the RL Shutter (the laser's shutter) is closed by default for the whole
		// acquisition and only opened for a short, fixed margin immediately around each actual
		// camera exposure (see Brillouin::acquireAndorFrame(), the one place every m_andor frame
		// grab in this class goes through) and closed again right after - not just during stage
		// travel between points, but between the two frames of a single point too, during a
		// per-point brightfield capture, during a live calibration switch, and everywhere else
		// nothing is actually integrating.
		bool useDoseProtection{ false };

		// Adaptive frame extension: independent of, and unaffected by, surfaceScanningGroupBox/
		// useSurfaceFollow - applies to every acquisition mode's main measurement grid loop
		// (runMeasurementPhase()), not just surface-follow scans. Reuses the same spectral
		// proxy-ROI boxes surface-follow autofocus already uses, now also as a per-point
		// stopping criterion - but NOT via estimateFrameMetric() on a single frame the way
		// autofocus reads it: a single fresh frame's own signal level doesn't improve just
		// because earlier frames already exist, so checking each new frame in isolation would
		// almost never clear a threshold a low-signal point was already failing, and the loop
		// would just burn through maxAdditionalFrames every time without actually helping.
		// What determines fit quality is the *accumulated* photon count, so
		// runMeasurementPhase() instead keeps a running per-pixel SUM across every frame taken
		// so far at this point (initial camera.frameCount frames, plus however many extras have
		// been added) and checks THAT via estimateFrameMetricFromSum() - one extra frame at a
		// time, added into the same running sum, not a fresh per-frame decision each time.
		bool extendFramesOnLowSignal{ false };
		// A fixed, absolute target for the running SUM described above - NOT a single frame's
		// own max (that's what estimateFrameMetric() returns for autofocus), and NOT rescaled
		// by how many frames have gone into the sum. Calibrate it directly against "max counts
		// in the image that actually gets fit" for your normal camera.frameCount - e.g. if you
		// already know from experience that a summed max of 249 counts gives a good fit at 2
		// frame repeats, set it to 249, and the running sum is compared against that same 249
		// whether it took camera.frameCount frames alone or needed extras added - no scaling in
		// between. (249 here is the default because it's what a max-vs-fit-SNR calibration
		// against real 2-frame-repeat data landed on - see
		// ProxyMetricThresholdAnalysis/proxy_metric_vs_snr_summed.py - not a universal constant;
		// recalibrate if your frame-repeat count or sample differs.)
		double proxyRoiMetricThreshold{ 249.0 };
		// Frames beyond the normal camera.frameCount a point may take while its metric stays
		// below proxyRoiMetricThreshold. 0 disables extension even if
		// extendFramesOnLowSignal is on.
		int maxAdditionalFrames{ 0 };

		// ROI parameters
		const double& xMin{ m_xMin };
		void setXMin(double xMin) {
			checkLimits(xMin, m_xyzLim);
			m_xMin = xMin;
			if (m_xMax < m_xMin) {
				m_xMax = m_xMin;
			}
		};
		const double& xMax{ m_xMax };
		void setXMax(double xMax) {
			checkLimits(xMax, m_xyzLim);
			m_xMax = xMax;
			if (m_xMax < m_xMin) {
				m_xMin = m_xMax;
			}
		};
		const int& xSteps{ m_xSteps };
		void setXSteps(int xSteps) {
			checkLimits(xSteps, m_stepsLim);
			m_xSteps = xSteps;
		};
		const double& yMin = m_yMin;
		void setYMin(double yMin) {
			checkLimits(yMin, m_xyzLim);
			m_yMin = yMin;
			if (m_yMax < m_yMin) {
				m_yMax = m_yMin;
			}
		};
		const double& yMax = m_yMax;
		void setYMax(double yMax) {
			checkLimits(yMax, m_xyzLim);
			m_yMax = yMax;
			if (m_yMax < m_yMin) {
				m_yMin = m_yMax;
			}
		};
		const int& ySteps = m_ySteps;
		void setYSteps(int ySteps) {
			checkLimits(ySteps, m_stepsLim);
			m_ySteps = ySteps;
		};
		const double& zMin = m_zMin;
		void setZMin(double zMin) {
			checkLimits(zMin, m_xyzLim);
			m_zMin = zMin;
			if (m_zMax < m_zMin) {
				m_zMax = m_zMin;
			}
		};
		const double& zMax = m_zMax;
		void setZMax(double zMax) {
			checkLimits(zMax, m_xyzLim);
			m_zMax = zMax;
			if (m_zMax < m_zMin) {
				m_zMin = m_zMax;
			}
		};
		const int& zSteps = m_zSteps;
		void setZSteps(int zSteps) {
			checkLimits(zSteps, m_stepsLim);
			m_zSteps = zSteps;
		};

		CAMERA_SETTINGS camera;
};

// Describes the geometry of a frame a spectral proxy ROI is defined against or measured
// on: its binned size (width/height, the array/plot cell grid it's indexed in) and the
// absolute sensor position and physical (pre-binning) size of that same frame (left/bottom
// matching CAMERA_ROI's convention, i.e. the sensor coordinate of the frame's own local
// origin). Needed by Brillouin::remapProxyRoi() to correctly translate a ROI between two
// frames that don't share a sensor origin (e.g. drawn while zoomed into a sub-region, then
// measured against a different crop) - a naive size-only rescale is only correct when both
// frames start at the same sensor position.
struct PROXY_ROI_FRAME {
	int width{ 0 };
	int height{ 0 };
	long long originLeft{ 0 };
	long long originBottom{ 0 };
	long long widthPhysical{ 0 };
	long long heightPhysical{ 0 };
};

class Brillouin : public AcquisitionMode {
	Q_OBJECT

public:
	Brillouin(QObject* parent, Acquisition* acquisition, Camera*& andor, Camera*& brightfieldCamera, ScanControl*& scanControl);
	~Brillouin();

	BRILLOUIN_SETTINGS& settings{ m_settings };

	// Converts a raw absolute stage/scanner reading (e.g. m_scanControl->getPosition()) into
	// the same frame this Brillouin measurement's own positions-x/y/z is saved in: absolute-
	// grid mode needs the saved origin subtracted (positions-x/y/z is origin-relative there);
	// relative-grid mode needs nothing - raw absolute is already what positions-x/y/z holds.
	// Public so any other acquisition mode capturing images alongside a Brillouin measurement
	// (see Fluorescence::__acquire()) can save its own per-image position through the same,
	// single conversion instead of each re-deriving it (a previous drift between two such
	// re-derivations is exactly what caused the ROI polygon overlay bug this was fixed for).
	POINT3 rawPositionToGridFrame(const POINT3& rawPosition) const;

	// The single source of truth for "where is the grid's absolute-mode origin, right now".
	// m_settings.absoluteGridOriginUm is what the user typed/saved - defined once, in the
	// reference objective's frame, and never rewritten by an objective switch. The active
	// objective's FOV-center offset (see ScanControl::getActiveObjectiveOffset(), empty/zero
	// when no offset calibration exists for the current objective pair - which makes this an
	// exact no-op on any setup that hasn't configured one) is added on top of it here, fresh,
	// every time a plan/position is resolved - never baked back into the stored setting -
	// so switching 10x<->20x any number of times can never double-apply it. Every absolute-
	// mode position computation in this file must go through this, not
	// m_settings.absoluteGridOriginUm directly, for the same reason rawPositionToGridFrame()
	// above is the one conversion everything shares.
	POINT3 resolvedGridOriginUm() const;

	// Remaps a proxy ROI rectangle (in `from`'s binned-cell coordinates) onto `to`'s
	// binned-cell coordinates, via each frame's absolute sensor position. Falls back to a
	// size-only proportional rescale if either frame is missing physical geometry (e.g. a
	// settings file saved before this existed) - still wrong whenever the origin differs,
	// same as before this function existed, but at least doesn't crash or drop the ROI.
	// Returns false (outputs untouched) only if `from`/`to` have a non-positive size.
	static bool remapProxyRoi(
		int roiLeft, int roiTop, int roiWidth, int roiHeight,
		const PROXY_ROI_FRAME& from, const PROXY_ROI_FRAME& to,
		int& outLeft, int& outTop, int& outWidth, int& outHeight
	);

	// Whether the overview/drift/rescan settings can run as set (calibrations, retract, camera...). The GUI asks before starting so the operator
	// gets a message and can change the settings; nothing is created or saved then. Uses the settings last given to setSettings().
	bool validateOverviewSetup(std::string* problem) const { return checkOverviewSetup(problem); }

	// Everything a continue of an interrupted measurement (crash, cancel, pause) needs, read from the file (ResumeFile.h, Brillouin_Resume.cpp)
	using ResumeInfo = resume::Info;
	// Reads it from the file; `storage` must be used on its own thread (call from there). False with the reason if the file holds nothing
	// that can be continued. Opens the repetition it found (no data is changed).
	static bool readResumeInfo(StorageWrapper* storage, ResumeInfo* info, std::string* problem);

public slots:
	void startRepetitions() override;
	// Continues the last measurement of the open file where it stopped: same file, same repetition, the settings of the file. The GUI has
	// checked readResumeInfo() and moved the stage to the start position (relative mode) before.
	void continueRepetition();
	// The operator's answer to the question about the surface scan of an earlier repetition (see s_askSurfacePrior()):
	// 0 new scan from scratch (as always), 1 new scan that starts from the earlier one, 2 no new scan, the earlier surface is used
	void setSurfacePriorAnswer(int choice);
	// The order the axes are scanned in (saved with the settings, in the file and in the application settings)
	SCAN_ORDER scanOrder() const { return m_scanOrder; }
	void setScanOrder(const SCAN_ORDER& order);
	// What the operator allowed for the next continueRepetition(): move the stage to the saved start position (relative mode), turn the
	// nosepiece back to the objective of the measurement (after retracting z) if another one is in the beam
	void setResumeOptions(bool moveToStart, bool switchObjective) {
		m_resumeMoveToStart = moveToStart;
		m_resumeSwitchObjective = switchObjective;
	}

	void waitForNextRepetition();
	void finaliseRepetitions();
	void finaliseRepetitions(int, int);

	// Resumes an acquisition paused at ACQUISITION_STATUS::WAITFORSURFACEREVIEW (see
	// acquire()). fullGrid selects between measuring every grid point (points without a
	// surface z value keep their flat scan-plan default) or only the points that got one.
	// A no-op if not currently paused for review.
	void continueAfterSurfaceReview(bool fullGrid);

	void setStepNumberX(int);
	void setStepNumberY(int);
	void setStepNumberZ(int);

	void setXMin(double);
	void setXMax(double);
	void setYMin(double);
	void setYMax(double);
	void setZMin(double);
	void setZMax(double);

	void setSettings(const BRILLOUIN_SETTINGS& settings);

	/*
	 *	Scan direction order related variables and functions
	 */

	void setScanOrderX(int x);
	void setScanOrderY(int y);
	void setScanOrderZ(int z);

	void setScanOrderAuto(bool automatical);

	void determineScanOrder();

	std::vector<POINT3> getOrderedPositions();
	// Same order/length as getOrderedPositions() - lets a caller match each position back
	// to its (x, y) scan-plan index, e.g. against getSurfaceFoundXYIndices().
	std::vector<INDEX3> getOrderedIndices() const;

	// (x, y) scan-plan index pairs that ended up with a surface z value (directly found
	// or interpolated) after the most recent surface pre-scan - i.e. the points a
	// surface-follow acquisition can actually use. Empty until a surface scan has run.
	std::set<std::pair<int, int>> getSurfaceFoundXYIndices() const;
	// Subset of getSurfaceFoundXYIndices() whose z value leans on at least one gap-filled
	// coarse cell (see the gap-fill pass in runSurfacePreScan()), as opposed to being
	// interpolated purely from genuinely-measured ones.
	std::set<std::pair<int, int>> getSurfaceInterpolatedXYIndices() const;

	// Used by the GUI to draw the overview-mosaic outline in the live view, so these
	// need to be callable from outside the class.
	std::vector<POINT2> overviewTileCentersXY() const;
	// Camera field of view in µm (x, y), as used to size each overview tile.
	POINT2 overviewTileFootprintUm() const;
	// One (topLeft, bottomRight) bounding box per group of active points close enough to be
	// tiled together - i.e. the outline of the area actually covered by that group's tiles,
	// not each individual tile.
	std::vector<std::pair<POINT2, POINT2>> overviewTileOutlinesUm() const;
	// The single grid-center point used by the "single image" coverage mode.
	POINT2 overviewGridCenterXY() const;
	// Coarse xy points (preScanXYBin) the surface pre-scan will actually measure, in the
	// same frame overviewTileCentersXY() uses - the GUI live-view "proposed" preview
	// reuses this instead of independently reconstructing an approximation of the coarse
	// grid from the dense grid's pixel-space bounding box, which could disagree with
	// where the pre-scan really goes (e.g. once ROI masking shrinks that bounding box).
	std::vector<POINT2> surfacePreScanGridXY() const;
	// Background reference points are calibration/sanity-check spectra (e.g. a water
	// Brillouin-shift check), not sample measurements - they get their own HDF5 group and
	// "Background" channel label precisely so they're never mistaken for, or counted alongside,
	// the actual sample grid. An xy grid at the SAME spacing as the main scan's x/y steps
	// (xMax-xMin)/(xSteps-1), but confined to settings.backgroundRoiPolygonUm's own bounding
	// box (which can sit anywhere, including entirely outside the main grid's xMin/xMax/
	// yMin/yMax) and masked to that polygon - deliberately NOT the same rectangle the main grid
	// uses. Plan-frame, same convention as roiPolygonUm/coarseXYSamples(). Empty whenever
	// useBackgroundRoiMask is off or the polygon has fewer than 3 points. Public (unlike
	// captureBackgroundPoints() below) so the GUI's own preview
	// (BrillouinAcquisition::updateBackgroundPositionsPreview()) can show the same points
	// without duplicating this computation.
	std::vector<POINT2> backgroundGridPoints() const;
	// Total number of BF overview images that saveOverviewBrightfieldPerZ will capture
	// across the whole grid (all z-planes combined) - 0 if that option is off. Used by the
	// GUI's estimated-acquisition-time calculation, which needs this count without
	// duplicating overviewCapturePoints()'s logic.
	int overviewImageCountTotal() const;

private:
	// True when the current plan visits the z-planes one after another (z outermost), i.e.
	// the z index never decreases along the traversal. Derived from the ordered indices
	// themselves rather than m_scanOrder, so it always describes the plan that will actually
	// run. Decides how the per-z overview batches are scheduled - see
	// runMeasurementPhase().
	bool isZOutermostInPlan() const;

	void abortMode(std::unique_ptr <StorageWrapper>& storage) override;

	void calibrate(std::unique_ptr <StorageWrapper>& storage);
	void applySurfaceFollowPlan();
	// Surface-follow autofocus's own metric: raw pixel MAX in the spectral proxy ROI box(es)
	// of a single frame, averaged across boxes. Unchanged - see estimateFrameMetricGeneric()
	// for the logic this and estimateFrameMetricFromSum() share.
	double estimateFrameMetric(const std::vector<std::byte>& image) const;
	// Same metric (max in the proxy ROI box(es), averaged across boxes), but reading from a
	// running per-pixel SUM across multiple frames instead of one raw frame - see
	// BRILLOUIN_SETTINGS::extendFramesOnLowSignal's own comment for why adaptive frame
	// extension needs this instead of estimateFrameMetric() on the latest frame alone.
	// `sumBuffer` must be sized camera.roi.width_binned * camera.roi.height_binned, indexed
	// the same raw (unflipped) row-major order as a raw camera frame buffer.
	double estimateFrameMetricFromSum(const std::vector<double>& sumBuffer) const;
	// Shared ROI-remap-and-max-scan logic behind both metrics above, parameterized over how a
	// pixel's value is read (raw dataType-typed bytes for estimateFrameMetric(), a plain
	// double for estimateFrameMetricFromSum()) so the remap/averaging logic itself - the part
	// that actually matters for correctness - exists exactly once.
	double estimateFrameMetricGeneric(
		int width, int height, const std::function<double(int x, int displayY)>& getDisplayValue) const;

	// The actual measurement loop - the back half of what used to be all of acquire(),
	// split out so continueAfterSurfaceReview() can also reach it after a surface-review
	// pause. Assumes the position/calibration vectors and storage metadata are ready.
	void runMeasurementPhase(std::unique_ptr<StorageWrapper>& storage);
	// End-of-repetition bookkeeping (advance the counter, schedule the next repetition or
	// finalize) - shared between waitForNextRepetition()'s normal path and
	// continueAfterSurfaceReview(), which both need to run it exactly once per repetition,
	// whether or not a surface-review pause happened in between.
	void finishRepetition();

	// Outcome of a coarse surface pre-scan: which (x, y) columns actually crossed the
	// medium-reference drop threshold, so the caller can report any that didn't rather
	// than silently leaving their z position at the flat scan-plan default.
	struct SurfaceScanResult {
		bool success{ false };
		int totalColumns{ 0 };
		int failedColumns{ 0 };
	};
	SurfaceScanResult runSurfacePreScan();
	// Coarse xSteps/ySteps reduced by `bin` - indices into the real, dense
	// linspace(xMin,xMax,xSteps)/linspace(yMin,yMax,ySteps) grid, evenly strided (every
	// `bin`-th real grid point, always including the last one). Deliberately snaps to real
	// grid indices rather than independently re-interpolating a new linspace(xMin,xMax,
	// xStepsCoarse) - the latter (this function's previous implementation) can only land
	// exactly on a real measurement point when `bin` happens to evenly divide (xSteps-1),
	// otherwise every coarse/anchor point sits between real points, or even reads as
	// "outside the ROI" when the nearest real point would actually be inside it. Shared by
	// the surface pre-scan's coarse columns and the "sampled grid points" overview coverage
	// mode (via coarseGridXYPoints()).
	std::pair<std::vector<double>, std::vector<double>> coarseXYSamples(int bin) const;
	// Up to `count` extra anchor points (m_settings.additionalBoundaryPoints), on top of the
	// uniform coarse grid coarseXYSamples() already produces. The candidate curve is the ROI
	// polygon clipped to the grid rectangle [xMin,xMax]x[yMin,yMax] (Sutherland-Hodgman - a
	// rectangle is always convex, so this is exact), or the bare rectangle when no ROI is
	// set - either way every returned point is guaranteed on or inside both the grid and the
	// ROI, never outside. Candidates are picked by greedy farthest-point selection (each new
	// point maximizes its minimum distance to every uniform anchor already kept - ROI-
	// filtered, same as coarseGridXYPoints() - and to every boundary point already chosen),
	// then each is snapped independently to its nearest real x index and nearest real y
	// index, same "always land on a real measurement point" rule coarseXYSamples() follows.
	// The result can have fewer than `count` entries: candidates that snap to an index
	// already used by the uniform grid or an earlier boundary point are skipped rather than
	// duplicated. Used by both runSurfacePreScan() (the actual measurement) and
	// surfacePreScanGridXY() (the GUI's live preview), so the two can never drift apart -
	// see planPositionToGridFrame()'s comment for why that matters here specifically.
	std::vector<POINT2> additionalBoundaryXYPoints(int count) const;
	// Captures one real Brillouin spectrum (same frameCount/exposure/gain as the main grid) at
	// every (backgroundGridPoints() x, z-step) combination, AFTER the main grid's own
	// measurement loop finishes - a separate, self-contained pass, not interleaved with it and
	// not counted by any of the main loop's own per-point bookkeeping (lastIndexForZ, per-point
	// brightfield, etc). Deliberately does none of: overview/per-point brightfield capture,
	// surface-follow z search, or ROI-mask exclusion bookkeeping - none of that applies to a
	// second, independent reference region. Written through StorageWrapper::s_enqueueBackground()
	// into the file's separate "background" HDF5 group (H5BM::setBackgroundPayloadData()) -
	// never the main "payload" group's calculateIndex() index space - see that group's own
	// comment in h5bm.h for why this can never collide with or be picked up by anything that
	// reads the main grid.
	void captureBackgroundPoints(std::unique_ptr<StorageWrapper>& storage);
	// Same seed-then-rewind-then-forward-search-with-verification algorithm searchColumn()
	// (local to runSurfacePreScan()) uses, generalized to an arbitrary (x, y) plan-frame
	// position instead of a coarse-grid (xi, yi) index - used for the additional boundary
	// points, which don't have a slot in the rectangular coarse grid to begin with.
	// seedZRel/zTravel/zStep/referenceThreshold are the exact same values runSurfacePreScan()
	// computed for its own rectangular pass. Returns the found z (relative to the grid's z
	// origin, same convention as zSurface[][] there), or std::nullopt if aborted or no
	// surface found within range.
	// onSample, if given, is invoked with (zRel, metric) after every single measurement here -
	// same live per-step detail searchColumn()'s own emitSurfaceProgress() calls show for the
	// rectangular grid, previously missing entirely for boundary points (the caller only ever
	// saw the final found-or-not result, not one z/metric reading along the way).
	std::optional<double> measureBoundarySurfaceZ(
		POINT2 xyPlan, double seedZRel, double zTravel, double zStep, double referenceThreshold,
		const std::function<void(double zRel, double metric)>& onSample = nullptr
	);
	// Converts a "grid-plan" position - the pre-origin frame directionsX/Y/Z,
	// m_settings.roiPolygonUm and coarseXYSamples() are all expressed in (see
	// isPointInPolygonUm() callers, which test roiPolygonUm directly against that frame) -
	// into the same frame positions-x/y/z is actually saved in: already origin-relative in
	// absolute-grid mode (nothing to add - see positionsX/Y/Z in runMeasurementPhase()), or
	// needing +m_startPosition in relative-grid mode to become the raw-absolute value
	// positions-x/y/z holds there. Used for roi-polygon-x/y-um and surface-prescan-x/y-um so
	// both stay in the exact frame positions-x/y/z is in, instead of each re-deriving this
	// (a previous drift between them here is what caused the ROI polygon overlay to land
	// nowhere near its own roi-scan-plan-mask on relative-grid files).
	POINT3 planPositionToGridFrame(const POINT3& planPosition) const;
	// Shared by surfacePreScanGridXY(): coarseXYSamples(bin), ROI-filtered and shifted into
	// the frame overviewTileCentersXY() uses.
	std::vector<POINT2> coarseGridXYPoints(int bin) const;
	// Flat plan z for a z-index - origin.z + directionsZ[zIndex], no surface-follow
	// adjustment. This is the sole z used for the overview image when the active coverage
	// mode's own full-stack toggle (overviewBrightfieldFullStackSingle/Mosaic) is off.
	double overviewFlatZAbs(int zIndex, const std::vector<double>& directionsZ) const;
	// xy point(s) for the overview image itself: the true grid center (single image) or
	// mosaic tile centers (full grid).
	std::vector<POINT2> overviewImageXY() const;
	// z-targets to capture at the overview image's xy point(s) for the given z-index: a
	// single flat value (overviewFlatZAbs(), active full-stack toggle off), or zSteps values
	// spanning either the grid's own zMin..zMax (surface-follow off) or the global
	// lowest-to-highest found surface offset by zMin/zMax (surface-follow on, see
	// m_surfaceZMinAbs/m_surfaceZMaxAbs).
	std::vector<double> overviewStackZAbs(int zIndex, const std::vector<double>& directionsZ) const;
	// One xy/z target per image actually captured for this z-index: the overview image's
	// xy point(s) (see overviewImageXY()) each paired with overviewStackZAbs() (so a full
	// stack, if enabled, only ever applies here).
	struct OverviewCapturePoint {
		POINT2 xy;
		std::vector<double> zAbs;
	};
	std::vector<OverviewCapturePoint> overviewCapturePoints(int zIndex, const std::vector<double>& directionsZ) const;
	void captureOverviewBrightfield(std::unique_ptr <StorageWrapper>& storage, int imageNumber, int zIndex, const POINT3& position);
	// Per-point brightfield capture (capturePerPointBrightfield) - independent of the per-z
	// overview above. See their own doc comments (Brillouin.cpp) for the "after" vs "during"
	// (perPointBrightfieldDuringAcquisition) split and the negative-imageNumber convention
	// that keeps them from colliding with captureOverviewBrightfield()'s own numbering if
	// both are enabled in the same run.
	bool shouldCapturePerPointBrightfield(gsl::index ll) const;
	void capturePerPointBrightfieldImage(std::unique_ptr<StorageWrapper>& storage, gsl::index ll, const POINT3& position);
	std::optional<CAMERA_SETTINGS> startPerPointBrightfieldDuring();
	void finishPerPointBrightfieldDuring(
		std::unique_ptr<StorageWrapper>& storage, gsl::index ll, const POINT3& position, CAMERA_SETTINGS brightfieldSettings);
	// The one place every m_andor frame grab in this class goes through - see its own doc
	// comment (Brillouin.cpp) for how dose protection (useDoseProtection) is folded in here
	// rather than into approachGridPosition() or any single call site, and for openBeam/
	// closeBeam (grabbing several frames back-to-back at one position without flapping the
	// shutter between them).
	void acquireAndorFrame(std::byte* buffer, bool openBeam = true, bool closeBeam = true);
	// Factored out of acquireAndorFrame()'s own closeBeam handling so runMeasurementPhase()'s
	// adaptive frame extension can close the RL shutter explicitly, exactly once, after
	// whichever frame ends up being the truly last one taken at a point (a normal frame if
	// extension didn't trigger or is disabled, or the final extra frame otherwise) - without
	// duplicating the useDoseProtection/setRLShutterOpen logic.
	void closeDoseProtectionShutter();
	// Resting state of the RL Shutter outside an exposure: open for the whole run, or closed
	// with dose protection on. Call after any preset switch back to the Brillouin optics.
	void setRLShutterRestingState();
	// Moves to a grid point during runMeasurementPhase(), honoring
	// useGridHysteresisCompensation (compensated approach vs. a direct move).
	// With the drift offset (see gridOffset()) added to x and y, unless applyGridOffset is false (the medium reference of the surface
	// scan is always taken at the same stage position).
	void approachGridPosition(const POINT3& position, bool applyGridOffset = true);

	std::string getRepetitionFilename();

	/*
	 * Overviews with other objectives, the overview schedule, drift correction and repeated surface scans (Brillouin_Overview.cpp).
	 * Nothing here runs unless one of those settings differs from its default: then the measurement loop is exactly the one that has
	 * always run (see useOverviewEngine()).
	 */

	// The footprint of one camera frame in stage um for the given objective's scale calibration (overviewTileFootprintUm() is the one of the
	// active objective), and the tile centres that cover the grid with frames of that footprint.
	POINT2 overviewTileFootprintUmFor(const ScaleCalibrationData& scaleCalibration) const;
	std::vector<POINT2> overviewTileCentersXYForFootprint(POINT2 footprintUm) const;
	// Whether overview images (of any objective) are expected in this run, i.e. the Fluorescence part of the file is needed.
	bool overviewImagesExpected() const;

	// One objective that takes overviews in this repetition
	struct OverviewObjectivePlan {
		int slot{ -1 };
		bool isMeasurementObjective{ false };
		OVERVIEW_OBJECTIVE_SETTINGS settings;
	};

	// What one objective's part of an overview event did (written to the file and logged)
	struct OverviewCaptureLog {
		int slot{ -1 };
		bool isMeasurementObjective{ false };
		bool captured{ false };
		std::string skippedReason;
		int firstImageNumber{ 0 };
		int tiles{ 0 };
		int stackDepth{ 0 };
		int imagesSaved{ 0 };
		double retractUm{ 0.0 };
		double switchToSeconds{ 0.0 };
		double captureSeconds{ 0.0 };
		double switchBackSeconds{ 0.0 };
		POINT2 fovCompensationUm{ 0.0, 0.0 };
		double focusDeltaUm{ 0.0 };
		POINT3 positionBeforeUm{ 0.0, 0.0, 0.0 };
	};

	// Result of the drift measurement of one overview event
	struct OverviewDriftLog {
		bool evaluated{ false };
		drift::DriftDecision decision{ drift::DriftDecision::REJECTED_NO_MATCH };
		POINT2 stepUm{ 0.0, 0.0 };
		POINT2 totalUm{ 0.0, 0.0 };
		POINT2 gridOffsetUm{ 0.0, 0.0 };
		double meanNcc{ 0.0 };
		int tilesCompared{ 0 };
		int tilesUsed{ 0 };
		double secondsSinceReference{ 0.0 };
		double registrationSeconds{ 0.0 };
	};

	// True if any setting that needs the overview engine is on; false: the measurement loop is the original one.
	bool useOverviewEngine() const;
	// The objectives that take overviews in this repetition (measuring objective first), from the settings.
	std::vector<OverviewObjectivePlan> planOverviewObjectives() const;
	// Whether the overview/drift/rescan settings can be run: other objectives need calibrations (scale, FOV offset, focus offset), a z
	// retract, brightfield camera; the drift objective must take overviews. `problem` says what is missing.
	bool checkOverviewSetup(std::string* problem) const;
	// Start of a repetition's measurement phase: plans the objectives, carries the drift over from the previous repetition (or starts
	// from none), writes the settings used and the calibrations of the overview objectives to the file.
	void beginOverviewSeries(std::unique_ptr<StorageWrapper>& storage);
	// Moments due before the first point / after point ll: one overview event for each (all objectives that take overviews, then the drift
	// update). Returns false if the acquisition was aborted.
	bool runOverviewMoments(
		std::unique_ptr<StorageWrapper>& storage, const std::vector<OverviewMoment>& moments, const std::vector<double>& directionsZ,
		gsl::index pointIndex
	);
	bool runOverviewEvent(
		std::unique_ptr<StorageWrapper>& storage, const OverviewMoment& moment, const std::vector<double>& directionsZ, gsl::index pointIndex
	);
	// One objective's overview of one event: for another objective the whole cycle (retract, switch, focus, capture, switch back, restore).
	// The centre slices of its tiles go to `driftTiles` if given.
	bool captureObjectiveOverview(
		std::unique_ptr<StorageWrapper>& storage, const OverviewObjectivePlan& plan, const OverviewMoment& moment,
		const std::vector<double>& directionsZ, int eventIndex, int objectiveOrder, gsl::index pointIndex,
		std::vector<drift::TileFrame>* driftTiles, OverviewCaptureLog* log
	);
	// Retract z, turn the nosepiece back to the measuring objective and put the stage back where it was (or leave it retracted).
	void returnToMeasurementObjective(int measurementSlot, const POINT3& restorePosition, bool restorePositionToo);
	// One frame of the brightfield camera with the given exposure/gain.
	bool grabBrightfieldFrame(int exposureMs, double gain, std::vector<std::byte>* image, CAMERA_SETTINGS* usedSettings);
	// Registers the tiles of an overview of the drift objective against the reference, updates the controller and the grid offset.
	void processDriftOverview(const std::vector<drift::TileFrame>& tiles, int slot, double timeSeconds, OverviewDriftLog* log);
	// The offset that every stage target of the grid and the overviews carries (the drift, 0 if drift correction is off).
	POINT2 gridOffset() const { return m_gridOffset; }
	void refreshGridOffset();
	// Repeats the surface scan after point ll and moves the remaining points to the new surface (everything of it is saved).
	void repeatSurfaceScan(std::unique_ptr<StorageWrapper>& storage, gsl::index pointIndex, const std::vector<double>& directionsZ);
	// Writes an event's record to the file, and the offset every measured point carried.
	void writeOverviewEvent(
		std::unique_ptr<StorageWrapper>& storage, int eventIndex, const OverviewMoment& moment, gsl::index pointIndex, double timeSeconds,
		const std::vector<OverviewCaptureLog>& captures, const OverviewDriftLog& driftLog
	);
	void writePointGridOffsets(std::unique_ptr<StorageWrapper>& storage);
	// Seconds on a steady clock
	static double steadySeconds();

	// ---- continue of an interrupted measurement (Brillouin_Resume.cpp)
	// Writes what a continue needs (settings, plan, start position, state) to the file; before the queues are started (direct) or later
	void writeResumeRecord(std::unique_ptr<StorageWrapper>& storage);
	void writeResumePlan(std::unique_ptr<StorageWrapper>& storage, bool direct);
	void setStorageState(const std::string& state);
	bool m_resuming{ false };
	// true while a measurement is in its measuring phase: what is written to the file as its state, see setStorageState()
	bool m_measuringState{ false };
	// true from the surface scan on until the measuring phase starts: the surface scan writes what a continue needs (checkpoint per column)
	bool m_surfacePhaseState{ false };
	bool m_checkpointSurface{ false };
	// a continue of a measurement that stopped during its surface scan: the scan starts from the checkpoint in m_resumeInfo
	bool m_resumeSurface{ false };
	bool m_resumeMoveToStart{ true };
	bool m_resumeSwitchObjective{ false };
	void writeResumeBasics(std::unique_ptr<StorageWrapper>& storage);

	// ---- live check (Brillouin_Overview.cpp): PNG previews and a log next to the file
	std::string livePreviewFolder() const;
	void writeLivePreview(const std::string& name, const std::vector<std::byte>& image, const CAMERA_SETTINGS& settings);
	void appendLiveLog(const std::string& line);
	bool m_livePreviewFailed{ false };
	std::shared_ptr<LivePreviewWorker> m_livePreviewWorker;		// writes the previews in the background
	void writeSurfaceCheckpointReference(double mediumReference, size_t columnsX, size_t columnsY);
	void writeSurfaceCheckpointColumn(gsl::index xi, gsl::index yi, bool valid, double z, int order, double metric,
		const std::vector<double>& curveZ, const std::vector<double>& curveMetric);
	// Back to the normal writing of the file after a continue has put its repetition in place
	void endResumeMode(std::unique_ptr<StorageWrapper>& storage);
	int m_resumeStartIndex{ 0 };			// first point of the plan that is measured (0 for a new measurement)
	ResumeInfo m_resumeInfo;

	// ---- the surface scan of an earlier repetition of the file (Brillouin_Resume.cpp)
	enum class SURFACE_STRATEGY { NEW_SCAN = 0, PRIOR = 1, REUSE = 2 };
	using PreviousSurface = resume::PreviousSurface;
	// Asks the operator (popup with a timer, default: start from the earlier scan) if the file holds a surface scan of an earlier repetition
	SURFACE_STRATEGY chooseSurfaceStrategy(std::unique_ptr<StorageWrapper>& storage, PreviousSurface* previous);
	void applyPreviousSurface(const PreviousSurface& previous);
	int m_surfacePriorAnswer{ -1 };
	POINT2 m_resumeCarry{ 0.0, 0.0 };		// grid offset a continued measurement starts with (becomes the carried offset of the drift reference)

	std::vector<OverviewObjectivePlan> m_overviewPlans;
	drift::DriftController m_driftController;
	std::vector<drift::TileFrame> m_driftReferenceTiles;	// the last accepted overview of the drift objective
	int m_driftReferenceSlot{ -1 };
	POINT3 m_driftSeriesStart{ 0.0, 0.0, 0.0 };				// start position of the repetition the reference belongs to
	POINT2 m_gridOffset{ 0.0, 0.0 };						// [um] drift applied to every stage target
	double m_seriesStartSeconds{ 0.0 };						// steady clock at the start of the measurement phase
	int m_overviewEventCount{ 0 };
	int m_surfaceRescanCount{ 0 };
	int m_measurementObjectiveSlot{ -1 };
	// per measured point (planned order): the offset it was measured with, NaN for points not measured
	std::vector<double> m_pointOffsetX;
	std::vector<double> m_pointOffsetY;

	// Populated at the end of runSurfacePreScan() - see getSurfaceFoundXYIndices().
	std::set<std::pair<int, int>> m_surfaceFoundXYIndices;
	// Populated at the end of runSurfacePreScan() - see getSurfaceInterpolatedXYIndices().
	std::set<std::pair<int, int>> m_surfaceInterpolatedXYIndices;
	// Global min/max of the found surface (absolute z, same convention as
	// zCenterByXYIndex in runSurfacePreScan()) - used by overviewStackZAbs() to build a
	// full-stack z range that covers every xy tile's surface, not just one neighbor's.
	// Only valid (m_surfaceZRangeValid) after a surface pre-scan actually found at least
	// one point.
	double m_surfaceZMinAbs{ 0.0 };
	double m_surfaceZMaxAbs{ 0.0 };
	bool m_surfaceZRangeValid{ false };

	// Populated at the end of runSurfacePreScan() - the coarse (binned) grid the pre-scan
	// itself actually measured on, before any interpolation onto the dense scan-plan grid.
	// (x, y) in the same relative-to-start-position um convention runSurfacePreScan()
	// already uses for xSamples/ySamples. Saved verbatim (see runMeasurementPhase()) so the
	// genuinely sparse raw measurement locations survive into the file, distinct from the
	// dense, already-interpolated surface-found-mask. Empty if the pre-scan never ran or
	// bailed out before any column was even attempted.
	std::vector<double> m_surfacePreScanXUm;
	std::vector<double> m_surfacePreScanYUm;
	// Per coarse-grid cell, flat-indexed xi * m_surfacePreScanYUm.size() + yi (mirroring
	// surface-found-mask's own x-major flat indexing), before any dense-grid interpolation:
	// - m_surfacePreScanFoundMask: 0 = no drop found within the travel range, 1 = a
	//   genuine verified measurement, 2 = filled in from neighboring coarse cells (see the
	//   coarse gap-fill pass in runSurfacePreScan()) - same 0/1/2 meaning as the dense
	//   surface-found-mask, just at the coarse pre-scan's own resolution.
	// - m_surfacePreScanZUm: the z (relative to this column's own zOrigin) at which the
	//   surface was found/filled - 0.0 where m_surfacePreScanFoundMask == 0.
	// - m_surfacePreScanMetric: the drop metric actually measured at that column (frame-
	//   averaged where verification ran) - the verified value on success, otherwise
	//   whatever was last measured before giving up on that column, so a failed column's
	//   metric can still be inspected (e.g. "how close did it get").
	std::vector<double> m_surfacePreScanFoundMask;
	std::vector<double> m_surfacePreScanZUm;
	std::vector<double> m_surfacePreScanMetric;
	// Populated at the end of runSurfacePreScan() - the additional boundary points that
	// actually found a surface (see additionalBoundaryXYPoints()/measureBoundarySurfaceZ()),
	// one entry per point: x, y in the same plan-frame convention as m_surfacePreScanXUm/YUm,
	// z relative to this scan's own zOrigin (same convention as m_surfacePreScanZUm). A
	// requested point that found nothing, or wasn't reached because the scan aborted, simply
	// has no entry here - unlike the rectangular coarse grid, there is no fixed slot for it
	// to occupy "empty". Empty if additionalBoundaryPoints was 0 or none of them found a
	// surface.
	std::vector<POINT3> m_surfaceBoundaryPointsUm;
	// The full (z, metric) trace measured at each coarse pre-scan column, in the order the
	// samples were actually taken (seed, then rewind steps if any, then the forward walk,
	// then the verification window on a candidate crossing) - unlike m_surfacePreScanMetric
	// (which only keeps the single found/last-measured value), this is what lets a reader
	// reconstruct and plot the actual intensity-drop curve behind each column's result, not
	// just its outcome. Rectangular [xSamples.size(), ySamples.size(),
	// m_surfaceCurveMaxSamples]-shaped, flat-indexed (xi * ySamples.size() + yi) *
	// m_surfaceCurveMaxSamples + k for sample k (mirroring the other coarse arrays' x-major
	// convention), NaN-padded past each column's own m_surfaceCurveSampleCounts - sized to
	// the longest column's trace actually needed during this scan, not a fixed worst-case
	// bound. Empty under the same conditions m_surfacePreScanXUm is.
	std::vector<double> m_surfaceCurveZUm;
	std::vector<double> m_surfaceCurveMetric;
	// Per coarse-grid cell (flat-indexed xi * ySamples.size() + yi, same as
	// m_surfacePreScanFoundMask), how many samples that column's trace actually holds in
	// m_surfaceCurveZUm/Metric before the NaN padding starts.
	std::vector<double> m_surfaceCurveSampleCounts;
	int m_surfaceCurveMaxSamples{ 0 };
	// The medium-reference-derived drop threshold actually used by runSurfacePreScan() -
	// (1 - surfaceDropFraction) * mediumReferenceValue, or NaN if mediumReferenceValue was
	// ~0 (see runSurfacePreScan()). Saved directly (see runMeasurementPhase()) so a reader
	// doesn't have to reconstruct it from those two fields - and isn't misled by the
	// separate, currently-unused settings.surfaceMetricThreshold UI preference saved
	// elsewhere, which this scan never actually reads.
	double m_surfaceReferenceThreshold{ std::numeric_limits<double>::quiet_NaN() };
	// A repeated surface scan (repeatSurfaceScan()) starts from the previous one: the coarse surface z of every column of the last scan
	// (flat-indexed like m_surfacePreScanZUm, valid where > 0 in m_surfacePriorValid) seeds the search. The first column found gives the
	// general offset between the two scans (new minus old surface), the median of all offsets found so far is added to the old surface
	// of every later column. Empty (the first scan of a measurement, and every legacy run): the neighbour-seeded search as always.
	std::vector<double> m_surfacePriorZUm;
	std::vector<double> m_surfacePriorValid;
	// Output of the last runSurfacePreScan(): median offset to the prior scan (NaN without prior) and how many columns gave an offset
	double m_surfacePriorOffsetUm{ std::numeric_limits<double>::quiet_NaN() };
	int m_surfacePriorOffsetCount{ 0 };

	BRILLOUIN_SETTINGS m_settings;
	SCAN_ORDER m_scanOrder;
	Camera*& m_andor;
	Camera*& m_brightfieldCamera;
	bool m_running{ false };				// is acquisition currently running
	POINT3 m_startPosition{ 0, 0, 0 };

	QTimer* m_repetitionTimer{ nullptr };
	QElapsedTimer m_startOfLastRepetition;
	int m_currentRepetition{ 0 };
	// The key of the repetition group the current repetition is written to in the file
	// ('Brillouin/<key>'). m_currentRepetition restarts at 0 on every Start, while the key keeps
	// counting across Starts that write to the same file (aborted/restarted measurements), so the
	// two differ from the second Start on. Images record this one as brillouin_repetition_index,
	// which is what readers (bmlab) match against the repetition group names.
	int m_fileRepetition{ 0 };

	int nrCalibrations{ 1 };

	std::string m_baseFilename{ "" };

	std::vector<POINT3> m_orderedPositions;	// The positions to measure in absolute values
	std::vector<POINT3> m_orderedPositionsRelative;	// The positions to measure relative to start position
	std::vector<INDEX3> m_orderedIndices;	// The associated indices
	std::vector<bool> m_calibrationAllowed;	// If a calibration is allowed for this position
	// Grid points the ROI mask excluded from the plan - preview-only, see ScanPlannerOutput.
	std::vector<POINT3> m_excludedPositions;
	std::vector<POINT3> m_excludedPositionsRelative;

private slots:
	void acquire(std::unique_ptr <StorageWrapper>& storage) override;

	void updatePositions();

	// "Set plane": zeroes the z grid at the current focus position without touching x/y -
	// the z counterpart to ScanControl::setHome(), offered specifically as its replacement in
	// absolute grid-coordinate mode, where Set home is disabled (see BrillouinAcquisition's
	// on_setHome_clicked()/homeControlsDisabled). Unlike x/y, the z grid origin is now always
	// m_startPosition.z regardless of gridCoordinatesAbsolute (see resolvedGridOriginUm()'s own
	// comment for why) - so this is the only way to re-anchor it before the next "Start"
	// overwrites it anyway with a fresh live capture.
	void setCurrentFocusAsZOrigin();

signals:
	// current position in x, y and z, as well as the current image number
	void s_positionChanged(POINT3, int);
	void s_timeToCalibration(int);	// time to next calibration
	void s_calibrationRunning(bool);	// is calibration running
	void s_scanOrderChanged(SCAN_ORDER);
	// The mode flag travels with the positions rather than being re-read by the receiver -
	// this signal crosses threads via a queued connection, so by the time a receiver's slot
	// actually runs, gridCoordinatesAbsolute may already have changed again (e.g. the operator
	// toggled the checkbox right after this emit). Re-deriving "was this absolute" from the
	// live setting at receive time would then reinterpret positions computed under the OLD mode
	// using the NEW mode's pixel-conversion formula - a purely visual grid/marker misalignment
	// with no effect on the actual measurement (which reads the flag synchronously, on this
	// same thread, right when it plans/moves) - see BrillouinAcquisition::AOI_changed().
	void s_orderedPositionsChanged(std::vector<POINT3>, bool isAbsolute);
	void s_excludedPositionsChanged(std::vector<POINT3>);
	void s_surfaceScanProgress(double progress, QString message);
	// A surface scan of an earlier repetition is in the file: ask the operator how to use it. The answer comes back through
	// setSurfacePriorAnswer(); without one after timeoutSeconds the GUI answers 1 (start from the earlier scan) if that is offered, else 0.
	void s_askSurfacePrior(QString text, bool offerPrior, bool offerReuse, int timeoutSeconds);
	// elapsed wall-clock time since the main measurement grid started, in seconds - drives the
	// live elapsed-time display that replaces the static pre-run estimate in estimatedAcquisitionTime
	// once a measurement is actually running (see BrillouinAcquisition::showMeasurementElapsedTime()).
	void s_measurementElapsed(int seconds);
};

#endif //BRILLOUIN_H

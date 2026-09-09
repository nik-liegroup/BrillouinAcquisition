#ifndef BRILLOUIN_H
#define BRILLOUIN_H

#include "AcquisitionMode.h"
#include "../../Devices/Cameras/Camera.h"
#include "../../helper/thread.h"
#include "src/lib/buffer_circular.h"
#include <limits>
#include <optional>
#include <set>
#include <utility>


struct SCAN_ORDER {
	bool automatical{ true };
	int x{ 0 };	// first scan in x-direction
	int y{ 1 };	// then in y-direction
	int z{ 2 };	// scan in z-direction last
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
			useSurfaceFollow = settings.useSurfaceFollow;
			surfaceZOffsetUm = settings.surfaceZOffsetUm;
			surfaceFollowHalfRangeUm = settings.surfaceFollowHalfRangeUm;
			surfaceMaxRewindUm = settings.surfaceMaxRewindUm;
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
			surfaceSmoothSigmaUm = settings.surfaceSmoothSigmaUm;
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
			overviewBrightfieldSampledGrid = settings.overviewBrightfieldSampledGrid;
			overviewBrightfieldBin = settings.overviewBrightfieldBin;
			overviewBrightfieldFullStack = settings.overviewBrightfieldFullStack;
			useGridHysteresisCompensation = settings.useGridHysteresisCompensation;
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
		bool useSurfaceFollow{ false };
		double surfaceZOffsetUm{ 0.0 };
		double surfaceFollowHalfRangeUm{ 10.0 };
		// Neighbor-seeded surface search: each coarse column starts its search from the
		// nearest already-found neighbor's z instead of a blind full-range sweep.
		double surfaceMaxRewindUm{ 15.0 };			// max distance allowed stepping back towards water before giving up
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
		double surfaceSmoothSigmaUm{ 5.0 };
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
		// "Sampled grid points": an independent, additive option (not exclusive with
		// overviewBrightfieldFullGrid above) - additionally captures one flat-z image at a
		// coarse-binned subset of the real measurement grid (overviewBrightfieldBin, same
		// coarse-binning logic as preScanXYBin) alongside whichever overview image is
		// configured. A full stack (see overviewBrightfieldFullStack below) never applies
		// to these - they always capture a single flat image per point.
		bool overviewBrightfieldSampledGrid{ false };
		int overviewBrightfieldBin{ 1 };
		// If false, one flat-z overview image per finished z slice (legacy behaviour: the
		// plane's own zMin..zMax offset from the grid origin, no surface-follow
		// adjustment). If true, one full z-stack per finished z slice instead - spanning
		// zMin..zMax (surface-follow off) or the global lowest-to-highest found surface,
		// offset by zMin/zMax (surface-follow on), always sampled at zSteps points. Only
		// ever applies to the overview image itself, never to "sampled grid points" above.
		bool overviewBrightfieldFullStack{ false };
		// Whether stepping from one measurement grid point to the next approaches it from
		// a consistent direction first (ScanControl::setPositionCompensated()) to cancel
		// out stage hysteresis/backlash - accurate but costs an extra move + 100 ms settle
		// per grid point. Off skips straight to the target (ScanControl::setPosition()) for
		// faster stepping, at the cost of potential backlash error. Only affects the main
		// grid-to-grid stepping in runMeasurementPhase(); the surface pre-scan always
		// compensates, since accuracy matters more there than speed.
		bool useGridHysteresisCompensation{ true };

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

public slots:
	void startRepetitions() override;

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
	// Coarse-binned real grid points (overviewBrightfieldBin), ROI-filtered and shifted
	// into the same absolute frame overviewTileCentersXY() uses - the GUI live-view
	// overlay draws these as markers for "sampled grid points", an option independent of
	// (and additive to) the single-image/full-grid overview image - see
	// overviewCapturePoints().
	std::vector<POINT2> overviewSampledGridXY() const;
	// The single grid-center point used by the "single image" coverage mode.
	POINT2 overviewGridCenterXY() const;
	// Coarse xy points (preScanXYBin) the surface pre-scan will actually measure, in the
	// same frame overviewTileCentersXY() uses - the GUI live-view "proposed" preview
	// reuses this instead of independently reconstructing an approximation of the coarse
	// grid from the dense grid's pixel-space bounding box, which could disagree with
	// where the pre-scan really goes (e.g. once ROI masking shrinks that bounding box).
	std::vector<POINT2> surfacePreScanGridXY() const;
	// Total number of BF overview images that saveOverviewBrightfieldPerZ will capture
	// across the whole grid (all z-planes combined) - 0 if that option is off. Used by the
	// GUI's estimated-acquisition-time calculation, which needs this count without
	// duplicating overviewCapturePoints()'s logic.
	int overviewImageCountTotal() const;

private:
	void abortMode(std::unique_ptr <StorageWrapper>& storage) override;

	void calibrate(std::unique_ptr <StorageWrapper>& storage);
	void applySurfaceFollowPlan();
	double estimateFrameMetric(const std::vector<std::byte>& image) const;

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
	// Same seed-then-rewind-then-forward-search-with-verification algorithm searchColumn()
	// (local to runSurfacePreScan()) uses, generalized to an arbitrary (x, y) plan-frame
	// position instead of a coarse-grid (xi, yi) index - used for the additional boundary
	// points, which don't have a slot in the rectangular coarse grid to begin with.
	// seedZRel/zTravel/zStep/referenceThreshold are the exact same values runSurfacePreScan()
	// computed for its own rectangular pass. Returns the found z (relative to the grid's z
	// origin, same convention as zSurface[][] there), or std::nullopt if aborted or no
	// surface found within range.
	std::optional<double> measureBoundarySurfaceZ(
		POINT2 xyPlan, double seedZRel, double zTravel, double zStep, double referenceThreshold
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
	// Shared by overviewSampledGridXY()/surfacePreScanGridXY(): coarseXYSamples(bin),
	// ROI-filtered and shifted into the frame overviewTileCentersXY() uses.
	std::vector<POINT2> coarseGridXYPoints(int bin) const;
	// Flat plan z for a z-index - origin.z + directionsZ[zIndex], no surface-follow
	// adjustment. This is the sole z used for "sampled grid points" (always) and for the
	// overview image when overviewBrightfieldFullStack is off.
	double overviewFlatZAbs(int zIndex, const std::vector<double>& directionsZ) const;
	// xy point(s) for the overview image itself: the true grid center (single image) or
	// mosaic tile centers (full grid) - NOT "sampled grid points", which is an independent,
	// additive option handled separately in overviewCapturePoints().
	std::vector<POINT2> overviewImageXY() const;
	// z-targets to capture at the overview image's xy point(s) for the given z-index: a
	// single flat value (overviewFlatZAbs(), overviewBrightfieldFullStack off), or zSteps
	// values spanning either the grid's own zMin..zMax (surface-follow off) or the global
	// lowest-to-highest found surface offset by zMin/zMax (surface-follow on, see
	// m_surfaceZMinAbs/m_surfaceZMaxAbs).
	std::vector<double> overviewStackZAbs(int zIndex, const std::vector<double>& directionsZ) const;
	// One xy/z target per image actually captured for this z-index: the overview image's
	// xy point(s) (see overviewImageXY()) each paired with overviewStackZAbs() (so a full
	// stack, if enabled, only ever applies here), plus - additionally, independently of the
	// overview image's own settings - "sampled grid points" (overviewSampledGridXY()) if
	// that option is on, each paired with a single flat overviewFlatZAbs() (a full stack
	// never applies to sampled grid points, no matter how many of them there are).
	struct OverviewCapturePoint {
		POINT2 xy;
		std::vector<double> zAbs;
	};
	std::vector<OverviewCapturePoint> overviewCapturePoints(int zIndex, const std::vector<double>& directionsZ) const;
	void captureOverviewBrightfield(std::unique_ptr <StorageWrapper>& storage, int imageNumber, int zIndex, const POINT3& position);
	// Moves to a grid point during runMeasurementPhase(), honoring
	// useGridHysteresisCompensation (compensated approach vs. a direct move).
	void approachGridPosition(const POINT3& position);

	std::string getRepetitionFilename();

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
	// The medium-reference-derived drop threshold actually used by runSurfacePreScan() -
	// (1 - surfaceDropFraction) * mediumReferenceValue, or NaN if mediumReferenceValue was
	// ~0 (see runSurfacePreScan()). Saved directly (see runMeasurementPhase()) so a reader
	// doesn't have to reconstruct it from those two fields - and isn't misled by the
	// separate, currently-unused settings.surfaceMetricThreshold UI preference saved
	// elsewhere, which this scan never actually reads.
	double m_surfaceReferenceThreshold{ std::numeric_limits<double>::quiet_NaN() };

	BRILLOUIN_SETTINGS m_settings;
	SCAN_ORDER m_scanOrder;
	Camera*& m_andor;
	Camera*& m_brightfieldCamera;
	bool m_running{ false };				// is acquisition currently running
	POINT3 m_startPosition{ 0, 0, 0 };

	QTimer* m_repetitionTimer{ nullptr };
	QElapsedTimer m_startOfLastRepetition;
	int m_currentRepetition{ 0 };

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

signals:
	// current position in x, y and z, as well as the current image number
	void s_positionChanged(POINT3, int);
	void s_timeToCalibration(int);	// time to next calibration
	void s_calibrationRunning(bool);	// is calibration running
	void s_scanOrderChanged(SCAN_ORDER);
	void s_orderedPositionsChanged(std::vector<POINT3>);
	void s_excludedPositionsChanged(std::vector<POINT3>);
	void s_surfaceScanProgress(double progress, QString message);
};

#endif //BRILLOUIN_H

#ifndef BRILLOUIN_H
#define BRILLOUIN_H

#include "AcquisitionMode.h"
#include "../../Devices/Cameras/Camera.h"
#include "../../helper/thread.h"
#include "src/lib/buffer_circular.h"
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
			surfaceProxyRoi2FrameWidth = settings.surfaceProxyRoi2FrameWidth;
			surfaceProxyRoi2FrameHeight = settings.surfaceProxyRoi2FrameHeight;
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
		// Frame size the two ROIs above were drawn against. If the camera's actual frame
		// size at measurement time (surface pre-scan, or a live preview reconfigured since)
		// differs - e.g. a different ROI/binning was active - the stored rectangles are
		// rescaled proportionally against these before use, instead of being applied as raw,
		// now-mismatched pixel coordinates (which could clamp them to nothing and silently
		// fall back to measuring the whole frame). 0 means "never drawn yet", in which case
		// no rescaling is attempted.
		int surfaceProxyRoiFrameWidth{ 0 };
		int surfaceProxyRoiFrameHeight{ 0 };
		int surfaceProxyRoi2FrameWidth{ 0 };
		int surfaceProxyRoi2FrameHeight{ 0 };
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

class Brillouin : public AcquisitionMode {
	Q_OBJECT

public:
	Brillouin(QObject* parent, Acquisition* acquisition, Camera*& andor, Camera*& brightfieldCamera, ScanControl*& scanControl);
	~Brillouin();

	BRILLOUIN_SETTINGS& settings{ m_settings };

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
	// Coarse xSteps/ySteps reduced by `bin`, evenly spaced over [xMin,xMax]/[yMin,yMax] -
	// the same logic runSurfacePreScan() uses for its coarse pre-scan columns, shared here
	// so the "sampled grid points" overview coverage mode reuses it verbatim rather than
	// re-implementing a different notion of "every Nth point".
	std::pair<std::vector<double>, std::vector<double>> coarseXYSamples(int bin) const;
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

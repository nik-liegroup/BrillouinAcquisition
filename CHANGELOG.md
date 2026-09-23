## Unreleased

### Fixed
- Progress bar: fixed an off-by-two error that overestimated remaining time
  by ~2 points' worth throughout a run.
- Dose-protection shutter is now guaranteed to close on every abort path
  during acquisition (previously could be left open if aborting mid-run),
  via a small RAII guard used in the main measurement loop, background
  point capture, and surface pre-scan.
- Large grids with many z steps made the software noticeably sluggish
  while editing grid settings: the position-overlay markers on the
  brightfield preview were plotted once per (x, y, z) point even though
  z has no effect on where a point projects on screen - every z-layer of
  a column landed on the exact same pixel and was reprojected/redrawn
  redundantly. The overlay now plots (and connects) one marker per unique
  (x, y) column regardless of zSteps.
- "Set plane" (absolute grid mode) now also zeroes the live z position
  readout at the point it's clicked, matching what switching to relative
  mode, clicking "Set home", and switching back used to give as a manual
  workaround - previously only the internal grid z-origin was updated, so
  the measurement grid was already centered correctly but the on-screen
  number never actually read 0.
- The "Est. time" field next to the Start button was frozen at its
  pre-run estimate for the whole run. It now shows live elapsed time
  while a measurement is running, then reverts to the (refreshed)
  pre-run estimate once it finishes or is aborted.
- Surface pre-scan progress no longer matched what was actually happening:
  the coarse grid and boundary points were numbered/reported separately
  by (x, y) grid index, and the final dense-grid interpolation pass used
  its own independent 0-100% scale - so the bar visibly reset backward
  right when interpolation began. Progress is now one running count of
  points actually mapped (coarse columns + boundary points combined, in
  a single "point N/M" numbering) out of the true total, advancing only
  once each point's search concludes; interpolation (which maps no new
  point) holds at 100% instead of restarting its own scale.
- Surface pre-scan: the "additional boundary points" pass (opt-in, extra
  anchor points outside the main rectangular coarse grid) only ever
  reported "Boundary point X/Y" with no metric/z/threshold - unlike every
  other point in the pre-scan, which shows that detail live. It now shows
  the same per-step detail throughout.

### Added
- Adaptive frame extension: an optional per-point acquisition extension
  (BM Camera tab, "Adaptive frame extension" group) that reuses the
  existing spectral proxy-ROI boxes (already used for surface-follow
  autofocus) to take extra frames - up to a configurable maximum - at a
  grid point while its signal stays below a configurable threshold,
  instead of always accepting a fixed frame count. The decision is based
  on the running SUM of every frame taken so far at that point (not any
  single frame's own reading, which wouldn't improve just because earlier
  frames exist), via the new `estimateFrameMetricFromSum()`. The
  threshold is a fixed, absolute target (no rescaling as extra frames are
  added) - calibrate it directly against "max counts in the image that
  actually gets fit" for your normal frame-repeat count. Independent of
  surface-follow scanning. The "Draw spectral ROIs"/"Measure spectral
  ROIs" controls moved from the Surface scanning group into this new
  group (same widget names/behavior).

### Changed
- Estimated acquisition time ("Est. time") recalibrated against the image
  timestamps of seven real scans (3000-3400 points each): per-point
  overhead beyond the exposure, a cost per moving x/y stage axis (instead
  of distance / an assumed stage speed), the measured cost of overview
  batches (~1.6 s + ~0.54 s per image - previously ~0.1 s per image, about
  5x too low) and of spectrometer calibrations, and "after" per-point
  brightfield now respects "every N". On those seven scans the old
  estimate was ~13% too high (excluding its surface pre-scan term); the new
  one is within ~3%. The surface pre-scan is no longer part of the
  estimate (its length depends on the sample); the tooltip says so.
- Automatic scan order now always keeps z as the outermost loop (plane by
  plane); only x and y are ordered by step count. Previously z became an
  inner loop whenever it had more steps than x or y.
- With a manual scan order that does not have z outermost, per-z overview
  images are switched off. Turning them back on asks for confirmation; they
  are then taken as the same number of batches as with z outermost, but at
  points spread evenly across the whole acquisition, the last one after all
  grid points have been scanned (no z-plane is finished earlier). New file
  metadata: `overview-brightfield-spread-across-scan-used` and
  `grid-scan-order-used`.

## 0.3.5 - 2025-07-31

### Added
- Simplify project compilation with .props file

### Fixed
- Use correct zeiss SDK character (µm) to fix encoding issue

## 0.3.4 - 2023-06-12

### Changed
- Use tubelens 1 for Eyepiece preset c800decc537bc79e8b385860a4b9307af2fdc95b

## 0.3.3 - 2023-01-02

### Changed
- Code cleanup #242

### Fixed
- Validate ROI settings correctly #243

## 0.3.2 - 2022-12-14

### Added
- Save all parameters and restore on program startup #196

### Changed
- Code cleanup #232 #233 #235

### Fixed
- Prevent out-of-range error #237
- Fix possible null-pointer dereference #238
- Prevent possible arithmetic overflows #239

## 0.3.1 - 2022-12-05

### Changed
- Dependency updates #225 #226 #228 #229
- Code cleanup #223 #224 #227 #230

## 0.3.0 - 2022-09-05

### Added
- Allow to chose camera number #215

### Changed
- Move relative when clicking into image #220
- Code cleanup #216

### Fixed
- Fix crash for scale calibration without camera #213
- Don't crash if calibration file cannot be written #214

## 0.2.2 - 2021-08-12

### Fixed
- Fix connectivity to Thorlabs devices #206

## 0.2.1 - 2021-08-10

### Fixed
- Correct position preview after aborting a measurement #198
- Adjust preview settings #202
- Correct mirror positions for FOB setup #200
- Fix calculation of bytes per frame on Andor #199

## 0.2.0 - 2021-03-03

### Added
- Show measurement positions as overlay in brightfield image #131 #141
- Implement acquiring a scale calibration #124 #155
- Store scale calibration in acquisition file #124 #149
- Automatically load last scale calibration  #189 #190
- Store camera meta data in acquisition file #107 #138 #192
- Allow to create separate data files per repetition #169 #170
- Implement RLShutter and lamp control for ZeissECU #180 #186
- Adjust colormaps used #126 #140
- Ask for confirmation before closing #150 #151
- Show version in title bar #154
- Add mock camera to simplify debugging #158 #163 
- Add license
- Add readme

### Fixed
- Correctly handle binning in combination with repetitions #142 #144 #152 #153
- Correctly handle odd image sizes when binning #145 #146 #147
- Block the laser when in eyepiece mode #185 #188
- Correctly close handles to HDF5 objects #168
- Save camera data as correct type #157 #174
- Acquisition sometimes crashes during voltage calibration #127 #141
- Fix high CPU load after brightfield/fluorescence acquisition #171
- Correctly delete variables #176 #177
- Reduce CPU load for camera preview by using signals #178 #179 #187
- Correctly exit threads #172
- Code cleanup #148 #181 #182 #183 #184

## 0.1.0 - 2020-11-02

### Added
- Add support for binning on Andor and PVCam cameras
- Add changelog
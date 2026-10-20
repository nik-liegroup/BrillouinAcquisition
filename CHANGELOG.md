## Unreleased

### Fixed
- Overview images are centred on the centre of gravity of the active grid points (one tile or a mosaic), the image centre now being the
  planned point: the targets are shifted by the scanner offset, the pixel-to-um conversion of the image centre and the FOV offset of the
  measuring objective (the laser, not the image centre, sits on a target). With a surface scan the overview z starts at the median of the
  found surface, the planes spaced as the grid has them; the z stack of every overview objective follows the grid (own stack steps/range
  removed). The brightfield preset is set after the objective has turned. Drift limits: min. correlation default 0.6, max. speed removed.
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
- Overview tab: overviews with several objectives (each with its own exposure/gain, mosaic and z-stack settings, chosen by a dropdown of the
  calibrated objectives and an "active" tick; a summary lists the active ones). The software retracts z, turns the nosepiece, applies the
  calibrated FOV and focus offsets, takes the overview and returns. Overviews can additionally be taken before the measurement (after the surface
  scan) and after it (before the background spectra), and the trigger is selectable (after every z plane, once per repetition, not during).
- Drift correction (xy only, centre slice of a z stack): the overview of the chosen objective is registered against the previous accepted one with
  the same masked cross-correlation as the offline drift analysis; the grid, the overview tiles and the surface scan follow the measured drift
  (absolute and relative grid mode). A step with low correlation or implausible size/speed is not applied. Independent of the overview on/off.
- Surface scan: optional repeat during the measurement (with every overview or after every z plane): new medium reference and threshold from the
  same start position, remaining points move to the new surface, every round is saved.
  The previous scan is the prior: the offset between the scans found in the first columns moves the old surface of all other columns, so the search
  starts next to the new surface. Start is refused with a message (nothing created or saved) if the overview/drift/rescan settings cannot run.
- Continue: a Brillouin measurement that was cancelled or interrupted (crash, power loss) can be continued from the open file with the new
  Continue button next to Start (Start reads "Restart" while there is something to continue and starts a new repetition as before; Continue
  is greyed out before a measurement and after it finished). It restores ALL settings from the file (every Brillouin and camera setting is
  now written to the file at the start of a repetition), moves the stage back to the start position in relative mode, removes points whose
  write was cut short, and appends to the same repetition from the first point that is not in the file. If the measurement stopped during
  its surface scan, the scan continues with the columns that are done (checkpoint per column, same medium reference). Every continue is noted
  in the file (resume-event-N-*: point, time, cancelled or crash, last point date). Overview events, drift offset and rescan numbering go on.
- Brillouin camera: this version starts with exposure 0.2 s, binning 2x2, pixel readout rate 100 MHz and pre-amp gain option 2 (16-bit), and
  no longer writes the camera ROI, exposure and frame count to the settings file that other versions share (every measurement stores its own
  camera settings in its file). The scan order preference is saved (settings file and measurement file).
- The question about the surface scan of an earlier repetition defaults to Yes (start from it) when the grid is the same.
- Live previews are written on a thread of their own (the measurement only copies the frame).
- Tests: Continue is tested against real HDF5 files (crash and reopen, state, plan, cut-short points, surface scan checkpoint, earlier
  surface), the objective decision and the surface scan decision without files (ResumeFileTest, ResumeSettingsTest).
- Live check: the centre slice of every overview is also saved as a PNG (contrast stretched) in a folder next to the file (<file>_live) with
  events.csv (one line per overview event, surface rescan and continue: drift step, correlation, offsets, objective switch times). Switch
  "Live PNG previews" in the Overview tab; it switches itself off if the folder cannot be written.
- Continue: asks before the stage goes back to the saved start position (relative mode), turns the nosepiece back to the objective of the
  measurement (after retracting z) if the software stopped during an overview with another objective, and also replaces Start after the
  surface review (Start is greyed out then). An earlier surface is only offered for the same grid (extent, steps, bin, mode; for reuse also the
  same z planes).
- Surface scan of an earlier repetition: a new repetition asks (popup with a timer, default: new scan as always) whether to start the scan
  from the earlier one (the offset found in the first columns moves the old surface of the others: faster) or, if that repetition has its plan
  in the file, to reuse its surface without a new scan.
- Everything is saved in the file (settings, events, per-tile positions and offsets, drift decisions, surface scans of every round). Automatic
  objective switching is blocked without a z (focus) calibration; manual switching, the calibration cycles and measurements/overviews with a
  single objective are unchanged.
- Objective focus offset: the objective calibration now also stores this objective's focal-plane z relative to the reference
  objective (`hasFocusOffset`, `focusOffsetUm`, `focusOffsetSigmaUm`; optional attributes, older files load without them). It is measured in
  the same automated switch-and-refocus cycles as the FOV offset: the z at the end of the operator's refocus at the reference and at the
  target objective is recorded each cycle, the offset and its sigma accumulate over the cycles, are composed through the reference's own
  stored offset, and are saved with the FOV offset. `ScanControl::getFocusOffsetBetweenSlots()` gives the z to add when going from one
  objective's focus to another's. The reference objective counts as a validated focus offset of 0 (also in files written before this
  existed). `ScanControl::canSwitchObjectiveAutomatically()` is false unless both objectives have a scale calibration, an FOV offset and a
  focus offset; it is meant for every switch the software makes on its own (never for manual switching or the calibration cycles).
  Not yet used by any acquisition mode and not shown in the calibration dialog.
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
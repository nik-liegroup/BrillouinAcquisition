#include "stdafx.h"
#include "BrillouinAcquisition.h"
#include "helper/logger.h"

#include <algorithm>

#include <QComboBox>
#include <QDoubleSpinBox>
#include <QGridLayout>
#include <QGroupBox>
#include <QLabel>
#include <QScrollArea>
#include <QSettings>
#include <QSpinBox>
#include <QStandardItemModel>
#include <QTimer>
#include <QToolButton>
#include <QVBoxLayout>

/*
 * The Overview tab: which objectives take overviews (each with its own settings), when overviews are taken, drift correction, and the
 * repeat of the surface scan during a measurement (that one sits in the Surface scanning tab). The settings live in
 * BRILLOUIN_SETTINGS (see Brillouin_Overview.cpp for what they do). The overview of the objective that measures is the one the Fluorescence
 * tab's "BF overview" box has always described (BRILLOUIN_SETTINGS::saveOverviewBrightfieldPerZ and its neighbours); this tab reads and writes
 * those through the same widgets, so both always agree. The other objectives' overviews are entries of BRILLOUIN_SETTINGS::overviewObjectives.
 * When the nosepiece is turned (by hand) the two swap roles: the settings of the objective that was measuring are stored as its entry, and the
 * entry of the new one becomes the settings of the Fluorescence tab.
 */

namespace {

	QString objectiveLabel(const std::vector<std::string>& names, int slot) {
		if (slot < 0) {
			return "Current objective";
		}
		if ((size_t)slot >= 1 && (size_t)slot <= names.size() && !names[(size_t)slot - 1].empty()) {
			return QString("%1: %2").arg(slot).arg(QString::fromStdString(names[(size_t)slot - 1]));
		}
		return QString("Objective %1").arg(slot);
	}

	// The tab is only about 240 px wide: a combo box must not widen it to fit its longest item
	void shrinkableCombo(QComboBox* combo) {
		combo->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLengthWithIcon);
		combo->setMinimumContentsLength(8);
	}

	// A combo box item that cannot be chosen, with the reason as its tooltip
	void setItemEnabled(QComboBox* combo, int index, bool enabled, const QString& tip) {
		if (auto* model = qobject_cast<QStandardItemModel*>(combo->model())) {
			if (auto* item = model->item(index)) {
				item->setEnabled(enabled);
			}
		}
		combo->setItemData(index, tip, Qt::ToolTipRole);
	}

	// Sets a combo box to the item with the given data, if there is one
	bool selectByData(QComboBox* combo, int data) {
		for (int i = 0; i < combo->count(); i++) {
			if (combo->itemData(i).toInt() == data) {
				combo->setCurrentIndex(i);
				return true;
			}
		}
		return false;
	}

}

OVERVIEW_OBJECTIVE_SETTINGS* BrillouinAcquisition::findOverviewObjectiveEntry(int slot, bool create) {
	for (auto& entry : m_Brillouin->settings.overviewObjectives) {
		if (entry.slot == slot) {
			return &entry;
		}
	}
	if (!create) {
		return nullptr;
	}
	OVERVIEW_OBJECTIVE_SETTINGS entry;
	entry.slot = slot;
	m_Brillouin->settings.overviewObjectives.push_back(entry);
	return &m_Brillouin->settings.overviewObjectives.back();
}

void BrillouinAcquisition::setupOverviewTab() {
	if (m_overviewTabPage || !ui->acquisitionModeTabs) {
		return;
	}
	m_overviewTabPage = new QWidget(ui->acquisitionModeTabs);
	auto* pageLayout = new QVBoxLayout(m_overviewTabPage);
	pageLayout->setContentsMargins(0, 0, 0, 0);
	auto* scroll = new QScrollArea(m_overviewTabPage);
	scroll->setWidgetResizable(true);
	scroll->setFrameShape(QFrame::NoFrame);
	pageLayout->addWidget(scroll);
	auto* content = new QWidget();
	scroll->setWidget(content);
	auto* layout = new QVBoxLayout(content);
	layout->setContentsMargins(2, 4, 2, 4);
	layout->setSpacing(6);

	m_overviewSummaryLabel = new QLabel(content);
	m_overviewSummaryLabel->setWordWrap(true);
	m_overviewSummaryLabel->setToolTip("The objectives that take overviews in the next measurement, and when.");
	layout->addWidget(m_overviewSummaryLabel);

	// ---- objectives
	auto* objectiveGroup = new QGroupBox("Per objective", content);
	auto* objectiveGrid = new QGridLayout(objectiveGroup);
	objectiveGrid->setContentsMargins(4, 6, 4, 6);
	objectiveGrid->setHorizontalSpacing(4);
	objectiveGrid->setVerticalSpacing(2);
	m_overviewObjectiveCombo = new QComboBox(objectiveGroup);
	shrinkableCombo(m_overviewObjectiveCombo);
	m_overviewObjectiveCombo->setToolTip("Choose an objective to see and change its overview settings. Objectives that cannot be used greyed out: "
		"switching to another objective during a measurement needs its scale, FOV-center and z (focus) calibration.");
	objectiveGrid->addWidget(m_overviewObjectiveCombo, 0, 0, 1, 2);
	m_overviewObjectiveActiveCheckbox = new QCheckBox("Take overviews", objectiveGroup);
	objectiveGrid->addWidget(m_overviewObjectiveActiveCheckbox, 1, 0, 1, 2);
	objectiveGrid->addWidget(new QLabel("Exposure", objectiveGroup), 2, 0);
	m_overviewObjectiveExposureSpinBox = new QSpinBox(objectiveGroup);
	m_overviewObjectiveExposureSpinBox->setSuffix(" ms");
	m_overviewObjectiveExposureSpinBox->setRange(1, 10000);
	objectiveGrid->addWidget(m_overviewObjectiveExposureSpinBox, 2, 1);
	objectiveGrid->addWidget(new QLabel("Gain", objectiveGroup), 3, 0);
	m_overviewObjectiveGainSpinBox = new QDoubleSpinBox(objectiveGroup);
	m_overviewObjectiveGainSpinBox->setRange(0.0, 100.0);
	m_overviewObjectiveGainSpinBox->setDecimals(2);
	objectiveGrid->addWidget(m_overviewObjectiveGainSpinBox, 3, 1);
	m_overviewObjectiveMosaicCheckbox = new QCheckBox("Full grid (mosaic)", objectiveGroup);
	m_overviewObjectiveMosaicCheckbox->setToolTip("Tile camera-sized images (20 % overlap) over the whole grid instead of one image at the grid center.");
	objectiveGrid->addWidget(m_overviewObjectiveMosaicCheckbox, 4, 0, 1, 2);
	m_overviewObjectiveStackCheckbox = new QCheckBox("Z stack", objectiveGroup);
	m_overviewObjectiveStackCheckbox->setToolTip("A brightfield z stack instead of one image, over the z planes of the measurement grid (z min to z max, z steps; with surface following the range of the found surface). The centre slice is used for drift correction.");
	objectiveGrid->addWidget(m_overviewObjectiveStackCheckbox, 5, 0, 1, 2);
	// a brightfield image at the measured points: with the objective that measures (another one would turn the nosepiece at every point)
	m_overviewPerPointCheckbox = new QCheckBox("BF image at points", objectiveGroup);
	m_overviewPerPointCheckbox->setToolTip("A brightfield image at every N-th measured point, with the objective that measures. Not available for the other "
		"objectives: they would have to turn the nosepiece at every point.");
	objectiveGrid->addWidget(m_overviewPerPointCheckbox, 6, 0, 1, 2);
	objectiveGrid->addWidget(new QLabel("Every N points", objectiveGroup), 7, 0);
	m_overviewPerPointEverySpinBox = new QSpinBox(objectiveGroup);
	m_overviewPerPointEverySpinBox->setRange(1, 100000);
	objectiveGrid->addWidget(m_overviewPerPointEverySpinBox, 7, 1);
	m_overviewPerPointDuringCheckbox = new QCheckBox("During the exposure", objectiveGroup);
	m_overviewPerPointDuringCheckbox->setToolTip("Take the image while the Brillouin spectrum is exposed (no extra time), instead of after it.");
	objectiveGrid->addWidget(m_overviewPerPointDuringCheckbox, 8, 0, 1, 2);
	layout->addWidget(objectiveGroup);

	// ---- when
	auto* whenGroup = new QGroupBox("When (all objectives)", content);
	auto* whenGrid = new QGridLayout(whenGroup);
	whenGrid->setContentsMargins(4, 6, 4, 6);
	whenGrid->setHorizontalSpacing(4);
	whenGrid->setVerticalSpacing(2);
	m_overviewTriggerCombo = new QComboBox(whenGroup);
	shrinkableCombo(m_overviewTriggerCombo);
	m_overviewTriggerCombo->addItem("Every z plane", (int)OVERVIEW_TRIGGER::PER_PLANE);
	m_overviewTriggerCombo->addItem("Every repetition", (int)OVERVIEW_TRIGGER::PER_REPETITION);
	m_overviewTriggerCombo->addItem("Not during run", (int)OVERVIEW_TRIGGER::NONE);
	m_overviewTriggerCombo->setToolTip("When the overviews of all objectives that take them are taken during a measurement: after each finished z plane, "
		"once per repetition (after the last point), or not at all (then only the two overviews below).");
	whenGrid->addWidget(m_overviewTriggerCombo, 0, 0, 1, 2);
	m_overviewBeforeCheckbox = new QCheckBox("Also before start", whenGroup);
	m_overviewBeforeCheckbox->setToolTip("An overview before the first point, after the surface scan if there is one.");
	whenGrid->addWidget(m_overviewBeforeCheckbox, 1, 0, 1, 2);
	m_overviewAfterCheckbox = new QCheckBox("Also after end", whenGroup);
	m_overviewAfterCheckbox->setToolTip("An overview after the last point, before the background spectra. Not taken twice if one is due there anyway.");
	whenGrid->addWidget(m_overviewAfterCheckbox, 2, 0, 1, 2);
	whenGrid->addWidget(new QLabel("Z retract", whenGroup), 3, 0);
	m_overviewRetractSpinBox = new QDoubleSpinBox(whenGroup);
	m_overviewRetractSpinBox->setSuffix(" µm");
	m_overviewRetractSpinBox->setRange(-10000.0, 10000.0);
	m_overviewRetractSpinBox->setDecimals(1);
	m_overviewRetractSpinBox->setToolTip("Relative z move that takes the objective away from the sample before the nosepiece turns (sign as in the automated "
		"objective calibration). Needed when another objective takes overviews.");
	whenGrid->addWidget(m_overviewRetractSpinBox, 3, 1);
	layout->addWidget(whenGroup);

	// ---- drift correction
	auto* driftGroup = new QGroupBox("Drift correction", content);
	auto* driftGrid = new QGridLayout(driftGroup);
	driftGrid->setContentsMargins(4, 6, 4, 6);
	driftGrid->setHorizontalSpacing(4);
	driftGrid->setVerticalSpacing(2);
	m_driftEnabledCheckbox = new QCheckBox("Correct drift (xy)", driftGroup);
	m_driftEnabledCheckbox->setToolTip("Register each overview of the chosen objective against the previous accepted one; the grid, the overviews and the "
		"surface scan follow the measured drift. A step that is not believed leaves the positions where they are.");
	driftGrid->addWidget(m_driftEnabledCheckbox, 0, 0, 1, 2);
	m_driftObjectiveCombo = new QComboBox(driftGroup);
	shrinkableCombo(m_driftObjectiveCombo);
	m_driftObjectiveCombo->setToolTip("Only objectives that take overviews can be chosen. The centre slice of a z stack is used.");
	driftGrid->addWidget(m_driftObjectiveCombo, 1, 0, 1, 2);
	// the limits of what is believed (see the tooltips)
	auto* driftAdvanced = new QWidget(driftGroup);
	auto* advancedGrid = new QGridLayout(driftAdvanced);
	advancedGrid->setContentsMargins(0, 0, 0, 0);
	advancedGrid->setHorizontalSpacing(4);
	advancedGrid->setVerticalSpacing(2);
	driftGrid->addWidget(driftAdvanced, 2, 0, 1, 2);
	advancedGrid->addWidget(new QLabel("Min. corr.", driftAdvanced), 0, 0);
	m_driftMinNccSpinBox = new QDoubleSpinBox(driftAdvanced);
	m_driftMinNccSpinBox->setRange(0.05, 0.99);
	m_driftMinNccSpinBox->setSingleStep(0.05);
	m_driftMinNccSpinBox->setDecimals(2);
	m_driftMinNccSpinBox->setToolTip("How alike the two overviews have to be (normalised cross-correlation, 1 = identical) for the measured shift to be believed. Consecutive overviews of a stable sample give 0.9 and more, unrelated images up to about 0.2.");
	advancedGrid->addWidget(m_driftMinNccSpinBox, 0, 1);
	advancedGrid->addWidget(new QLabel("Max. step", driftAdvanced), 1, 0);
	m_driftMaxStepSpinBox = new QDoubleSpinBox(driftAdvanced);
	m_driftMaxStepSpinBox->setSuffix(" µm");
	m_driftMaxStepSpinBox->setRange(1.0, 1000.0);
	m_driftMaxStepSpinBox->setDecimals(0);
	m_driftMaxStepSpinBox->setToolTip("A shift larger than this between two overviews is not believed (a wrong match, not drift).");
	advancedGrid->addWidget(m_driftMaxStepSpinBox, 1, 1);
	m_driftExtrapolateCheckbox = new QCheckBox("Extrapolate drift between overviews", driftAdvanced);
	m_driftExtrapolateCheckbox->setToolTip("Between two overviews the grid keeps moving at the drift speed of the last accepted step (extrapolated for at most twice that interval). Off: the last measured offset is used unchanged until the next overview.");
	advancedGrid->addWidget(m_driftExtrapolateCheckbox, 2, 0, 1, 2);
	m_driftStatusLabel = new QLabel(driftGroup);
	m_driftStatusLabel->setWordWrap(true);
	driftGrid->addWidget(m_driftStatusLabel, 4, 0, 1, 2);
	layout->addWidget(driftGroup);
	m_livePreviewsCheckbox = new QCheckBox("Live PNG previews", content);
	m_livePreviewsCheckbox->setToolTip("Saves the centre slice of every overview as a PNG (contrast stretched) in a folder next to the file "
		"(<file>_live), with events.csv: one line per overview event, surface rescan and continue (drift step, correlation, offsets, objective "
		"switch times). To look at a running measurement without opening the file.");
	layout->addWidget(m_livePreviewsCheckbox);
	layout->addStretch(1);

	ui->acquisitionModeTabs->addTab(m_overviewTabPage, "Overview");
	// The overview and the brightfield image at the points of the objective that measures are set here now (the same settings, coupled):
	// the "BF overview" box of the Fluorescence tab is gone, its controls stay behind it as the ones this tab operates
	ui->bfOverviewGroupBox->hide();
	arrangeAcquisitionTabs();

	// ---- the repeat of the surface scan, in the Surface scanning tab below its group (the group has a fixed height of 380, the tab 456)
	if (ui->scrollAreaWidgetContents_4) {
		m_surfaceRescanGroup = new QGroupBox("Repeat during measurement", ui->scrollAreaWidgetContents_4);
		m_surfaceRescanGroup->setGeometry(8, 392, 248, 58);
		auto* rescanLayout = new QVBoxLayout(m_surfaceRescanGroup);
		rescanLayout->setContentsMargins(4, 6, 4, 4);
		m_surfaceRescanCombo = new QComboBox(m_surfaceRescanGroup);
		shrinkableCombo(m_surfaceRescanCombo);
		m_surfaceRescanCombo->addItem("Only at repetition start", (int)SURFACE_RESCAN::NEVER);
		m_surfaceRescanCombo->addItem("With every overview", (int)SURFACE_RESCAN::WITH_OVERVIEW);
		m_surfaceRescanCombo->addItem("After every z plane", (int)SURFACE_RESCAN::PER_PLANE);
		m_surfaceRescanCombo->setToolTip("Repeats the whole surface scan during the measurement (a new medium reference and threshold at the same start "
			"position, the remaining points move to the new surface; everything of every round is saved in the file). Never after the last point. "
			"With drift correction the columns follow the sample, the medium reference does not.");
		rescanLayout->addWidget(m_surfaceRescanCombo);
		m_surfaceRescanGroup->show();
	}

	// ---- behaviour
	connect(m_overviewObjectiveCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, [this](int index) {
		if (m_updatingOverviewTab || index < 0) {
			return;
		}
		m_overviewSelectedSlot = m_overviewObjectiveCombo->itemData(index).toInt();
		updateOverviewTab();
	});
	connect(m_overviewObjectiveActiveCheckbox, &QCheckBox::toggled, this, [this](bool checked) {
		if (m_updatingOverviewTab) {
			return;
		}
		if (m_overviewSelectedSlot == overviewMeasuringSlot()) {
			// the measuring objective: through the checkbox of the BF overview box, which also asks about a scan order without z outermost
			if (m_saveOverviewBrightfieldPerZCheckbox) {
				m_saveOverviewBrightfieldPerZCheckbox->setChecked(checked);
			}
		} else if (auto* entry = findOverviewObjectiveEntry(m_overviewSelectedSlot, true)) {
			entry->active = checked;
		}
		updateOverviewTab();
		updateEstimatedAcquisitionTime();
	});
	connect(m_overviewObjectiveExposureSpinBox, qOverload<int>(&QSpinBox::valueChanged), this, [this](int value) {
		if (m_updatingOverviewTab) {
			return;
		}
		if (m_overviewSelectedSlot == overviewMeasuringSlot()) {
			// the objective that measures: the same exposure as the BF exposure of the Fluorescence tab (one setting, changed in either place)
			m_Brillouin->settings.overviewBrightfieldExposureMs = value;
			if (ui->fluoBrightfieldExposure->value() != value) {
				ui->fluoBrightfieldExposure->setValue(value);
			}
		} else if (auto* entry = findOverviewObjectiveEntry(m_overviewSelectedSlot, true)) {
			entry->exposureMs = value;
		}
		// also while a measurement runs: the next overview is taken with it
		m_Brillouin->setLiveOverviewCamera(m_overviewSelectedSlot, value, m_overviewObjectiveGainSpinBox->value());
	});
	connect(m_overviewObjectiveGainSpinBox, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this](double value) {
		if (m_updatingOverviewTab) {
			return;
		}
		if (m_overviewSelectedSlot == overviewMeasuringSlot()) {
			m_Brillouin->settings.overviewBrightfieldGain = value;
			if (std::abs(ui->fluoBrightfieldGain->value() - value) > 1e-9) {
				ui->fluoBrightfieldGain->setValue(value);
			}
		} else if (auto* entry = findOverviewObjectiveEntry(m_overviewSelectedSlot, true)) {
			entry->gain = value;
		}
		m_Brillouin->setLiveOverviewCamera(m_overviewSelectedSlot, m_overviewObjectiveExposureSpinBox->value(), value);
	});
	connect(m_overviewObjectiveMosaicCheckbox, &QCheckBox::toggled, this, [this](bool checked) {
		if (m_updatingOverviewTab) {
			return;
		}
		if (m_overviewSelectedSlot == overviewMeasuringSlot()) {
			if (checked && m_overviewFullGridRadio) {
				m_overviewFullGridRadio->setChecked(true);
			} else if (!checked && m_overviewSingleImageRadio) {
				m_overviewSingleImageRadio->setChecked(true);
			}
		} else if (auto* entry = findOverviewObjectiveEntry(m_overviewSelectedSlot, true)) {
			entry->fullGrid = checked;
		}
		updateOverviewTab();
	});
	connect(m_overviewObjectiveStackCheckbox, &QCheckBox::toggled, this, [this](bool checked) {
		if (m_updatingOverviewTab) {
			return;
		}
		if (m_overviewSelectedSlot == overviewMeasuringSlot()) {
			if (m_overviewFullStackCheckbox) {
				m_overviewFullStackCheckbox->setChecked(checked);
			}
		} else if (auto* entry = findOverviewObjectiveEntry(m_overviewSelectedSlot, true)) {
			entry->fullStack = checked;
		}
		updateOverviewTab();
	});
	connect(m_overviewPerPointCheckbox, &QCheckBox::toggled, this, [this](bool checked) {
		if (!m_updatingOverviewTab && m_capturePerPointBrightfieldCheckbox) {
			m_capturePerPointBrightfieldCheckbox->setChecked(checked);
		}
		updateOverviewTab();
	});
	connect(m_overviewPerPointEverySpinBox, qOverload<int>(&QSpinBox::valueChanged), this, [this](int value) {
		if (!m_updatingOverviewTab && m_perPointBrightfieldEveryNSpinBox) {
			m_perPointBrightfieldEveryNSpinBox->setValue(value);
		}
	});
	connect(m_overviewPerPointDuringCheckbox, &QCheckBox::toggled, this, [this](bool checked) {
		if (!m_updatingOverviewTab && m_perPointBrightfieldDuringAcquisitionCheckbox) {
			m_perPointBrightfieldDuringAcquisitionCheckbox->setChecked(checked);
		}
	});
	connect(m_overviewTriggerCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, [this](int index) {
		if (m_updatingOverviewTab || index < 0) {
			return;
		}
		m_Brillouin->settings.overviewTrigger = (OVERVIEW_TRIGGER)m_overviewTriggerCombo->itemData(index).toInt();
		updateOverviewTab();
		updateEstimatedAcquisitionTime();
	});
	connect(m_overviewBeforeCheckbox, &QCheckBox::toggled, this, [this](bool checked) {
		if (m_updatingOverviewTab) {
			return;
		}
		m_Brillouin->settings.overviewBeforeMeasurement = checked;
		updateOverviewTab();
		updateEstimatedAcquisitionTime();
	});
	connect(m_overviewAfterCheckbox, &QCheckBox::toggled, this, [this](bool checked) {
		if (m_updatingOverviewTab) {
			return;
		}
		m_Brillouin->settings.overviewAfterMeasurement = checked;
		updateOverviewTab();
		updateEstimatedAcquisitionTime();
	});
	connect(m_overviewRetractSpinBox, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this](double value) {
		if (m_updatingOverviewTab) {
			return;
		}
		m_Brillouin->settings.overviewObjectiveRetractUm = value;
	});
	connect(m_driftEnabledCheckbox, &QCheckBox::toggled, this, [this](bool checked) {
		if (m_updatingOverviewTab) {
			return;
		}
		m_Brillouin->settings.driftCorrectionEnabled = checked;
		if (checked && m_driftObjectiveCombo->count() > 0 && m_driftObjectiveCombo->currentIndex() >= 0) {
			m_Brillouin->settings.driftObjectiveSlot = m_driftObjectiveCombo->currentData().toInt();
		}
		updateOverviewTab();
	});
	connect(m_driftObjectiveCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, [this](int index) {
		if (m_updatingOverviewTab || index < 0) {
			return;
		}
		m_Brillouin->settings.driftObjectiveSlot = m_driftObjectiveCombo->itemData(index).toInt();
		updateOverviewTab();
	});
	connect(m_driftMinNccSpinBox, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this](double value) {
		if (!m_updatingOverviewTab) {
			m_Brillouin->settings.driftMinNcc = value;
		}
	});
	connect(m_driftMaxStepSpinBox, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this](double value) {
		if (!m_updatingOverviewTab) {
			m_Brillouin->settings.driftMaxStepUm = value;
		}
	});
	connect(m_livePreviewsCheckbox, &QCheckBox::toggled, this, [this](bool checked) {
		if (!m_updatingOverviewTab) {
			m_Brillouin->settings.livePreviews = checked;
		}
	});
	connect(m_driftExtrapolateCheckbox, &QCheckBox::toggled, this, [this](bool checked) {
		if (!m_updatingOverviewTab) {
			m_Brillouin->settings.driftExtrapolate = checked;
		}
	});
	if (m_surfaceRescanCombo) {
		connect(m_surfaceRescanCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, [this](int index) {
			if (m_updatingOverviewTab || index < 0) {
				return;
			}
			m_Brillouin->settings.surfaceRescan = (SURFACE_RESCAN)m_surfaceRescanCombo->itemData(index).toInt();
			updateOverviewTab();
		});
	}
	updateOverviewTab();
}

int BrillouinAcquisition::overviewMeasuringSlot() const {
	const auto running = m_Brillouin ? m_Brillouin->runningMeasurementObjectiveSlot() : -1;
	if (running >= 0) {
		return running;
	}
	return m_scanControl ? m_scanControl->getActiveObjectiveSlot() : -1;
}

void BrillouinAcquisition::arrangeAcquisitionTabs() {
	auto* tabs = ui->acquisitionModeTabs;
	if (!tabs) {
		return;
	}
	const QStringList order{ "Grid", "Fluorescence", "Overview", "Surface scanning", "ODT" };
	auto target = 0;
	for (const auto& name : order) {
		for (int i = 0; i < tabs->count(); i++) {
			if (tabs->tabText(i) == name) {
				if (i != target) {
					tabs->tabBar()->moveTab(i, target);
				}
				target++;
				break;
			}
		}
	}
}

void BrillouinAcquisition::scheduleOverviewTabUpdate() {
	if (m_overviewTabUpdatePending) {
		return;
	}
	m_overviewTabUpdatePending = true;
	QTimer::singleShot(30, this, [this]() {
		m_overviewTabUpdatePending = false;
		updateOverviewTab();
	});
}

void BrillouinAcquisition::updateOverviewTab() {
	if (!m_overviewTabPage || !m_Brillouin) {
		return;
	}
	struct SlowLog {
		QElapsedTimer timer;
		SlowLog() { timer.start(); }
		~SlowLog() {
			if (timer.elapsed() > 30) {
				qWarning(logWarning()) << "updateOverviewTab took" << timer.elapsed() << "ms";
			}
		}
	} slowLog;
	struct Updating {
		bool& flag;
		explicit Updating(bool& f) : flag(f) { flag = true; }
		~Updating() { flag = false; }
	} updating{ m_updatingOverviewTab };

	auto& settings = m_Brillouin->settings;
	const auto activeSlot = overviewMeasuringSlot();
	const auto cameraConnected = m_brightfieldCamera != nullptr && m_brightfieldCamera->getConnectionStatus();
	// the same condition the BF overview box of the Fluorescence tab uses
	const auto overviewPossible = m_hasFluorescence && cameraConnected;

	// ---- the objectives that can be chosen
	{
		std::vector<int> objectiveSlots;
		if (m_scanControl) {
			for (size_t i = 0; i < m_objectiveSlotNames.size(); i++) {
				const auto slot = (int)i + 1;
				if (slot == activeSlot || m_scanControl->hasObjectiveCalibration(slot)) {
					objectiveSlots.push_back(slot);
				}
			}
		}
		if (objectiveSlots.empty() || std::find(objectiveSlots.begin(), objectiveSlots.end(), activeSlot) == objectiveSlots.end()) {
			objectiveSlots.insert(objectiveSlots.begin(), activeSlot);
		}
		m_overviewObjectiveCombo->clear();
		for (const auto slot : objectiveSlots) {
			auto text = objectiveLabel(m_objectiveSlotNames, slot);
			if (slot == activeSlot) {
				text += " (measures)";
			}
			m_overviewObjectiveCombo->addItem(text, slot);
			const auto index = m_overviewObjectiveCombo->count() - 1;
			if (slot == activeSlot) {
				setItemEnabled(m_overviewObjectiveCombo, index, true, "The objective that measures: the one in the beam when the measurement starts.");
			} else {
				std::string reason;
				const auto usable = m_scanControl && m_scanControl->hasObjectiveChanger()
					&& m_scanControl->canSwitchObjectiveAutomatically(activeSlot, slot, &reason);
				setItemEnabled(m_overviewObjectiveCombo, index, usable,
					usable ? "Overviews with this objective: the software turns the nosepiece, retracts z and refocuses on its own."
						: QString::fromStdString(m_scanControl && m_scanControl->hasObjectiveChanger() ? reason : "No motorized objective changer."));
			}
		}
		if (!selectByData(m_overviewObjectiveCombo, m_overviewSelectedSlot)) {
			m_overviewSelectedSlot = activeSlot;
			selectByData(m_overviewObjectiveCombo, activeSlot);
		}
		const auto* model = qobject_cast<QStandardItemModel*>(m_overviewObjectiveCombo->model());
		if (model && model->item(m_overviewObjectiveCombo->currentIndex()) && !model->item(m_overviewObjectiveCombo->currentIndex())->isEnabled()) {
			m_overviewSelectedSlot = activeSlot;
			selectByData(m_overviewObjectiveCombo, activeSlot);
		}
	}

	// ---- the settings of the chosen objective
	{
		const auto measuring = m_overviewSelectedSlot == activeSlot;
		const auto* entry = measuring ? nullptr : findOverviewObjectiveEntry(m_overviewSelectedSlot, false);
		OVERVIEW_OBJECTIVE_SETTINGS shown;
		if (measuring) {
			shown.slot = activeSlot;
			shown.active = settings.saveOverviewBrightfieldPerZ;
			shown.exposureMs = settings.overviewBrightfieldExposureMs;
			shown.gain = settings.overviewBrightfieldGain;
			shown.fullGrid = settings.overviewBrightfieldFullGrid;
			shown.fullStack = settings.overviewBrightfieldFullGrid ? settings.overviewBrightfieldFullStackMosaic : settings.overviewBrightfieldFullStackSingle;
		} else if (entry) {
			shown = *entry;
		} else {
			shown.slot = m_overviewSelectedSlot;
		}
		m_overviewObjectiveActiveCheckbox->setChecked(shown.active);
		m_overviewObjectiveActiveCheckbox->setEnabled(overviewPossible);
		m_overviewObjectiveActiveCheckbox->setToolTip(overviewPossible ? "" : "Needs the brightfield camera (and the Fluorescence hardware).");
		m_overviewObjectiveExposureSpinBox->setValue(shown.exposureMs);
		m_overviewObjectiveGainSpinBox->setValue(shown.gain);
		// the exposure and gain of the measuring objective's overview are those of the brightfield camera panel
		// the measuring objective's are the BF exposure and gain of the Fluorescence tab (coupled); all can be changed during a measurement
		m_overviewObjectiveExposureSpinBox->setEnabled(overviewPossible && (shown.active || measuring));
		m_overviewObjectiveGainSpinBox->setEnabled(overviewPossible && (shown.active || measuring));
		m_overviewObjectiveExposureSpinBox->setToolTip(measuring ? "The same setting as the BF exposure of the Fluorescence tab. Can be changed during a measurement." : "Can be changed during a measurement.");
		m_overviewObjectiveGainSpinBox->setToolTip(measuring ? "The same setting as the BF gain of the Fluorescence tab. Can be changed during a measurement." : "Can be changed during a measurement.");
		m_overviewObjectiveMosaicCheckbox->setChecked(shown.fullGrid);
		m_overviewObjectiveMosaicCheckbox->setEnabled(overviewPossible && shown.active);
		m_overviewObjectiveStackCheckbox->setChecked(shown.fullStack);
		m_overviewObjectiveStackCheckbox->setEnabled(overviewPossible && shown.active);
		// brightfield images at the points: only the objective that measures has them
		const auto perPointOn = settings.capturePerPointBrightfield;
		m_overviewPerPointCheckbox->setChecked(measuring && perPointOn);
		m_overviewPerPointCheckbox->setEnabled(overviewPossible && measuring);
		m_overviewPerPointEverySpinBox->setValue(settings.perPointBrightfieldEveryN);
		m_overviewPerPointEverySpinBox->setEnabled(overviewPossible && measuring && perPointOn);
		m_overviewPerPointDuringCheckbox->setChecked(settings.perPointBrightfieldDuringAcquisition);
		m_overviewPerPointDuringCheckbox->setEnabled(overviewPossible && measuring && perPointOn);
	}

	// ---- when
	{
		selectByData(m_overviewTriggerCombo, (int)settings.overviewTrigger);
		m_overviewTriggerCombo->setEnabled(overviewPossible);
		m_overviewBeforeCheckbox->setChecked(settings.overviewBeforeMeasurement);
		m_overviewBeforeCheckbox->setEnabled(overviewPossible);
		m_overviewAfterCheckbox->setChecked(settings.overviewAfterMeasurement);
		m_overviewAfterCheckbox->setEnabled(overviewPossible);
		m_overviewRetractSpinBox->setValue(settings.overviewObjectiveRetractUm);
		auto otherActive = false;
		for (const auto& entry : settings.overviewObjectives) {
			otherActive = otherActive || (entry.active && entry.slot != activeSlot);
		}
		m_overviewRetractSpinBox->setEnabled(overviewPossible && otherActive);
	}

	// ---- drift correction: only objectives that take overviews can be chosen
	{
		std::vector<int> choices;
		if (settings.saveOverviewBrightfieldPerZ) {
			choices.push_back(activeSlot);
		}
		for (const auto& entry : settings.overviewObjectives) {
			if (entry.active && entry.slot != activeSlot) {
				choices.push_back(entry.slot);
			}
		}
		m_driftObjectiveCombo->clear();
		for (const auto slot : choices) {
			m_driftObjectiveCombo->addItem(objectiveLabel(m_objectiveSlotNames, slot) + (slot == activeSlot ? " (measures)" : ""), slot);
		}
		if (!choices.empty() && !selectByData(m_driftObjectiveCombo, settings.driftObjectiveSlot)) {
			m_driftObjectiveCombo->setCurrentIndex(0);
			settings.driftObjectiveSlot = m_driftObjectiveCombo->itemData(0).toInt();
		}
		const auto driftPossible = overviewPossible && !choices.empty();
		if (!driftPossible && settings.driftCorrectionEnabled) {
			// never while a measurement runs (it works with what it started with); and always said, so a switch that happens by itself is seen
			if (m_enabledModes == ACQUISITION_MODE::NONE) {
				qWarning(logWarning()) << "Overview tab: drift correction switched off - no objective takes overviews (overview possible:"
					<< overviewPossible << ", choices:" << (int)choices.size() << ").";
				settings.driftCorrectionEnabled = false;
			} else {
				qWarning(logWarning()) << "Overview tab: no objective takes overviews at the moment, drift correction stays on while the measurement runs.";
			}
		}
		m_driftEnabledCheckbox->setChecked(settings.driftCorrectionEnabled);
		m_driftEnabledCheckbox->setEnabled(driftPossible);
		m_driftObjectiveCombo->setEnabled(driftPossible && settings.driftCorrectionEnabled);
		m_driftMinNccSpinBox->setValue(settings.driftMinNcc);
		m_driftMaxStepSpinBox->setValue(settings.driftMaxStepUm);
		m_driftExtrapolateCheckbox->setChecked(settings.driftExtrapolate);
		for (QWidget* widget : std::initializer_list<QWidget*>{ m_driftMinNccSpinBox, m_driftMaxStepSpinBox, m_driftExtrapolateCheckbox }) {
			widget->setEnabled(driftPossible && settings.driftCorrectionEnabled);
		}
		m_driftStatusLabel->setText(choices.empty()
			? "Needs an objective that takes overviews (activate one above)."
			: (settings.driftCorrectionEnabled ? "Needs overviews during the measurement: after every z plane is best (drift is only known at overviews)." : ""));
	}

	m_livePreviewsCheckbox->setChecked(settings.livePreviews);

	// ---- the repeat of the surface scan
	if (m_surfaceRescanCombo) {
		selectByData(m_surfaceRescanCombo, (int)settings.surfaceRescan);
		m_surfaceRescanCombo->setEnabled(settings.useSurfaceFollow);
		m_surfaceRescanGroup->setToolTip(settings.useSurfaceFollow ? "" : "Needs surface scanning to be on.");
	}

	// ---- summary
	{
		QStringList active;
		if (settings.saveOverviewBrightfieldPerZ) {
			active << objectiveLabel(m_objectiveSlotNames, activeSlot) + " (measures)";
		}
		for (const auto& entry : settings.overviewObjectives) {
			if (entry.active && entry.slot != activeSlot) {
				active << objectiveLabel(m_objectiveSlotNames, entry.slot);
			}
		}
		QString when;
		switch (settings.overviewTrigger) {
			case OVERVIEW_TRIGGER::PER_PLANE:
				when = "after every z plane";
				break;
			case OVERVIEW_TRIGGER::PER_REPETITION:
				when = "once per repetition";
				break;
			case OVERVIEW_TRIGGER::NONE:
				when = "not during the measurement";
				break;
		}
		if (settings.overviewBeforeMeasurement) {
			when += ", before";
		}
		if (settings.overviewAfterMeasurement) {
			when += ", after";
		}
		auto text = active.isEmpty() ? QString("Overviews: none") : QString("Overviews: %1 (%2)").arg(active.join(", ")).arg(when);
		if (settings.driftCorrectionEnabled) {
			text += QString("\nDrift correction: objective %1").arg(settings.driftObjectiveSlot);
		}
		m_overviewSummaryLabel->setText(text);
	}
}

void BrillouinAcquisition::syncOverviewObjectiveSettingsOnSwitch(int previousSlot, int newSlot) {
	if (!m_Brillouin || previousSlot == newSlot) {
		return;
	}
	auto& settings = m_Brillouin->settings;
	// The Fluorescence tab's BF overview settings belong to the objective that measures. Store them as the entry of the one that leaves
	// and take over the entry of the one that arrives (if it has one: otherwise the settings stay as they are, the new objective inherits).
	const auto legacyAsEntry = [&settings](int slot) {
		OVERVIEW_OBJECTIVE_SETTINGS entry;
		entry.slot = slot;
		entry.active = settings.saveOverviewBrightfieldPerZ;
		entry.exposureMs = settings.overviewBrightfieldExposureMs;
		entry.gain = settings.overviewBrightfieldGain;
		entry.fullGrid = settings.overviewBrightfieldFullGrid;
		entry.fullStack = settings.overviewBrightfieldFullGrid ? settings.overviewBrightfieldFullStackMosaic : settings.overviewBrightfieldFullStackSingle;
		return entry;
	};
	if (previousSlot >= 0) {
		const auto current = legacyAsEntry(previousSlot);
		// An objective that was only passed through (a calibration cycle, a quick look) and whose settings were inherited and never touched
		// has not been set up: it gets no entry, so it cannot turn into an overview objective by accident.
		const auto& snapshot = m_overviewInheritedSnapshot;
		const auto untouched = m_overviewInheritedSlot == previousSlot && snapshot.active == current.active && snapshot.exposureMs == current.exposureMs
			&& snapshot.gain == current.gain && snapshot.fullGrid == current.fullGrid && snapshot.fullStack == current.fullStack;
		if (!untouched) {
			if (auto* entry = findOverviewObjectiveEntry(previousSlot, true)) {
				entry->active = current.active;
				entry->exposureMs = current.exposureMs;
				entry->gain = current.gain;
				entry->fullGrid = current.fullGrid;
				entry->fullStack = current.fullStack;
			}
		}
	}
	if (const auto* entry = findOverviewObjectiveEntry(newSlot, false)) {
		settings.saveOverviewBrightfieldPerZ = entry->active;
		settings.overviewBrightfieldExposureMs = entry->exposureMs;
		settings.overviewBrightfieldGain = entry->gain;
		settings.overviewBrightfieldFullGrid = entry->fullGrid;
		if (entry->fullGrid) {
			settings.overviewBrightfieldFullStackMosaic = entry->fullStack;
		} else {
			settings.overviewBrightfieldFullStackSingle = entry->fullStack;
		}
		m_overviewInheritedSlot = -1;
	} else {
		// no entry: the settings stay as they are (as they always did when the nosepiece was turned) and are remembered as inherited
		m_overviewInheritedSlot = newSlot;
		m_overviewInheritedSnapshot = legacyAsEntry(newSlot);
	}
	m_overviewSelectedSlot = newSlot;
	updateBrillouinSettings();
}

void BrillouinAcquisition::writeOverviewSettings(QSettings& settings) {
	const auto& s = m_Brillouin->settings;
	settings.setValue("brillouin-overview-trigger", (int)s.overviewTrigger);
	settings.setValue("brillouin-overview-before-measurement", s.overviewBeforeMeasurement);
	settings.setValue("brillouin-overview-after-measurement", s.overviewAfterMeasurement);
	settings.setValue("brillouin-overview-objective-retract-um", s.overviewObjectiveRetractUm);
	settings.setValue("brillouin-drift-correction-enabled", s.driftCorrectionEnabled);
	settings.setValue("brillouin-drift-objective-slot", s.driftObjectiveSlot);
	settings.setValue("brillouin-drift-min-ncc", s.driftMinNcc);
	settings.setValue("brillouin-drift-max-step-um", s.driftMaxStepUm);
	settings.setValue("brillouin-drift-extrapolate", s.driftExtrapolate);
	settings.setValue("brillouin-surface-rescan", (int)s.surfaceRescan);
	settings.setValue("brillouin-live-previews", s.livePreviews);
	const auto order = m_Brillouin->scanOrder();
	settings.setValue("brillouin-scan-order-automatic", order.automatical);
	settings.setValue("brillouin-scan-order-x", order.x);
	settings.setValue("brillouin-scan-order-y", order.y);
	settings.setValue("brillouin-scan-order-z", order.z);
	settings.setValue("brillouin-overview-objectives-count", (int)s.overviewObjectives.size());
	for (size_t i = 0; i < s.overviewObjectives.size(); i++) {
		const auto& entry = s.overviewObjectives[i];
		const auto key = QString("brillouin-overview-objective-%1-").arg(i);
		settings.setValue(key + "slot", entry.slot);
		settings.setValue(key + "active", entry.active);
		settings.setValue(key + "exposure-ms", entry.exposureMs);
		settings.setValue(key + "gain", entry.gain);
		settings.setValue(key + "full-grid", entry.fullGrid);
		settings.setValue(key + "full-stack", entry.fullStack);
	}
}

void BrillouinAcquisition::readOverviewSettings(QSettings& settings) {
	auto& s = m_Brillouin->settings;
	s.overviewTrigger = (OVERVIEW_TRIGGER)std::clamp(settings.value("brillouin-overview-trigger", (int)s.overviewTrigger).toInt(), 0, 2);
	s.overviewBeforeMeasurement = settings.value("brillouin-overview-before-measurement", s.overviewBeforeMeasurement).toBool();
	s.overviewAfterMeasurement = settings.value("brillouin-overview-after-measurement", s.overviewAfterMeasurement).toBool();
	s.overviewObjectiveRetractUm = settings.value("brillouin-overview-objective-retract-um", s.overviewObjectiveRetractUm).toDouble();
	s.driftCorrectionEnabled = settings.value("brillouin-drift-correction-enabled", s.driftCorrectionEnabled).toBool();
	s.driftObjectiveSlot = settings.value("brillouin-drift-objective-slot", s.driftObjectiveSlot).toInt();
	s.driftMinNcc = settings.value("brillouin-drift-min-ncc", s.driftMinNcc).toDouble();
	s.driftMaxStepUm = settings.value("brillouin-drift-max-step-um", s.driftMaxStepUm).toDouble();
	s.driftExtrapolate = settings.value("brillouin-drift-extrapolate", s.driftExtrapolate).toBool();
	s.surfaceRescan = (SURFACE_RESCAN)std::clamp(settings.value("brillouin-surface-rescan", (int)s.surfaceRescan).toInt(), 0, 2);
	s.livePreviews = settings.value("brillouin-live-previews", s.livePreviews).toBool();
	if (settings.contains("brillouin-scan-order-x")) {
		SCAN_ORDER order = m_Brillouin->scanOrder();
		order.automatical = settings.value("brillouin-scan-order-automatic", order.automatical).toBool();
		order.x = settings.value("brillouin-scan-order-x", order.x).toInt();
		order.y = settings.value("brillouin-scan-order-y", order.y).toInt();
		order.z = settings.value("brillouin-scan-order-z", order.z).toInt();
		m_Brillouin->setScanOrder(order);
	}
	const auto count = std::clamp(settings.value("brillouin-overview-objectives-count", 0).toInt(), 0, 32);
	s.overviewObjectives.clear();
	for (int i = 0; i < count; i++) {
		const auto key = QString("brillouin-overview-objective-%1-").arg(i);
		OVERVIEW_OBJECTIVE_SETTINGS entry;
		entry.slot = settings.value(key + "slot", -1).toInt();
		entry.active = settings.value(key + "active", false).toBool();
		entry.exposureMs = std::max(1, settings.value(key + "exposure-ms", entry.exposureMs).toInt());
		entry.gain = settings.value(key + "gain", entry.gain).toDouble();
		entry.fullGrid = settings.value(key + "full-grid", false).toBool();
		entry.fullStack = settings.value(key + "full-stack", false).toBool();
		s.overviewObjectives.push_back(entry);
	}
}

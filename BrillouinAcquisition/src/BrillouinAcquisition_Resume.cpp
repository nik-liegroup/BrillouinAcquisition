#include "stdafx.h"
#include "BrillouinAcquisition.h"
#include "Acquisition/AcquisitionModes/BrillouinResume.h"

#include <QBoxLayout>
#include <QDialog>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>

/*
 * Continue of an interrupted Brillouin measurement (crash, cancel): the Continue button next to Start, the "Restart" label of Start while there
 * is something to continue, and the question about the surface scan of an earlier repetition. What a continue does, and what is in the file for
 * it, is described in Brillouin_Resume.cpp.
 */

void BrillouinAcquisition::setupContinueButton() {
	if (m_brillouinContinueButton || !ui->BrillouinStart) {
		return;
	}
	m_brillouinContinueButton = new QPushButton("Continue", ui->BrillouinStart->parentWidget());
	m_brillouinContinueButton->setMinimumSize(ui->BrillouinStart->minimumSize());
	m_brillouinContinueButton->setEnabled(false);
	m_brillouinContinueButton->setToolTip("Continue the last Brillouin measurement of the open file where it stopped (cancelled, or the software "
		"stopped). Not available before a measurement was started, and not after it finished.");
	if (auto* box = qobject_cast<QBoxLayout*>(ui->BrillouinStart->parentWidget()->layout())) {
		box->insertWidget(box->indexOf(ui->BrillouinStart) + 1, m_brillouinContinueButton);
	}
	connect(m_brillouinContinueButton, &QPushButton::clicked, this, [this]() { onBrillouinContinueClicked(); });
}

void BrillouinAcquisition::updateContinueAvailability() {
	if (!m_brillouinContinueButton || !m_Brillouin || !m_acquisition) {
		return;
	}
	const auto status = m_Brillouin->getStatus();
	// Start as it always is (ODT), the review of the surface scan below takes it over
	updateBrillouinStartAvailability();
	if (status == ACQUISITION_STATUS::WAITFORSURFACEREVIEW) {
		// the one Continue button also continues after the review of the surface scan (it happens by itself when the timer runs out)
		ui->BrillouinStart->setEnabled(false);
		ui->BrillouinStart->setText("Start");
		m_brillouinContinueButton->setEnabled(true);
		m_brillouinContinueButton->setToolTip("Continue with the measurement after the surface scan. Happens by itself when the timer runs out.");
		return;
	}
	m_brillouinContinueButton->setToolTip("Continue the last Brillouin measurement of the open file where it stopped (cancelled, or the software "
		"stopped). Not available before a measurement was started, and not after it finished.");
	const auto running = status == ACQUISITION_STATUS::WAITFORREPETITION || status == ACQUISITION_STATUS::STARTED
		|| status == ACQUISITION_STATUS::RUNNING || status == ACQUISITION_STATUS::WAITFORSURFACEREVIEW || status == ACQUISITION_STATUS::ALIGNING;
	auto* storage = m_acquisition->m_storage.get();
	if (storage && !running && m_enabledModes == ACQUISITION_MODE::NONE) {
		// Only what the file says about its last repetition (two small reads), the details are read when Continue is pressed. Asked on the
		// storage thread without waiting for it, so the interface never stands still for the file: the answer comes back as a queued call.
		QMetaObject::invokeMethod(storage, [this, storage]() {
			const auto repetition = storage->lastRepetition(ACQUISITION_MODE::BRILLOUIN);
			const auto state = repetition < 0 ? std::string{} : storage->getRepetitionState(ACQUISITION_MODE::BRILLOUIN, repetition);
			QMetaObject::invokeMethod(this, [this, state]() { applyContinueAvailability(state, false); }, Qt::QueuedConnection);
		}, Qt::QueuedConnection);
		return;
	}
	applyContinueAvailability(std::string{}, running);
}

void BrillouinAcquisition::applyContinueAvailability(const std::string& state, bool running) {
	// a measurement started in the meantime: nothing to show
	const auto status = m_Brillouin->getStatus();
	if (status == ACQUISITION_STATUS::WAITFORREPETITION || status == ACQUISITION_STATUS::STARTED || status == ACQUISITION_STATUS::RUNNING
		|| status == ACQUISITION_STATUS::WAITFORSURFACEREVIEW || status == ACQUISITION_STATUS::ALIGNING || m_enabledModes != ACQUISITION_MODE::NONE) {
		running = true;
	}
	const auto resumable = !running && (state == "measuring" || state == "stopped" || state == "surface-scan" || state == "surface-scan-stopped");
	m_brillouinContinueButton->setEnabled(resumable);
	if (resumable) {
		// Start makes a new repetition in the same file, as it always did; Continue appends to the one that was interrupted
		ui->BrillouinStart->setText("Restart");
		ui->BrillouinStart->setToolTip("Start a new repetition (the interrupted one stays in the file as it is). Continue picks the interrupted one up.");
	} else if (!running && ui->BrillouinStart->text() == "Restart") {
		ui->BrillouinStart->setText("Start");
		ui->BrillouinStart->setToolTip(QString{});
	}
}

void BrillouinAcquisition::applyRestoredBrillouinSettings(const BRILLOUIN_SETTINGS& restored) {
	m_Brillouin->settings = restored;
	const auto& camera = restored.camera;
	// the camera of the Brillouin measurement: the widgets (whose handlers write into the settings and push them to the camera) and then the
	// whole camera settings again, including what has no widget
	ui->exposureTime->setValue(camera.exposureTime);
	ui->frameCount->setValue((int)camera.frameCount);
	ui->binning->setCurrentText(QString::fromStdWString(camera.roi.binning));
	ui->pixelReadoutRate->setCurrentText(QString::fromStdWString(camera.readout.pixelReadoutRate));
	ui->preAmpGain->setCurrentText(QString::fromStdWString(camera.readout.preAmpGain));
	ui->pixelEncoding->setCurrentText(QString::fromStdWString(camera.readout.pixelEncoding));
	ui->cycleMode->setCurrentText(QString::fromStdWString(camera.readout.cycleMode));
	ui->ROILeft->setValue((int)camera.roi.left);
	ui->ROIWidth->setValue((int)camera.roi.width_physical);
	ui->ROITop->setValue((int)camera.roi.top);
	ui->ROIHeight->setValue((int)camera.roi.height_physical);
	m_Brillouin->settings.camera = camera;
	applyCameraSettings();
	updateBrillouinSettings();
	updateEstimatedAcquisitionTime();
}

void BrillouinAcquisition::onBrillouinContinueClicked() {
	if (m_Brillouin->getStatus() == ACQUISITION_STATUS::WAITFORSURFACEREVIEW) {
		// after the surface scan: measure the points that have a surface (Full grid measures all of them)
		QMetaObject::invokeMethod(
			m_Brillouin,
			[&m_Brillouin = m_Brillouin]() { m_Brillouin->continueAfterSurfaceReview(false); },
			Qt::AutoConnection
		);
		return;
	}
	auto* storage = m_acquisition ? m_acquisition->m_storage.get() : nullptr;
	if (!storage) {
		return;
	}
	Brillouin::ResumeInfo info;
	std::string problem;
	auto readable = false;
	QMetaObject::invokeMethod(storage, [&]() { readable = Brillouin::readResumeInfo(storage, &info, &problem); }, Qt::BlockingQueuedConnection);
	if (!readable) {
		QMessageBox::warning(this, "Cannot continue", QString::fromStdString(problem));
		updateContinueAvailability();
		return;
	}
	if (!info.orderedPrefix) {
		QMessageBox::warning(this, "Cannot continue",
			"The points that are in the file are not the first ones of the plan; that cannot be continued safely.");
		return;
	}
	BRILLOUIN_SETTINGS restored;
	restored = m_Brillouin->settings;
	resume::decodeSettings(info.settings, &restored);

	// Another objective in the beam (the software probably stopped during an overview with it): the measurement continues with the objective it
	// was made with. It is turned back automatically only if the operator agrees and the switch is allowed (calibrations, retract distance).
	auto switchObjective = false;
	if (info.objectiveSlot >= 0 && m_scanControl && m_scanControl->getActiveObjectiveSlot() != info.objectiveSlot) {
		const auto active = m_scanControl->getActiveObjectiveSlot();
		std::string reason;
		const auto retract = restored.overviewObjectiveRetractUm;
		if (retract != 0.0 && m_scanControl->hasObjectiveChanger() && m_scanControl->canSwitchObjectiveAutomatically(active, info.objectiveSlot, &reason)) {
			const auto answer = QMessageBox::question(this, "Objective",
				QString("The objective in slot %1 is in the beam (the software probably stopped during an overview with it). The measurement was made "
					"with the objective in slot %2.\n\nRetract z by %3 \u00b5m, turn the nosepiece to slot %2 and continue?")
				.arg(active).arg(info.objectiveSlot).arg(retract, 0, 'f', 0),
				QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
			if (answer != QMessageBox::Yes) {
				return;
			}
			switchObjective = true;
		} else {
			QMessageBox::warning(this, "Cannot continue", QString("The objective in slot %1 is in the beam, the measurement was made with the objective "
				"in slot %2. Turn the nosepiece back and press Continue again.%3").arg(active).arg(info.objectiveSlot)
				.arg(reason.empty() ? QString{} : QString("\n\nAutomatic turning is not possible: %1").arg(QString::fromStdString(reason))));
			return;
		}
	}

	auto text = info.surfaceScanPhase
		? QString("Continue the surface scan of repetition %1 of %2?\n\n%3 columns are done, the rest is scanned and the measurement goes on as always.\n\n")
			.arg(info.repetition).arg(QString::fromStdString(m_storagePath.filename)).arg((int)info.surfaceColumns.size())
		: QString("Continue repetition %1 of %2 at point %3 of %4?\n\n")
			.arg(info.repetition).arg(QString::fromStdString(m_storagePath.filename)).arg(info.pointsDone + 1).arg(info.pointsTotal);
	text += (info.state == "stopped" || info.state == "surface-scan-stopped") ? "The measurement was cancelled." : "The software stopped without finishing the measurement (crash or power loss).";
	if (!info.lastDataDate.empty()) {
		text += QString("\nLast complete point: %1").arg(QString::fromStdString(info.lastDataDate));
	}
	if (!info.incompletePoints.empty()) {
		text += QString("\n%1 point(s) that were only partly written are measured again.").arg((int)info.incompletePoints.size());
	}
	text += "\n\nThe settings of the file are loaded.";
	if (QMessageBox::question(this, "Continue measurement", text, QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes) != QMessageBox::Yes) {
		return;
	}

	// A grid in relative coordinates hangs on the start position of the measurement: ask before the stage goes back to it
	auto moveToStart = true;
	if (!info.gridAbsolute) {
		const auto answer = QMessageBox::question(this, "Start position",
			QString("The grid is relative to the start position of the measurement.\n\nMove the stage to the saved start position (%1, %2, %3 \u00b5m) first?\n\n"
				"No: the stage stays where it is, the grid is still anchored at the saved start position.")
			.arg(info.startPosition.x, 0, 'f', 1).arg(info.startPosition.y, 0, 'f', 1).arg(info.startPosition.z, 0, 'f', 1),
			QMessageBox::Yes | QMessageBox::No | QMessageBox::Cancel, QMessageBox::Yes);
		if (answer == QMessageBox::Cancel) {
			return;
		}
		moveToStart = answer == QMessageBox::Yes;
	}
	applyRestoredBrillouinSettings(restored);
	// as in Start: the ROI of the camera as the GUI holds it
	m_Brillouin->settings.camera.roi = m_deviceSettings.camera.roi;
	m_Brillouin->setSettings(m_Brillouin->settings);
	m_Brillouin->setResumeOptions(moveToStart, switchObjective);
	QMetaObject::invokeMethod(
		m_Brillouin,
		[&m_Brillouin = m_Brillouin]() { m_Brillouin->continueRepetition(); },
		Qt::AutoConnection
	);
}

void BrillouinAcquisition::showSurfacePriorQuestion(const QString& text, bool offerPrior, bool offerReuse, int timeoutSeconds) {
	QDialog dialog(this);
	dialog.setWindowTitle("Surface scan");
	auto* layout = new QVBoxLayout(&dialog);
	auto* label = new QLabel(text, &dialog);
	label->setWordWrap(true);
	layout->addWidget(label);
	auto* countdown = new QLabel(&dialog);
	layout->addWidget(countdown);
	auto* buttons = new QHBoxLayout();
	auto* yes = new QPushButton("Yes: start from it", &dialog);
	auto* no = new QPushButton("No: scan from scratch", &dialog);
	auto* reuse = new QPushButton("Reuse it (no scan)", &dialog);
	buttons->addWidget(yes);
	buttons->addWidget(no);
	if (offerReuse) {
		buttons->addWidget(reuse);
	} else {
		reuse->hide();
	}
	layout->addLayout(buttons);
	// the default is to start from the earlier scan (Yes) if that is possible, else a new scan
	yes->setEnabled(offerPrior);
	const auto defaultAnswer = offerPrior ? 1 : 0;
	(offerPrior ? yes : no)->setDefault(true);
	(offerPrior ? yes : no)->setFocus();

	auto answer = defaultAnswer;
	connect(yes, &QPushButton::clicked, &dialog, [&]() { answer = 1; dialog.accept(); });
	connect(no, &QPushButton::clicked, &dialog, [&]() { answer = 0; dialog.accept(); });
	connect(reuse, &QPushButton::clicked, &dialog, [&]() { answer = 2; dialog.accept(); });
	auto remaining = timeoutSeconds;
	const auto showRemaining = [&]() {
		countdown->setText(QString("Without an answer: %1 in %2 s").arg(offerPrior ? "scan starting from the earlier one" : "new scan from scratch").arg(remaining));
	};
	showRemaining();
	QTimer timer;
	connect(&timer, &QTimer::timeout, &dialog, [&]() {
		remaining--;
		if (remaining <= 0) {
			answer = defaultAnswer;
			dialog.accept();
			return;
		}
		showRemaining();
	});
	timer.start(1000);
	dialog.exec();
	timer.stop();
	QMetaObject::invokeMethod(
		m_Brillouin,
		[&m_Brillouin = m_Brillouin, answer]() { m_Brillouin->setSurfacePriorAnswer(answer); },
		Qt::AutoConnection
	);
}

#include "stdafx.h"
#include "filesystem"

#include "BrillouinAcquisition.h"
#include "version.h"
#include "helper/logger.h"
#include "lib/math/simplemath.h"
#include "lib/colormaps.h"

#include <cmath>
#include <algorithm>
#include <array>
#include <limits>

#include <QRegularExpression>
#include <QFileInfo>
#include <QCoreApplication>
#include <QDir>
#include <QDateTime>

using namespace std::filesystem;

namespace {
	constexpr const char* kSettingsOrg = "Guck Lab";
	constexpr const char* kSettingsApp = "Brillouin Acquisition Experimental";

	double orient2d(const POINT2& a, const POINT2& b, const POINT2& c) {
		return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
	}

	bool onSegment(const POINT2& a, const POINT2& b, const POINT2& p) {
		const auto eps = 1e-9;
		return std::min(a.x, b.x) - eps <= p.x && p.x <= std::max(a.x, b.x) + eps
			&& std::min(a.y, b.y) - eps <= p.y && p.y <= std::max(a.y, b.y) + eps;
	}

	bool segmentsIntersect(const POINT2& p1, const POINT2& p2, const POINT2& q1, const POINT2& q2) {
		const auto o1 = orient2d(p1, p2, q1);
		const auto o2 = orient2d(p1, p2, q2);
		const auto o3 = orient2d(q1, q2, p1);
		const auto o4 = orient2d(q1, q2, p2);
		const auto eps = 1e-9;

		if ((o1 > eps && o2 < -eps || o1 < -eps && o2 > eps) &&
			(o3 > eps && o4 < -eps || o3 < -eps && o4 > eps)) {
			return true;
		}
		if (std::abs(o1) <= eps && onSegment(p1, p2, q1)) return true;
		if (std::abs(o2) <= eps && onSegment(p1, p2, q2)) return true;
		if (std::abs(o3) <= eps && onSegment(q1, q2, p1)) return true;
		if (std::abs(o4) <= eps && onSegment(q1, q2, p2)) return true;
		return false;
	}

	bool isSelfIntersectingPolygon(const std::vector<POINT2>& poly) {
		if (poly.size() < 4) {
			return false;
		}
		const auto n = (int)poly.size();
		for (int i = 0; i < n; ++i) {
			const int i2 = (i + 1) % n;
			const auto& a1 = poly[(size_t)i];
			const auto& a2 = poly[(size_t)i2];
			for (int j = i + 1; j < n; ++j) {
				const int j2 = (j + 1) % n;
				if (i == j || i2 == j || j2 == i) {
					continue; // adjacent edges share a vertex
				}
				// first and last edge are adjacent in a closed polygon
				if (i == 0 && j2 == 0) {
					continue;
				}
				const auto& b1 = poly[(size_t)j];
				const auto& b2 = poly[(size_t)j2];
				if (segmentsIntersect(a1, a2, b1, b2)) {
					return true;
				}
			}
		}
		return false;
	}

	QString serializeRoiPolygon(const std::vector<POINT2>& polygon) {
		QStringList parts;
		for (const auto& p : polygon) {
			parts << (QString::number(p.x, 'g', 15) + "," + QString::number(p.y, 'g', 15));
		}
		return parts.join(";");
	}

	std::vector<POINT2> deserializeRoiPolygon(const QString& serialized) {
		std::vector<POINT2> polygon;
		const auto entries = serialized.split(";", Qt::SkipEmptyParts);
		polygon.reserve(entries.size());
		for (const auto& entry : entries) {
			const auto xy = entry.split(",", Qt::KeepEmptyParts);
			if (xy.size() != 2) {
				continue;
			}
			bool okX = false;
			bool okY = false;
			const auto x = xy[0].toDouble(&okX);
			const auto y = xy[1].toDouble(&okY);
			if (okX && okY) {
				polygon.push_back(POINT2{ x, y });
			}
		}
		return polygon;
	}
}

BrillouinAcquisition::BrillouinAcquisition(QWidget *parent) noexcept :
	QMainWindow(parent), ui(new Ui::BrillouinAcquisitionClass) {
	ui->setupUi(this);

	m_surfaceReviewTimer = new QTimer(this);
	m_surfaceReviewTimer->setInterval(1000);
	connect(m_surfaceReviewTimer, &QTimer::timeout, this, &BrillouinAcquisition::onSurfaceReviewTimerTick);

	// Set window title
	auto title = QString{ "BrillouinAcquisition v%1.%2.%3" }.arg(Version::MAJOR).arg(Version::MINOR).arg(Version::PATCH);
	if (Version::PRERELEASE.length() > 0) {
		title += "-" + QString::fromStdString(Version::PRERELEASE);
	}
	#ifdef _DEBUG
		title += QString{ " - Debug" };
	#endif
	this->setWindowTitle(title);

	static QMetaObject::Connection connection;
	// slot to limit the axis of the camera display after user interaction
	connection = QWidget::connect<void(QCPAxis::*)(const QCPRange &)>(
		ui->customplot->xAxis,
		&QCPAxis::rangeChanged,
		this,
		[this](QCPRange newRange) { xAxisRangeChanged(newRange); }
	);
	connection = QWidget::connect<void(QCPAxis::*)(const QCPRange &)>(
		ui->customplot->yAxis,
		&QCPAxis::rangeChanged,
		this,
		[this](QCPRange newRange) { yAxisRangeChanged(newRange); }
	);

	// slot to limit the axis of the camera display after user interaction
	connection = QWidget::connect<void(QCPAxis::*)(const QCPRange &)>(
		ui->customplot_brightfield->xAxis,
		&QCPAxis::rangeChanged,
		this,
		[this](QCPRange newRange) { xAxisRangeChangedODT(newRange); }
	);
	connection = QWidget::connect<void(QCPAxis::*)(const QCPRange &)>(
		ui->customplot_brightfield->yAxis,
		&QCPAxis::rangeChanged,
		this,
		[this](QCPRange newRange) { yAxisRangeChangedODT(newRange); }
	);

	// slot to update filename
	connection = QWidget::connect(
		m_acquisition,
		&Acquisition::s_filenameChanged,
		this,
		[this](std::string filename) { updateFilename(filename); }
	);

	// slot to show current acquisition progress
	connection = QWidget::connect(
		m_acquisition,
		&Acquisition::s_enabledModes,
		this,
		[this](ACQUISITION_MODE modes) { showEnabledModes(modes); }
	);

	// slot to show current acquisition position
	connection = QWidget::connect(
		m_Brillouin,
		&Brillouin::s_positionChanged,
		this,
		[this](POINT3 position, int imageNr) { showAcqPosition(position, imageNr); }
	);

	// slot to show current acquisition state
	connection = QWidget::connect(
		m_Brillouin,
		&Brillouin::s_acquisitionStatus,
		this,
		[this](ACQUISITION_STATUS state) { showBrillouinStatus(state); }
	);

	// slot to show current repetition progress
	connection = QWidget::connect(
		m_Brillouin,
		&Brillouin::s_repetitionProgress,
		this,
		[this](double progress, int seconds) { showBrillouinProgress(progress, seconds); }
	);
	connection = QWidget::connect(
		m_Brillouin,
		&Brillouin::s_surfaceScanProgress,
		this,
		[this](double progress, const QString& message) { showSurfaceScanProgress(progress, message); }
	);

	// slot to show calibration running
	connection = QWidget::connect(
		m_Brillouin,
		&Brillouin::s_calibrationRunning,
		this,
		[this](bool isCalibrating) { showCalibrationRunning(isCalibrating); }
	);

	// slot to show time until next calibration
	connection = QWidget::connect(
		m_Brillouin,
		&Brillouin::s_timeToCalibration,
		this,
		[this](int value) { showCalibrationInterval(value); }
	);

	// slot to show repetitions
	connection = QWidget::connect(
		m_Brillouin,
		&Brillouin::s_totalProgress,
		this,
		[this](int repNumber, int timeToNext) { showRepProgress(repNumber, timeToNext); }
	);
	// slot to update the scan order
	connection = QWidget::connect(
		m_Brillouin,
		&Brillouin::s_scanOrderChanged,
		this,
		[this](SCAN_ORDER scanOrder) { scanOrderChanged(scanOrder); }
	);

	// slot to positions in brightfield
	connection = QWidget::connect(
		m_Brillouin,
		&Brillouin::s_orderedPositionsChanged,
		this,
		[this](std::vector<POINT3> orderedPositions, bool isAbsolute) { AOI_changed(orderedPositions, isAbsolute); }
	);
	connection = QWidget::connect(
		m_Brillouin,
		&Brillouin::s_excludedPositionsChanged,
		this,
		[this](std::vector<POINT3> excludedPositions) { excludedAOI_changed(excludedPositions); }
	);

	m_Brillouin->determineScanOrder();

	qRegisterMetaType<std::string>("std::string");
	qRegisterMetaType<AT_64>("AT_64");
	qRegisterMetaType<StoragePath>("StoragePath");
	qRegisterMetaType<ACQUISITION_MODE>("ACQUISITION_MODE");
	qRegisterMetaType<ACQUISITION_STATUS>("ACQUISITION_STATUS");
	qRegisterMetaType<BRILLOUIN_SETTINGS>("BRILLOUIN_SETTINGS");
	qRegisterMetaType<CAMERA_SETTINGS>("CAMERA_SETTINGS");
	qRegisterMetaType<CAMERA_SETTING>("CAMERA_SETTING");
	qRegisterMetaType<CAMERA_OPTIONS>("CAMERA_OPTIONS");
	qRegisterMetaType<std::vector<int>>("std::vector<int>");
	qRegisterMetaType<std::vector<double>>("std::vector<double>");
	qRegisterMetaType<std::vector<float>>("std::vector<float>");
	qRegisterMetaType<std::vector<unsigned short>>("std::vector<unsigned short>");
	qRegisterMetaType<std::vector<unsigned char>>("std::vector<unsigned char>");
	qRegisterMetaType<std::vector<unsigned int>>("std::vector<unsigned int>");
	qRegisterMetaType<std::vector<FLUORESCENCE_MODE>>("std::vector<FLUORESCENCE_MODE>");
	qRegisterMetaType<std::vector<POINT2>>("std::vector<POINT2>");
	qRegisterMetaType<std::vector<POINT3>>("std::vector<POINT3>");
	qRegisterMetaType<QSerialPort::SerialPortError>("QSerialPort::SerialPortError");
	qRegisterMetaType<IMAGE<unsigned char>*>("IMAGE<unsigned char>*");
	qRegisterMetaType<IMAGE<unsigned short>*>("IMAGE<unsigned short>*");
	qRegisterMetaType<CALIBRATION<unsigned char>*>("CALIBRATION<unsigned char>*");
	qRegisterMetaType<CALIBRATION<unsigned short>*>("CALIBRATION<unsigned short>*");
	qRegisterMetaType<ScanPreset>("ScanPreset");
	qRegisterMetaType<DeviceElement>("DeviceElement");
	qRegisterMetaType<SensorTemperature>("SensorTemperature");
	qRegisterMetaType<POINT3>("POINT3");
	qRegisterMetaType<POINT2>("POINT2");
	qRegisterMetaType<BOUNDS>("BOUNDS");
	qRegisterMetaType<QMouseEvent*>("QMouseEvent*");
	qRegisterMetaType<VOLTAGE2>("VOLTAGE2");
	qRegisterMetaType<ODT_MODE>("ODT_MODE");
	qRegisterMetaType<ODT_SETTING>("ODT_SETTING");
	qRegisterMetaType<ODT_SETTINGS>("ODT_SETTINGS");
	qRegisterMetaType<ODTIMAGE<unsigned char>*>("ODTIMAGE<unsigned char>*");
	qRegisterMetaType<ODTIMAGE<unsigned short>*>("ODTIMAGE<unsigned short>*");
	qRegisterMetaType<FLUOIMAGE<unsigned char>*>("FLUOIMAGE<unsigned char>*");
	qRegisterMetaType<FLUOIMAGE<unsigned short>*>("FLUOIMAGE<unsigned short>*");
	qRegisterMetaType<FLUORESCENCE_SETTINGS>("FLUORESCENCE_SETTINGS");
	qRegisterMetaType<FLUORESCENCE_MODE>("FLUORESCENCE_MODE");
	qRegisterMetaType<PLOT_SETTINGS*>("PLOT_SETTINGS*");
	qRegisterMetaType<PreviewBuffer<unsigned short>*>("PreviewBuffer<unsigned short>*");
	qRegisterMetaType<PreviewBuffer<unsigned char>*>("PreviewBuffer<unsigned char>*");
	qRegisterMetaType<unsigned char*>("unsigned char*");
	qRegisterMetaType<unsigned short*>("unsigned short*");
	qRegisterMetaType<bool*>("bool*");
	qRegisterMetaType<VoltageCalibrationData>("VoltageCalibrationData");
	qRegisterMetaType<ScaleCalibrationData>("ScaleCalibrationData");
	qRegisterMetaType<SCAN_ORDER>("SCAN_ORDER");
	
	// Set up icons
	m_icons.disconnected.addFile(":/BrillouinAcquisition/assets/00disconnected10px.png", QSize(10, 10));
	m_icons.disconnected.addFile(":/BrillouinAcquisition/assets/00disconnected16px.png", QSize(16, 16));
	m_icons.disconnected.addFile(":/BrillouinAcquisition/assets/00disconnected24px.png", QSize(24, 24));
	m_icons.disconnected.addFile(":/BrillouinAcquisition/assets/00disconnected32px.png", QSize(32, 32));

	m_icons.standby.addFile(":/BrillouinAcquisition/assets/01standby10px.png", QSize(10, 10));
	m_icons.standby.addFile(":/BrillouinAcquisition/assets/01standby16px.png", QSize(16, 16));
	m_icons.standby.addFile(":/BrillouinAcquisition/assets/01standby24px.png", QSize(24, 24));
	m_icons.standby.addFile(":/BrillouinAcquisition/assets/01standby32px.png", QSize(32, 32));

	m_icons.cooling.addFile(":/BrillouinAcquisition/assets/02cooling10px.png", QSize(10, 10));
	m_icons.cooling.addFile(":/BrillouinAcquisition/assets/02cooling16px.png", QSize(16, 16));
	m_icons.cooling.addFile(":/BrillouinAcquisition/assets/02cooling24px.png", QSize(24, 24));
	m_icons.cooling.addFile(":/BrillouinAcquisition/assets/02cooling32px.png", QSize(32, 32));

	m_icons.ready.addFile(":/BrillouinAcquisition/assets/03ready10px.png", QSize(10, 10));
	m_icons.ready.addFile(":/BrillouinAcquisition/assets/03ready16px.png", QSize(16, 16));
	m_icons.ready.addFile(":/BrillouinAcquisition/assets/03ready24px.png", QSize(24, 24));
	m_icons.ready.addFile(":/BrillouinAcquisition/assets/03ready32px.png", QSize(32, 32));

	m_icons.fluoBlue.addFile(":/BrillouinAcquisition/assets/04fluoBlue10px.png", QSize(10, 10));
	m_icons.fluoBlue.addFile(":/BrillouinAcquisition/assets/04fluoBlue16px.png", QSize(16, 16));
	m_icons.fluoBlue.addFile(":/BrillouinAcquisition/assets/04fluoBlue24px.png", QSize(24, 24));
	m_icons.fluoBlue.addFile(":/BrillouinAcquisition/assets/04fluoBlue32px.png", QSize(32, 32));

	m_icons.fluoGreen.addFile(":/BrillouinAcquisition/assets/05fluoGreen10px.png", QSize(10, 10));
	m_icons.fluoGreen.addFile(":/BrillouinAcquisition/assets/05fluoGreen16px.png", QSize(16, 16));
	m_icons.fluoGreen.addFile(":/BrillouinAcquisition/assets/05fluoGreen24px.png", QSize(24, 24));
	m_icons.fluoGreen.addFile(":/BrillouinAcquisition/assets/05fluoGreen32px.png", QSize(32, 32));

	m_icons.fluoRed.addFile(":/BrillouinAcquisition/assets/06fluoRed10px.png", QSize(10, 10));
	m_icons.fluoRed.addFile(":/BrillouinAcquisition/assets/06fluoRed16px.png", QSize(16, 16));
	m_icons.fluoRed.addFile(":/BrillouinAcquisition/assets/06fluoRed24px.png", QSize(24, 24));
	m_icons.fluoRed.addFile(":/BrillouinAcquisition/assets/06fluoRed32px.png", QSize(32, 32));

	m_icons.fluoBrightfield.addFile(":/BrillouinAcquisition/assets/07fluoBrightfield10px.png", QSize(10, 10));
	m_icons.fluoBrightfield.addFile(":/BrillouinAcquisition/assets/07fluoBrightfield16px.png", QSize(16, 16));
	m_icons.fluoBrightfield.addFile(":/BrillouinAcquisition/assets/07fluoBrightfield24px.png", QSize(24, 24));
	m_icons.fluoBrightfield.addFile(":/BrillouinAcquisition/assets/07fluoBrightfield32px.png", QSize(32, 32));

	ui->settingsWidget->setTabIcon(0, m_icons.disconnected);
	ui->settingsWidget->setTabIcon(1, m_icons.disconnected);
	ui->settingsWidget->setTabIcon(2, m_icons.disconnected);
	ui->settingsWidget->setTabIcon(3, m_icons.disconnected);
	ui->settingsWidget->setIconSize(QSize(10, 10));

	// Set icons for fluorescence
	QLabel *fluoBlueIcon = new QLabel(ui->fluoBlueIcon);
	fluoBlueIcon->setPixmap(m_icons.fluoBlue.pixmap(QSize(15, 15)));
	fluoBlueIcon->show();

	QLabel *fluoGreenIcon = new QLabel(ui->fluoGreenIcon);
	fluoGreenIcon->setPixmap(m_icons.fluoGreen.pixmap(QSize(15, 15)));
	fluoGreenIcon->show();

	QLabel *fluoRedIcon = new QLabel(ui->fluoRedIcon);
	fluoRedIcon->setPixmap(m_icons.fluoRed.pixmap(QSize(15, 15)));
	fluoRedIcon->show();

	QLabel *fluoBrightfieldIcon = new QLabel(ui->fluoBrightfieldIcon);
	fluoBrightfieldIcon->setPixmap(m_icons.fluoBrightfield.pixmap(QSize(15, 15)));
	fluoBrightfieldIcon->show();

	ui->actionEnable_Cooling->setEnabled(false);
	ui->autoscalePlot->setChecked(m_BrillouinPlot.autoscale);

	connection = QWidget::connect<void(converter::*)(PLOT_SETTINGS*, long long, long long, std::vector<unsigned char>)>(
		m_converter,
		&converter::s_converted,
		this,
		[this](PLOT_SETTINGS* plotSettings, long long dim_x, long long dim_y, std::vector<unsigned char> unpackedBuffer) { plot(plotSettings, dim_x, dim_y, unpackedBuffer); }
	);
	connection = QWidget::connect<void(converter::*)(PLOT_SETTINGS*, long long, long long, std::vector<unsigned short>)>(
		m_converter,
		&converter::s_converted,
		this,
		[this](PLOT_SETTINGS* plotSettings, long long dim_x, long long dim_y, std::vector<unsigned short> unpackedBuffer) { plot(plotSettings, dim_x, dim_y, unpackedBuffer); }
	);
	connection = QWidget::connect<void(converter::*)(PLOT_SETTINGS*, long long, long long, std::vector<double>)>(
		m_converter,
		&converter::s_converted,
		this,
		[this](PLOT_SETTINGS* plotSettings, long long dim_x, long long dim_y, std::vector<double> unpackedBuffer) { plot(plotSettings, dim_x, dim_y, unpackedBuffer); }
	);
	connection = QWidget::connect<void(converter::*)(PLOT_SETTINGS*, long long, long long, std::vector<float>)>(
		m_converter,
		&converter::s_converted,
		this,
		[this](PLOT_SETTINGS* plotSettings, long long dim_x, long long dim_y, std::vector<float> unpackedBuffer) { plot(plotSettings, dim_x, dim_y, unpackedBuffer); }
	);
	connection = QWidget::connect<void(converter::*)(PLOT_SETTINGS*, long long, long long, std::vector<int>)>(
		m_converter,
		&converter::s_converted,
		this,
		[this](PLOT_SETTINGS* plotSettings, long long dim_x, long long dim_y, std::vector<int> unpackedBuffer) { plot(plotSettings, dim_x, dim_y, unpackedBuffer); }
	);

	// start acquisition thread
	m_acquisitionThread.startWorker(m_acquisition);
	// start Brillouin thread
	m_acquisitionThread.startWorker(m_Brillouin);
	// start plotting thread
	m_plottingThread.startWorker(m_converter);

	// set up the QCPColorMap:
	m_BrillouinPlot = {
		ui->customplot,
		new QCPColorMap(ui->customplot->xAxis, ui->customplot->yAxis),
		{ 100, 300 },
		ui->rangeLower,
		ui->rangeUpper,
		[this](QCPRange range) {
			this->updateCLimRange(ui->rangeLower, ui->rangeUpper, range);
		},
		false,
		CustomGradientPreset::gpViridis
	};

	m_ODTPlot = {
		ui->customplot_brightfield,
		new QCPColorMap(ui->customplot_brightfield->xAxis, ui->customplot_brightfield->yAxis),
		{ 0, 100 },
		ui->rangeLowerODT,
		ui->rangeUpperODT,
		[this](QCPRange range) {
			this->updateCLimRange(ui->rangeLowerODT, ui->rangeUpperODT, range);
		},
		false,
		CustomGradientPreset::gpGrayscale
	};

	// set up the camera image plot
	BrillouinAcquisition::initializePlot(m_BrillouinPlot);
	BrillouinAcquisition::initializePlot(m_ODTPlot);
	// Lock the brightfield view to a 1:1 axis-unit-to-pixel ratio so the image never gets
	// stretched/squashed to whatever rectangle the surrounding layout happens to hand the
	// widget (e.g. when other panels show/hide and the horizontal split changes) - without
	// this, a rectangular camera frame (e.g. 1024x1280) can visibly render as square.
	m_ODTPlot.plotHandle->yAxis->setScaleRatio(m_ODTPlot.plotHandle->xAxis, 1.0);

	connection = QWidget::connect(
		m_ODTPlot.plotHandle,
		&QCustomPlot::mousePress,
		this,
		[this](QMouseEvent* event) { plotClick(event); }
	);

	connection = QWidget::connect(
		m_ODTPlot.plotHandle,
		&QCustomPlot::mouseMove,
		this,
		[this](QMouseEvent* event) {
			if (!m_scanControl) {
				return;
			}
			if (m_draggingRoiVertex && m_draggedRoiVertexIndex >= 0) {
				updateDraggedRoiVertex(mainRoiTarget(), event);
			} else if (m_draggingBackgroundRoiVertex && m_draggedBackgroundRoiVertexIndex >= 0) {
				updateDraggedRoiVertex(backgroundRoiTarget(), event);
			}
		}
	);

	connection = QWidget::connect(
		m_ODTPlot.plotHandle,
		&QCustomPlot::mouseRelease,
		this,
		[this](QMouseEvent* event) {
			Q_UNUSED(event);
			if (m_draggingRoiVertex) {
				event->accept();
				m_draggingRoiVertex = false;
				m_draggedRoiVertexIndex = -1;
				QMetaObject::invokeMethod(m_Brillouin, "updatePositions", Qt::AutoConnection);
				updateBrillouinSettings();
				return;
			}
			if (m_draggingBackgroundRoiVertex) {
				event->accept();
				m_draggingBackgroundRoiVertex = false;
				m_draggedBackgroundRoiVertexIndex = -1;
				updateBrillouinSettings();
			}
		}
	);

	connection = QWidget::connect(
		ui->customplot,
		&QCustomPlot::mousePress,
		this,
		[this](QMouseEvent* event) {
			if (!(m_editSpectralProxyRoiCheckbox && m_editSpectralProxyRoiCheckbox->isChecked())) {
				return;
			}
			event->accept(); // prevent default plot drag/zoom handling
			if (event->button() == Qt::RightButton) {
				m_spectralProxyDragActive = false;
				clearSpectralProxyRois();
				return;
			}
			if (event->button() != Qt::LeftButton) {
				return;
			}
			m_spectralProxyDragStart = event->pos();
			m_spectralProxyDragActive = true;
			m_spectralProxyActiveRoiIndex = m_spectralProxyNextRoiIndex;
			ensureSpectralProxyRoiRect(m_spectralProxyActiveRoiIndex);
		}
	);

	connection = QWidget::connect(
		ui->customplot,
		&QCustomPlot::mouseMove,
		this,
		[this](QMouseEvent* event) {
			if (m_editSpectralProxyRoiCheckbox && m_editSpectralProxyRoiCheckbox->isChecked()) {
				event->accept(); // block plot translation while in ROI edit mode
			}
			if (!m_spectralProxyDragActive) {
				return;
			}
			auto* rectItem = ensureSpectralProxyRoiRect(m_spectralProxyActiveRoiIndex);
			const auto x0 = ui->customplot->xAxis->pixelToCoord(m_spectralProxyDragStart.x());
			const auto y0 = ui->customplot->yAxis->pixelToCoord(m_spectralProxyDragStart.y());
			const auto x1 = ui->customplot->xAxis->pixelToCoord(event->pos().x());
			const auto y1 = ui->customplot->yAxis->pixelToCoord(event->pos().y());
			rectItem->topLeft->setCoords(std::min(x0, x1), std::max(y0, y1));
			rectItem->bottomRight->setCoords(std::max(x0, x1), std::min(y0, y1));
			ui->customplot->replot();
		}
	);

	connection = QWidget::connect(
		ui->customplot,
		&QCustomPlot::mouseRelease,
		this,
		[this](QMouseEvent* event) {
			if (m_editSpectralProxyRoiCheckbox && m_editSpectralProxyRoiCheckbox->isChecked()) {
				event->accept(); // keep plot static during ROI draw
			}
			if (!m_spectralProxyDragActive) {
				return;
			}
			m_spectralProxyDragActive = false;
			const auto x0 = ui->customplot->xAxis->pixelToCoord(m_spectralProxyDragStart.x());
			const auto y0 = ui->customplot->yAxis->pixelToCoord(m_spectralProxyDragStart.y());
			const auto x1 = ui->customplot->xAxis->pixelToCoord(event->pos().x());
			const auto y1 = ui->customplot->yAxis->pixelToCoord(event->pos().y());

			auto* mapData = m_BrillouinPlot.colorMap ? m_BrillouinPlot.colorMap->data() : nullptr;
			const auto liveRoi = currentSpectralCameraRoi();
			const int frameW = mapData ? std::max(1, mapData->keySize()) : std::max(1, (int)liveRoi.width_binned);
			const int frameH = mapData ? std::max(1, mapData->valueSize()) : std::max(1, (int)liveRoi.height_binned);
			int cellX0{ 0 };
			int cellY0{ 0 };
			int cellX1{ 0 };
			int cellY1{ 0 };
			if (mapData) {
				mapData->coordToCell(x0, y0, &cellX0, &cellY0);
				mapData->coordToCell(x1, y1, &cellX1, &cellY1);
			} else {
				cellX0 = (int)std::floor(std::max(0.0, x0 - 1.0));
				cellY0 = (int)std::floor(std::max(0.0, y0 - 1.0));
				cellX1 = (int)std::floor(std::max(0.0, x1 - 1.0));
				cellY1 = (int)std::floor(std::max(0.0, y1 - 1.0));
			}

			const int clampedLeft = std::clamp(std::min(cellX0, cellX1), 0, frameW - 1);
			const int clampedRight = std::clamp(std::max(cellX0, cellX1), clampedLeft, frameW - 1);
			const int clampedDisplayBottom = std::clamp(std::min(cellY0, cellY1), 0, frameH - 1);
			const int clampedDisplayTop = std::clamp(std::max(cellY0, cellY1), clampedDisplayBottom, frameH - 1);
			const int displayRoiTop = clampedDisplayBottom;
			const int displayRoiHeight = clampedDisplayTop - clampedDisplayBottom + 1;
			// Absolute sensor position/size of the frame this ROI is being drawn against,
			// so estimateFrameMetric() can remap it correctly even if the camera ROI's
			// origin (not just its size) differs at measurement time - see the comment
			// on surfaceProxyRoiFrameWidth in Brillouin.h. liveRoi (not
			// m_Brillouin->settings.camera.roi) so this matches frameW/frameH above exactly -
			// see currentSpectralCameraRoi()'s own comment.
			const auto& currentRoi = liveRoi;
			if (m_spectralProxyActiveRoiIndex == 1) {
				m_Brillouin->settings.surfaceProxyRoi2Left = clampedLeft;
				m_Brillouin->settings.surfaceProxyRoi2Top = displayRoiTop;
				m_Brillouin->settings.surfaceProxyRoi2Width = clampedRight - clampedLeft + 1;
				m_Brillouin->settings.surfaceProxyRoi2Height = displayRoiHeight;
				m_Brillouin->settings.surfaceProxyRoi2FrameWidth = frameW;
				m_Brillouin->settings.surfaceProxyRoi2FrameHeight = frameH;
				m_Brillouin->settings.surfaceProxyRoi2FrameOriginLeft = currentRoi.left;
				m_Brillouin->settings.surfaceProxyRoi2FrameOriginBottom = currentRoi.bottom;
				m_Brillouin->settings.surfaceProxyRoi2FrameWidthPhysical = currentRoi.width_physical;
				m_Brillouin->settings.surfaceProxyRoi2FrameHeightPhysical = currentRoi.height_physical;
			} else {
				m_Brillouin->settings.surfaceProxyRoiLeft = clampedLeft;
				m_Brillouin->settings.surfaceProxyRoiTop = displayRoiTop;
				m_Brillouin->settings.surfaceProxyRoiWidth = clampedRight - clampedLeft + 1;
				m_Brillouin->settings.surfaceProxyRoiHeight = displayRoiHeight;
				m_Brillouin->settings.surfaceProxyRoiFrameWidth = frameW;
				m_Brillouin->settings.surfaceProxyRoiFrameHeight = frameH;
				m_Brillouin->settings.surfaceProxyRoiFrameOriginLeft = currentRoi.left;
				m_Brillouin->settings.surfaceProxyRoiFrameOriginBottom = currentRoi.bottom;
				m_Brillouin->settings.surfaceProxyRoiFrameWidthPhysical = currentRoi.width_physical;
				m_Brillouin->settings.surfaceProxyRoiFrameHeightPhysical = currentRoi.height_physical;
			}

			updateSpectralProxyRoiRect(m_spectralProxyActiveRoiIndex);
			m_spectralProxyNextRoiIndex = 1 - m_spectralProxyActiveRoiIndex;
			ui->customplot->replot();
		}
	);

	initializeODTVoltagePlot(ui->alignmentVoltagesODT);
	initializeODTVoltagePlot(ui->acquisitionVoltagesODT);

	// First read device settings then init devices
	readSettings();
	initCameraBrillouin();
	initScanControl();
	initCamera();

	// set up laser focus marker
	ui->addFocusMarker_brightfield->setIcon(m_icons.fluoBlue);
	ui->addFocusMarker_brightfield->setText("");
	ui->relocateFocusMarker_brightfield->setIcon(m_icons.fluoGreen);
	ui->relocateFocusMarker_brightfield->setText("");
	initializeLaserPositionLocation();

	updateBrillouinSettings();
	initSettingsDialog();

	// Set up GUI
	initBeampathButtons();
	updateSavedPositions();

	// Runtime controls for advanced scan planning.
	// Keep these controls in a dedicated AOI section to avoid crowding legacy scan-direction controls.
	if (ui->acquisitionAOI != nullptr) {
		auto* grid = ui->advancedPlanningGrid;
		if (grid != nullptr) {
			// Match legacy panel rhythm: consistent row spacing with small block separators.
			grid->setVerticalSpacing(6);
			grid->setHorizontalSpacing(10);

			m_useRoiMaskCheckbox = ui->useRoiMaskCheckbox;
			m_editRoiCheckbox = ui->drawRoiButton;
			m_clearRoiButton = ui->clearRoiButton;
			m_useBackgroundRoiMaskCheckbox = ui->useBackgroundRoiMaskCheckbox;
			m_editBackgroundRoiCheckbox = ui->drawBackgroundRoiButton;
			m_clearBackgroundRoiButton = ui->clearBackgroundRoiButton;
			m_useSurfaceFollowCheckbox = ui->useSurfaceFollowCheckbox;
			m_preScanXYBinSpinBox = ui->preScanXYBinSpinBox;
			m_additionalBoundaryPointsSpinBox = ui->additionalBoundaryPointsSpinBox;
			m_preScanZStepSpinBox = ui->preScanZStepSpinBox;
			m_preScanZTravelSpinBox = ui->preScanZTravelSpinBox;
			m_surfaceDropSpinBox = ui->surfaceDropSpinBox;
			m_mediumReferenceFrameCountSpinBox = ui->mediumReferenceFrameCountSpinBox;
			m_surfaceMaxRewindSpinBox = ui->surfaceMaxRewindSpinBox;
			m_surfaceVerificationStepsSpinBox = ui->surfaceVerificationStepsSpinBox;
			m_surfaceVerificationFrameAverageSpinBox = ui->surfaceVerificationFrameAverageSpinBox;
			m_surfaceVerificationToleranceSpinBox = ui->surfaceVerificationToleranceSpinBox;
			m_absoluteGridCheckbox = ui->absoluteGridCheckbox;
			m_gridHysteresisCompensationCheckbox = ui->gridHysteresisCompensationCheckbox;
			m_doseProtectionCheckbox = ui->doseProtectionCheckbox;
			m_saveOverviewBrightfieldPerZCheckbox = ui->saveOverviewBrightfieldPerZCheckbox;
			m_overviewSingleImageRadio = ui->overviewSingleImageRadio;
			m_overviewFullGridRadio = ui->overviewFullGridRadio;
			m_overviewFullStackCheckbox = ui->overviewFullStackCheckbox;
			m_capturePerPointBrightfieldCheckbox = ui->capturePerPointBrightfieldCheckbox;
			m_perPointBrightfieldEveryNSpinBox = ui->perPointBrightfieldEveryNSpinBox;
			m_perPointBrightfieldDuringAcquisitionCheckbox = ui->perPointBrightfieldDuringAcquisitionCheckbox;
			m_editSpectralProxyRoiCheckbox = ui->editSpectralProxyRoiCheckbox;

			// Main ROI and background ROI share every bit of editing/preview/clear logic (see
			// RoiTarget's own comment) - only the target passed to each shared helper differs.
			connect(m_useRoiMaskCheckbox, &QCheckBox::toggled, this, [this](bool enabled) {
				if (enabled && !tryEnableRoiMaskFor(mainRoiTarget())) {
					const QSignalBlocker blocker(m_useRoiMaskCheckbox);
					m_useRoiMaskCheckbox->setChecked(false);
					return;
				}
				m_Brillouin->settings.useRoiMask = enabled;
				QMetaObject::invokeMethod(m_Brillouin, "updatePositions", Qt::AutoConnection);
				update_AOI_preview();
			});

			connect(m_editRoiCheckbox, &QAbstractButton::toggled, this, [this](bool enabled) {
				if (enabled) {
					m_ODTPlot.plotHandle->setInteractions(QCP::iNone);
					statusBar()->showMessage("Draw ROI mode: click to add points, drag points to adjust.", 5000);
					// Only one polygon can be edited by clicking at a time - see plotClick().
					if (m_editBackgroundRoiCheckbox && m_editBackgroundRoiCheckbox->isChecked()) {
						const QSignalBlocker blocker(m_editBackgroundRoiCheckbox);
						m_editBackgroundRoiCheckbox->setChecked(false);
					}
				} else {
					m_ODTPlot.plotHandle->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
					m_draggingRoiVertex = false;
					m_draggedRoiVertexIndex = -1;
				}
			});

			connect(m_clearRoiButton, &QPushButton::clicked, this, [this]() {
				clearRoiPolygonFor(mainRoiTarget());
				QMetaObject::invokeMethod(m_Brillouin, "updatePositions", Qt::AutoConnection);
				update_AOI_preview();
			});

			connect(m_useBackgroundRoiMaskCheckbox, &QCheckBox::toggled, this, [this](bool enabled) {
				if (enabled && !tryEnableRoiMaskFor(backgroundRoiTarget())) {
					const QSignalBlocker blocker(m_useBackgroundRoiMaskCheckbox);
					m_useBackgroundRoiMaskCheckbox->setChecked(false);
					return;
				}
				m_Brillouin->settings.useBackgroundRoiMask = enabled;
				update_AOI_preview();
			});

			connect(m_editBackgroundRoiCheckbox, &QAbstractButton::toggled, this, [this](bool enabled) {
				if (enabled) {
					m_ODTPlot.plotHandle->setInteractions(QCP::iNone);
					statusBar()->showMessage("Draw background ROI mode: click to add points, drag points to adjust.", 5000);
					if (m_editRoiCheckbox && m_editRoiCheckbox->isChecked()) {
						const QSignalBlocker blocker(m_editRoiCheckbox);
						m_editRoiCheckbox->setChecked(false);
					}
				} else {
					m_ODTPlot.plotHandle->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
					m_draggingBackgroundRoiVertex = false;
					m_draggedBackgroundRoiVertexIndex = -1;
				}
			});

			connect(m_clearBackgroundRoiButton, &QPushButton::clicked, this, [this]() {
				clearRoiPolygonFor(backgroundRoiTarget());
				update_AOI_preview();
			});

			connect(m_useSurfaceFollowCheckbox, &QCheckBox::toggled, this, [this](bool enabled) {
				m_Brillouin->settings.useSurfaceFollow = enabled;
				if (m_preScanXYBinSpinBox) m_preScanXYBinSpinBox->setEnabled(enabled);
				if (m_additionalBoundaryPointsSpinBox) m_additionalBoundaryPointsSpinBox->setEnabled(enabled);
				if (m_preScanZStepSpinBox) m_preScanZStepSpinBox->setEnabled(enabled);
				if (m_preScanZTravelSpinBox) m_preScanZTravelSpinBox->setEnabled(enabled);
				if (m_surfaceDropSpinBox) m_surfaceDropSpinBox->setEnabled(enabled);
				if (m_mediumReferenceFrameCountSpinBox) m_mediumReferenceFrameCountSpinBox->setEnabled(enabled);
				if (m_surfaceMaxRewindSpinBox) m_surfaceMaxRewindSpinBox->setEnabled(enabled);
				if (m_surfaceVerificationStepsSpinBox) m_surfaceVerificationStepsSpinBox->setEnabled(enabled);
				if (m_surfaceVerificationFrameAverageSpinBox) m_surfaceVerificationFrameAverageSpinBox->setEnabled(enabled);
				if (m_surfaceVerificationToleranceSpinBox) m_surfaceVerificationToleranceSpinBox->setEnabled(enabled);
				if (m_editSpectralProxyRoiCheckbox) m_editSpectralProxyRoiCheckbox->setEnabled(enabled);
				updateBrillouinStartAvailability();
				update_AOI_preview();
			});
			connect(m_preScanXYBinSpinBox, qOverload<int>(&QSpinBox::valueChanged), this, [this](int value) {
				m_Brillouin->settings.preScanXYBin = std::max(1, value);
				update_AOI_preview();
			});
			connect(m_additionalBoundaryPointsSpinBox, qOverload<int>(&QSpinBox::valueChanged), this, [this](int value) {
				m_Brillouin->settings.additionalBoundaryPoints = std::max(0, value);
				update_AOI_preview();
			});
			connect(m_preScanZStepSpinBox, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this](double value) {
				m_Brillouin->settings.preScanZStepUm = std::max(0.01, value);
			});
			connect(m_preScanZTravelSpinBox, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this](double value) {
				m_Brillouin->settings.preScanZTravelRangeUm = std::max(0.01, value);
				updateBrillouinStartAvailability();
			});

			connect(m_surfaceDropSpinBox, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this](double value) {
				m_Brillouin->settings.surfaceDropFraction = value / 100.0;
			});

			connect(m_mediumReferenceFrameCountSpinBox, qOverload<int>(&QSpinBox::valueChanged), this, [this](int value) {
				m_Brillouin->settings.mediumReferenceFrameCount = std::max(1, value);
			});

			connect(m_surfaceMaxRewindSpinBox, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this](double value) {
				m_Brillouin->settings.surfaceMaxRewindUm = std::max(0.0, value);
			});

			connect(m_surfaceVerificationStepsSpinBox, qOverload<int>(&QSpinBox::valueChanged), this, [this](int value) {
				m_Brillouin->settings.surfaceVerificationSteps = std::max(0, value);
			});

			connect(m_surfaceVerificationFrameAverageSpinBox, qOverload<int>(&QSpinBox::valueChanged), this, [this](int value) {
				m_Brillouin->settings.surfaceVerificationFrameAverage = std::max(1, value);
			});

			connect(m_surfaceVerificationToleranceSpinBox, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this](double value) {
				m_Brillouin->settings.surfaceVerificationToleranceFraction = std::max(0.0, value) / 100.0;
			});

			connect(m_absoluteGridCheckbox, &QCheckBox::toggled, this, [this](bool enabled) {
				preservePhysicalGridForAbsoluteMode(enabled);
				m_Brillouin->settings.gridCoordinatesAbsolute = enabled;
				ui->setHome->setDisabled(enabled);
				ui->moveHome->setDisabled(enabled);
				// See the matching comment on this same lock in the ACQUISITION_STATUS
				// handler - applied here too so it takes effect immediately on toggling,
				// rather than waiting for the next status change to lock/unlock the fields.
				ui->startX->setDisabled(enabled);
				ui->startY->setDisabled(enabled);
				ui->startZ->setDisabled(enabled);
				ui->endX->setDisabled(enabled);
				ui->endY->setDisabled(enabled);
				ui->endZ->setDisabled(enabled);
				ui->stepsX->setDisabled(enabled);
				ui->stepsY->setDisabled(enabled);
				ui->stepsZ->setDisabled(enabled);
				QMetaObject::invokeMethod(m_Brillouin, "updatePositions", Qt::AutoConnection);
				updateBrillouinSettings();
				updateAbsoluteGridStatus();
				// This redraw still runs against the PRE-toggle m_positionsMicrometer/
				// m_positionsPixel/m_positionsMicrometerIsAbsolute - updatePositions() above is
				// invoked onto m_Brillouin's own thread (queued), so the recomputed positions
				// only arrive later via AOI_changed(), which redraws again once they do.
				update_AOI_preview();
			});

			connect(m_gridHysteresisCompensationCheckbox, &QCheckBox::toggled, this, [this](bool enabled) {
				m_Brillouin->settings.useGridHysteresisCompensation = enabled;
			});

			connect(m_doseProtectionCheckbox, &QCheckBox::toggled, this, [this](bool enabled) {
				m_Brillouin->settings.useDoseProtection = enabled;
			});

			connect(m_saveOverviewBrightfieldPerZCheckbox, &QCheckBox::toggled, this, [this](bool enabled) {
				m_Brillouin->settings.saveOverviewBrightfieldPerZ = enabled;
				updateEstimatedAcquisitionTime();
				updateBrillouinSettings();
				updateOverviewTileOutlines();
			});

			connect(m_overviewSingleImageRadio, &QRadioButton::toggled, this, [this](bool checked) {
				if (!checked) {
					return;
				}
				m_Brillouin->settings.overviewBrightfieldFullGrid = false;
				// Refreshes the full-z-stack checkbox's checked state (see
				// updateBrillouinSettings()) - it's mode-scoped internally (separate
				// single-image/mosaic settings), so switching coverage mode changes
				// which underlying setting it now shows/writes.
				updateBrillouinSettings();
				updateEstimatedAcquisitionTime();
				updateOverviewTileOutlines();
			});

			connect(m_overviewFullGridRadio, &QRadioButton::toggled, this, [this](bool checked) {
				if (!checked) {
					return;
				}
				m_Brillouin->settings.overviewBrightfieldFullGrid = true;
				updateBrillouinSettings();
				updateEstimatedAcquisitionTime();
				updateOverviewTileOutlines();
			});

			connect(m_overviewFullStackCheckbox, &QCheckBox::toggled, this, [this](bool enabled) {
				// Writes into whichever coverage mode is currently active - see
				// overviewFullStackCheckbox's tooltip and updateBrillouinSettings().
				if (m_Brillouin->settings.overviewBrightfieldFullGrid) {
					m_Brillouin->settings.overviewBrightfieldFullStackMosaic = enabled;
				} else {
					m_Brillouin->settings.overviewBrightfieldFullStackSingle = enabled;
				}
				updateEstimatedAcquisitionTime();
			});

			connect(m_capturePerPointBrightfieldCheckbox, &QCheckBox::toggled, this, [this](bool enabled) {
				m_Brillouin->settings.capturePerPointBrightfield = enabled;
				if (m_perPointBrightfieldEveryNSpinBox) {
					m_perPointBrightfieldEveryNSpinBox->setEnabled(enabled);
				}
				if (m_perPointBrightfieldDuringAcquisitionCheckbox) {
					m_perPointBrightfieldDuringAcquisitionCheckbox->setEnabled(enabled);
				}
				updateEstimatedAcquisitionTime();
			});

			connect(m_perPointBrightfieldEveryNSpinBox, qOverload<int>(&QSpinBox::valueChanged), this, [this](int value) {
				m_Brillouin->settings.perPointBrightfieldEveryN = std::max(1, value);
				updateEstimatedAcquisitionTime();
			});

			connect(m_perPointBrightfieldDuringAcquisitionCheckbox, &QCheckBox::toggled, this, [this](bool enabled) {
				m_Brillouin->settings.perPointBrightfieldDuringAcquisition = enabled;
				updateEstimatedAcquisitionTime();
			});

			connect(m_editSpectralProxyRoiCheckbox, &QAbstractButton::toggled, this, [this](bool enabled) {
				if (enabled) {
					ui->customplot->setInteractions(QCP::iNone);
					ui->customplot->setCursor(Qt::CrossCursor);
					ui->statusBar->showMessage("Draw spectral ROI 1, then ROI 2. Right-click clears both ROIs.");
				} else {
					ui->customplot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
					ui->customplot->unsetCursor();
					m_spectralProxyDragActive = false;
					ui->statusBar->clearMessage();
				}
			});
		}
	}

	updateBrillouinSettings();
	updateAbsoluteGridStatus();

	// disable keyboard tracking on stage position input
	// so only complete numbers emit signals
	ui->setPositionX->setKeyboardTracking(false);
	ui->setPositionY->setKeyboardTracking(false);
	ui->setPositionZ->setKeyboardTracking(false);

	ui->parametersWidget->layout()->setAlignment(Qt::AlignTop);

	// hide brightfield preview by default
	ui->brightfieldImage->hide();

	// set up scan direction radio button ids
	ui->buttonGroup->setId(ui->scanDirX0, 0);
	ui->buttonGroup->setId(ui->scanDirX1, 1);
	ui->buttonGroup->setId(ui->scanDirX2, 2);
	ui->buttonGroup_2->setId(ui->scanDirY0, 0);
	ui->buttonGroup_2->setId(ui->scanDirY1, 1);
	ui->buttonGroup_2->setId(ui->scanDirY2, 2);
	ui->buttonGroup_3->setId(ui->scanDirZ0, 0);
	ui->buttonGroup_3->setId(ui->scanDirZ1, 1);
	ui->buttonGroup_3->setId(ui->scanDirZ2, 2);
}

BrillouinAcquisition::~BrillouinAcquisition() {
	writeSettings();
	if (m_acquisition) {
		m_acquisition->deleteLater();
		m_acquisition = nullptr;
	}
	if (m_Brillouin) {
		m_Brillouin->deleteLater();
		m_Brillouin = nullptr;
	}
	if (m_ODT) {
		m_ODT->deleteLater();
		m_ODT = nullptr;
	}
	if (m_converter) {
		m_converter->deleteLater();
		m_converter = nullptr;
	}
	if (tableModel) {
		tableModel->deleteLater();
		tableModel = nullptr;
	}
	if (m_Fluorescence) {
		m_Fluorescence->deleteLater();
		m_Fluorescence = nullptr;
	}
	if (m_scaleCalibration) {
		m_scaleCalibration->deleteLater();
		m_scaleCalibration = nullptr;
	}
	if (m_voltageCalibration) {
		m_voltageCalibration->deleteLater();
		m_voltageCalibration = nullptr;
	}
	if (m_brightfieldCamera) {
		m_brightfieldCamera->deleteLater();
		m_brightfieldCamera = nullptr;
	}
	if (m_scanControl) {
		m_scanControl->deleteLater();
		m_scanControl = nullptr;
	}
	if (m_andor) {
		m_andor->deleteLater();
		m_andor = nullptr;
	}
	m_andorThread.exit();
	m_andorThread.wait();
	m_brightfieldCameraThread.exit();
	m_brightfieldCameraThread.wait();
	m_acquisitionThread.exit();
	m_acquisitionThread.wait();
	m_plottingThread.exit();
	m_plottingThread.wait();
	qInfo(logInfo()) << "BrillouinAcquisition closed.";
	delete ui;
}

void BrillouinAcquisition::closeEvent(QCloseEvent* event) {
	event->ignore();
	if (QMessageBox::Yes == confirmQuit()) {
		event->accept();
	}
}

void BrillouinAcquisition::on_actionQuit_triggered() {
	if (QMessageBox::Yes == confirmQuit()) {
		QApplication::quit();
	}
}

QMessageBox::StandardButton BrillouinAcquisition::confirmQuit() {
	// Directly quit in Debug mode
	#ifdef _DEBUG
		return QMessageBox::Yes;
	#else
		return QMessageBox::question(
			this,
			"Close BrillouinAcquisition?",
			"Do you really want to close BrillouinAcquisition?",
			QMessageBox::Yes | QMessageBox::No
		);
	#endif
}

void BrillouinAcquisition::plotClick(QMouseEvent* event) {
	if (m_scanControl == nullptr) {
		return;
	}

	auto position = event->pos();

	auto posX = m_ODTPlot.plotHandle->xAxis->pixelToCoord(position.x());
	auto posY = m_ODTPlot.plotHandle->yAxis->pixelToCoord(position.y());

	auto positionInPix = POINT2{ posX, posY };
	auto positionInRawPix = brightfieldDisplayToRaw(positionInPix);

	const auto roiEditEnabled = (m_editRoiCheckbox != nullptr && m_editRoiCheckbox->isChecked());
	const auto backgroundRoiEditEnabled = (m_editBackgroundRoiCheckbox != nullptr && m_editBackgroundRoiCheckbox->isChecked());
	const auto modifiers = QApplication::keyboardModifiers();
	if (roiEditEnabled || backgroundRoiEditEnabled || modifiers.testFlag(Qt::ControlModifier)) {
		event->accept();
		// Explicit "Draw background ROI" mode edits the background polygon; anything else
		// (main "Draw ROI" mode, or the Ctrl+click shortcut with neither draw mode active)
		// edits the main ROI - matching the prior Ctrl+click-always-means-main-ROI behavior.
		const auto target = backgroundRoiEditEnabled ? backgroundRoiTarget() : mainRoiTarget();

		auto nearestVertexIndex = [&](const POINT2& pix, double maxDistPix) -> int {
			const auto& poly = *target.polygon;
			if (poly.empty()) {
				return -1;
			}
			int bestIdx = -1;
			double bestDist2 = maxDistPix * maxDistPix;
			for (size_t i = 0; i < poly.size(); ++i) {
				auto pUm = gridOffsetToImagePlaneUm(poly[i]);
				const auto pPix = brightfieldRawToDisplay(m_scanControl->microMeterToPix(pUm));
				const auto dx = pPix.x - pix.x;
				const auto dy = pPix.y - pix.y;
				const auto d2 = dx * dx + dy * dy;
				if (d2 <= bestDist2) {
					bestDist2 = d2;
					bestIdx = (int)i;
				}
			}
			return bestIdx;
		};

		if (event->button() == Qt::RightButton) {
			quickClearRoiPolygonFor(target);
			*target.draggingVertex = false;
			*target.draggedVertexIndex = -1;
			updateRoiPolygonPreviewFor(target);
			updateBrillouinSettings();
			return;
		}

		if (event->button() == Qt::LeftButton) {
			const int dragged = nearestVertexIndex(positionInPix, 8.0);
			if (dragged >= 0) {
				*target.draggingVertex = true;
				*target.draggedVertexIndex = dragged;
				return;
			}

			auto positionInUm = imagePlaneUmToGridOffset(m_scanControl->pixToMicroMeter(positionInRawPix));
			addRoiPolygonPointFor(target, positionInUm);
			updateRoiPolygonPreviewFor(target);
			QMetaObject::invokeMethod(m_Brillouin, "updatePositions", Qt::AutoConnection);
			updateBrillouinSettings();
			return;
		}
	}

	// If we currently select the new focus, don't move there
	if (m_locatePositionScanner) {
		m_scanControl->locatePositionScanner(positionInRawPix);
		// Confirmed - disarm immediately so the button reverts to idle and the next click
		// resumes normal click-to-move, instead of relocating the marker again.
		setLaserPositionLocationArmed(false);
	} else if (m_relocatePositionScanner) {
		relocateBeamKeepingGridFixed(positionInRawPix);
		setRelocateFocusMarkerArmed(false);
	} else {
		auto xRange = m_ODTPlot.plotHandle->xAxis->range();
		auto yRange = m_ODTPlot.plotHandle->yAxis->range();

		if (!xRange.contains(posX) || !yRange.contains(posY)) {
			return;
		}

		// Set laser focus to this position
		QMetaObject::invokeMethod(
			m_scanControl,
			[&m_scanControl = m_scanControl, positionInRawPix]() {
				m_scanControl->setPositionInPix(positionInRawPix);
			},
			Qt::QueuedConnection
		);
	}
}

void BrillouinAcquisition::showEvent(QShowEvent* event) {
	QWidget::showEvent(event);

	// connect microscope automatically
	QMetaObject::invokeMethod(
		m_scanControl,
		[&m_scanControl = m_scanControl]() {
			m_scanControl->connectDevice();
		},
		Qt::QueuedConnection
	);
}

void BrillouinAcquisition::setElement(DeviceElement element, double position) {
	QMetaObject::invokeMethod(
		m_scanControl,
		[&m_scanControl = m_scanControl, element, position]() {
			m_scanControl->setElement(element, position);
		},
		Qt::QueuedConnection
	);
}

void BrillouinAcquisition::on_autoscalePlot_stateChanged(int state) {
	m_BrillouinPlot.autoscale = (bool)state;
	ui->rangeLower->setDisabled(state);
	ui->rangeUpper->setDisabled(state);
}

void BrillouinAcquisition::on_autoscalePlot_brightfield_stateChanged(int state) {
	m_ODTPlot.autoscale = (bool)state;
	ui->rangeLowerODT->setDisabled(state);
	ui->rangeUpperODT->setDisabled(state);
}

void BrillouinAcquisition::setPreset(ScanPreset preset) {
	QMetaObject::invokeMethod(
		m_scanControl,
		[&m_scanControl = m_scanControl, preset]() {
			m_scanControl->setPreset(preset);
		},
		Qt::QueuedConnection);
}

void BrillouinAcquisition::cameraOptionsChanged(const CAMERA_OPTIONS& options) {
	m_cameraOptions.ROIHeightLimits = options.ROIHeightLimits;
	m_cameraOptions.ROIWidthLimits = options.ROIWidthLimits;

	// set size of colormap to maximum image size
	m_BrillouinPlot.colorMap->data()->setSize(options.ROIWidthLimits[1], options.ROIHeightLimits[1]);

	addListToComboBox(ui->triggerMode, options.triggerModes);
	addListToComboBox(ui->binning, options.imageBinnings);
	addListToComboBox(ui->pixelReadoutRate, options.pixelReadoutRates);
	addListToComboBox(ui->cycleMode, options.cycleModes);
	addListToComboBox(ui->preAmpGain, options.preAmpGains);
	addListToComboBox(ui->pixelEncoding, options.pixelEncodings);

	ui->exposureTime->setMinimum(options.exposureTimeLimits[0]);
	ui->exposureTime->setMaximum(options.exposureTimeLimits[1]);
	ui->frameCount->setMinimum(options.frameCountLimits[0]);
	ui->frameCount->setMaximum(options.frameCountLimits[1]);

	ui->ROIHeight->setMinimum(options.ROIHeightLimits[0]);
	ui->ROIHeight->setMaximum(options.ROIHeightLimits[1]);
	ui->ROIHeight->setValue(options.ROIHeightLimits[1]);
	ui->ROITop->setMinimum(1);
	ui->ROITop->setMaximum(options.ROIHeightLimits[1]);
	ui->ROITop->setValue(1);
	ui->ROIWidth->setMinimum(options.ROIWidthLimits[0]);
	ui->ROIWidth->setMaximum(options.ROIWidthLimits[1]);
	ui->ROIWidth->setValue(options.ROIWidthLimits[1]);
	ui->ROILeft->setMinimum(1);
	ui->ROILeft->setMaximum(options.ROIWidthLimits[1]);
	ui->ROILeft->setValue(1);
}

void BrillouinAcquisition::cameraODTOptionsChanged(const CAMERA_OPTIONS& options) {
	m_cameraOptionsODT.exposureTimeLimits = options.exposureTimeLimits;

	m_cameraOptionsODT.ROIHeightLimits = options.ROIHeightLimits;
	m_cameraOptionsODT.ROIWidthLimits = options.ROIWidthLimits;

	// Adjust plotting range only when neither preview nor acquisition are running
	if (!(m_brightfieldCamera->m_isPreviewRunning || m_brightfieldCamera->m_isAcquisitionRunning)) {
		m_brightfieldRawWidth = std::max(1, (int)options.ROIWidthLimits[1]);
		m_brightfieldRawHeight = std::max(1, (int)options.ROIHeightLimits[1]);
		m_ODTPlot.plotHandle->xAxis->setRange(QCPRange(1, brightfieldDisplayWidth()));
		m_ODTPlot.plotHandle->yAxis->setRange(QCPRange(1, brightfieldDisplayHeight()));

		ui->ROIHeightODT->setValue(options.ROIHeightLimits[1]);
		ui->ROITopODT->setValue(0);
		ui->ROIWidthODT->setValue(options.ROIWidthLimits[1]);
		ui->ROILeftODT->setValue(0);

	}

	ui->ROIHeightODT->setMinimum(options.ROIHeightLimits[0]);
	ui->ROIHeightODT->setMaximum(options.ROIHeightLimits[1]);
	ui->ROITopODT->setMinimum(0);
	ui->ROITopODT->setMaximum(options.ROIHeightLimits[1]);
	ui->ROIWidthODT->setMinimum(options.ROIWidthLimits[0]);
	ui->ROIWidthODT->setMaximum(options.ROIWidthLimits[1]);
	ui->ROILeftODT->setMinimum(0);
	ui->ROILeftODT->setMaximum(options.ROIWidthLimits[1]);

	// block signals to not trigger setting a new value
	const QSignalBlocker blocker(ui->exposureTimeODT);
	ui->exposureTimeODT->setMinimum(m_cameraOptionsODT.exposureTimeLimits[0]);
	ui->exposureTimeODT->setMaximum(m_cameraOptionsODT.exposureTimeLimits[1]);

	ui->fluoBlueExposure->setMinimum(1e3*m_cameraOptionsODT.exposureTimeLimits[0]);
	ui->fluoBlueExposure->setMaximum(1e3*m_cameraOptionsODT.exposureTimeLimits[1]);
	ui->fluoGreenExposure->setMinimum(1e3*m_cameraOptionsODT.exposureTimeLimits[0]);
	ui->fluoGreenExposure->setMaximum(1e3*m_cameraOptionsODT.exposureTimeLimits[1]);
	ui->fluoRedExposure->setMinimum(1e3*m_cameraOptionsODT.exposureTimeLimits[0]);
	ui->fluoRedExposure->setMaximum(1e3*m_cameraOptionsODT.exposureTimeLimits[1]);
	ui->fluoBrightfieldExposure->setMinimum(1e3*m_cameraOptionsODT.exposureTimeLimits[0]);
	ui->fluoBrightfieldExposure->setMaximum(1e3*m_cameraOptionsODT.exposureTimeLimits[1]);

	addListToComboBox(ui->pixelEncodingODT, options.pixelEncodings);
}

void BrillouinAcquisition::showAcqPosition(POINT3 position, int imageNr) {
	showPosition(position);
	ui->imageNr->setText(QString::number(imageNr));
}

void BrillouinAcquisition::updateEstimatedAcquisitionTime() {
	const auto pointCount = m_positionsComputed
		? m_positionsMicrometer.size()
		: (size_t)std::max(1, m_Brillouin->settings.xSteps)
			* (size_t)std::max(1, m_Brillouin->settings.ySteps)
			* (size_t)std::max(1, m_Brillouin->settings.zSteps);
	const auto frameCount = std::max<int64_t>(1, m_Brillouin->settings.camera.frameCount);
	const auto exposureSeconds = std::max(0.0, m_Brillouin->settings.camera.exposureTime);
	const auto exposureOnlySeconds = exposureSeconds * frameCount * (double)pointCount;

	// Movement estimate: total travel distance along the actual planned path (only known
	// once m_positionsMicrometer is populated - falls back to 0 extra time before that,
	// same as the exposure-only estimate already did) divided by an assumed stage speed,
	// since no hardware-reported speed exists anywhere in ScanControl to query instead.
	// kAssumedStageSpeedUmPerS is a rough approximation, not a calibrated value - adjust it
	// here if it's consistently far off for your hardware.
	constexpr double kAssumedStageSpeedUmPerS = 1000.0;
	double totalTravelUm = 0.0;
	for (size_t i = 1; i < m_positionsMicrometer.size(); i++) {
		const auto delta = m_positionsMicrometer[i] - m_positionsMicrometer[i - 1];
		totalTravelUm += std::sqrt(delta.x * delta.x + delta.y * delta.y + delta.z * delta.z);
	}
	const size_t moveCount = pointCount > 0 ? pointCount - 1 : 0;
	double moveSeconds = totalTravelUm / kAssumedStageSpeedUmPerS;
	// Compensated moves add an extra pre-approach move and a settle delay whenever xy
	// changes (see Brillouin::approachGridPosition()/ScanControl::setPositionCompensated())
	// - approximated here as a fixed overhead per move rather than tracking which moves
	// actually change xy, since this is an estimate, not an exact replay of the scan.
	if (m_Brillouin->settings.useGridHysteresisCompensation) {
		moveSeconds += 0.1 * (double)moveCount;
	}

	// BF overview estimate: one exposure per captured image, plus a fixed per-image
	// overhead for the preset switch/settle moves captureOverviewBrightfield() actually
	// does (see its own 100 ms post-move sleep).
	const auto overviewImageCount = m_Brillouin->overviewImageCountTotal();
	const auto overviewExposureSeconds = 1e-3 * std::max(1, m_Brillouin->settings.overviewBrightfieldExposureMs);
	constexpr double kOverviewPerImageOverheadS = 0.1;
	const double overviewSeconds = (double)overviewImageCount * (overviewExposureSeconds + kOverviewPerImageOverheadS);

	const auto totalSeconds = (int)std::ceil(exposureOnlySeconds + moveSeconds + overviewSeconds);
	ui->estimatedAcquisitionTime->setText(formatSeconds(totalSeconds));
	ui->estimatedAcquisitionTime->setToolTip(
		QString("%1 points x %2 frames x %3 s exposure (%4) + ~%5 stage movement (assumes %6 um/s) + %7 BF overview images (%8).")
		.arg((qulonglong)pointCount)
		.arg((qlonglong)frameCount)
		.arg(exposureSeconds, 0, 'g', 4)
		.arg(formatSeconds((int)std::ceil(exposureOnlySeconds)))
		.arg(formatSeconds((int)std::ceil(moveSeconds)))
		.arg(kAssumedStageSpeedUmPerS, 0, 'g', 4)
		.arg(overviewImageCount)
		.arg(formatSeconds((int)std::ceil(overviewSeconds)))
	);
}

QCPItemRect* BrillouinAcquisition::ensureSpectralProxyRoiRect(int index) {
	auto** rectItem = index == 1 ? &m_spectralProxyRoi2RectItem : &m_spectralProxyRoiRectItem;
	if (!*rectItem) {
		*rectItem = new QCPItemRect(ui->customplot);
		QPen pen(index == 1 ? QColor(0, 200, 255) : QColor(255, 215, 0));
		pen.setWidth(2);
		(*rectItem)->setPen(pen);
	}
	return *rectItem;
}

CAMERA_ROI BrillouinAcquisition::currentSpectralCameraRoi() const {
	// See this function's own declaration for why the source switches here rather than always
	// reading one or the other.
	if (m_Brillouin && m_Brillouin->getStatus() > ACQUISITION_STATUS::STOPPED) {
		return m_Brillouin->settings.camera.roi;
	}
	return m_deviceSettings.camera.roi;
}

void BrillouinAcquisition::updateSpectralProxyRoiRect(int index) {
	const auto& settings = m_Brillouin->settings;
	const auto rawLeft = index == 1 ? settings.surfaceProxyRoi2Left : settings.surfaceProxyRoiLeft;
	const auto rawTop = index == 1 ? settings.surfaceProxyRoi2Top : settings.surfaceProxyRoiTop;
	const auto rawWidth = index == 1 ? settings.surfaceProxyRoi2Width : settings.surfaceProxyRoiWidth;
	const auto rawHeight = index == 1 ? settings.surfaceProxyRoi2Height : settings.surfaceProxyRoiHeight;
	const PROXY_ROI_FRAME drawnFrame = index == 1 ?
		PROXY_ROI_FRAME{
			settings.surfaceProxyRoi2FrameWidth, settings.surfaceProxyRoi2FrameHeight,
			settings.surfaceProxyRoi2FrameOriginLeft, settings.surfaceProxyRoi2FrameOriginBottom,
			settings.surfaceProxyRoi2FrameWidthPhysical, settings.surfaceProxyRoi2FrameHeightPhysical
		} :
		PROXY_ROI_FRAME{
			settings.surfaceProxyRoiFrameWidth, settings.surfaceProxyRoiFrameHeight,
			settings.surfaceProxyRoiFrameOriginLeft, settings.surfaceProxyRoiFrameOriginBottom,
			settings.surfaceProxyRoiFrameWidthPhysical, settings.surfaceProxyRoiFrameHeightPhysical
		};
	auto** rectItem = index == 1 ? &m_spectralProxyRoi2RectItem : &m_spectralProxyRoiRectItem;

	if (rawWidth <= 0 || rawHeight <= 0) {
		if (*rectItem) {
			ui->customplot->removeItem(*rectItem);
			*rectItem = nullptr;
		}
		return;
	}

	auto* rect = ensureSpectralProxyRoiRect(index);
	const auto liveRoi = currentSpectralCameraRoi();
	auto* mapData = m_BrillouinPlot.colorMap ? m_BrillouinPlot.colorMap->data() : nullptr;
	const int frameW = mapData ? std::max(1, mapData->keySize()) : std::max(1, (int)liveRoi.width_binned);
	const int frameH = mapData ? std::max(1, mapData->valueSize()) : std::max(1, (int)liveRoi.height_binned);
	// Remap onto the currently displayed frame if it differs from whatever frame this ROI
	// was drawn against, so the visible rectangle never silently drifts off-screen, shrinks
	// to nothing, or lands on the wrong physical location after a camera ROI/binning change
	// - see Brillouin::remapProxyRoi() for why a size-only rescale isn't enough, and
	// estimateFrameMetric() for why the actual measurement does the same remap.
	const PROXY_ROI_FRAME currentFrame{
		frameW, frameH,
		liveRoi.left, liveRoi.bottom,
		liveRoi.width_physical, liveRoi.height_physical
	};
	int left{ rawLeft }, top{ rawTop }, width{ rawWidth }, height{ rawHeight };
	Brillouin::remapProxyRoi(rawLeft, rawTop, rawWidth, rawHeight, drawnFrame, currentFrame,
		left, top, width, height);
	const int displayLeft = std::clamp(left, 0, frameW - 1);
	const int displayRight = std::clamp(left + width - 1, displayLeft, frameW - 1);
	const int displayBottom = std::clamp(top, 0, frameH - 1);
	const int displayTop = std::clamp(top + height - 1, displayBottom, frameH - 1);
	if (mapData) {
		double xLeft{ 0.0 };
		double yTop{ 0.0 };
		double xRight{ 0.0 };
		double yBottom{ 0.0 };
		mapData->cellToCoord(displayLeft, displayTop, &xLeft, &yTop);
		mapData->cellToCoord(displayRight, displayBottom, &xRight, &yBottom);
		rect->topLeft->setCoords(xLeft, yTop);
		rect->bottomRight->setCoords(xRight, yBottom);
	} else {
		rect->topLeft->setCoords(displayLeft + 1, displayTop + 1);
		rect->bottomRight->setCoords(displayRight + 1, displayBottom + 1);
	}
}

void BrillouinAcquisition::refreshSpectralProxyRoiRects() {
	if (m_spectralProxyRoiRectItem || m_Brillouin->settings.surfaceProxyRoiWidth > 0) {
		updateSpectralProxyRoiRect(0);
	}
	if (m_spectralProxyRoi2RectItem || m_Brillouin->settings.surfaceProxyRoi2Width > 0) {
		updateSpectralProxyRoiRect(1);
	}
}

void BrillouinAcquisition::clearSpectralProxyRois() {
	m_Brillouin->settings.surfaceProxyRoiLeft = 0;
	m_Brillouin->settings.surfaceProxyRoiTop = 0;
	m_Brillouin->settings.surfaceProxyRoiWidth = 0;
	m_Brillouin->settings.surfaceProxyRoiHeight = 0;
	m_Brillouin->settings.surfaceProxyRoi2Left = 0;
	m_Brillouin->settings.surfaceProxyRoi2Top = 0;
	m_Brillouin->settings.surfaceProxyRoi2Width = 0;
	m_Brillouin->settings.surfaceProxyRoi2Height = 0;
	m_Brillouin->settings.surfaceProxyRoiFrameWidth = 0;
	m_Brillouin->settings.surfaceProxyRoiFrameHeight = 0;
	m_Brillouin->settings.surfaceProxyRoiFrameOriginLeft = 0;
	m_Brillouin->settings.surfaceProxyRoiFrameOriginBottom = 0;
	m_Brillouin->settings.surfaceProxyRoiFrameWidthPhysical = 0;
	m_Brillouin->settings.surfaceProxyRoiFrameHeightPhysical = 0;
	m_Brillouin->settings.surfaceProxyRoi2FrameWidth = 0;
	m_Brillouin->settings.surfaceProxyRoi2FrameHeight = 0;
	m_Brillouin->settings.surfaceProxyRoi2FrameOriginLeft = 0;
	m_Brillouin->settings.surfaceProxyRoi2FrameOriginBottom = 0;
	m_Brillouin->settings.surfaceProxyRoi2FrameWidthPhysical = 0;
	m_Brillouin->settings.surfaceProxyRoi2FrameHeightPhysical = 0;
	updateSpectralProxyRoiRect(0);
	updateSpectralProxyRoiRect(1);
	m_spectralProxyNextRoiIndex = 0;
	ui->customplot->replot();
}

void BrillouinAcquisition::updateBrillouinStartAvailability() {
	const auto odtRunning = (bool)(m_enabledModes & ACQUISITION_MODE::ODT);
	ui->BrillouinStart->setEnabled(!odtRunning);
	ui->BrillouinStart->setToolTip(QString{});
}

void BrillouinAcquisition::showPosition(POINT3 position) {
	ui->positionX->setText(QString::number(position.x));
	ui->positionY->setText(QString::number(position.y));
	ui->positionZ->setText(QString::number(position.z));
	if (!ui->setPositionX->hasFocus()) {
		const QSignalBlocker blocker(ui->setPositionX);
		ui->setPositionX->setValue(position.x);
	}
	if (!ui->setPositionY->hasFocus()) {
		const QSignalBlocker blocker(ui->setPositionY);
		ui->setPositionY->setValue(position.y);
	}
	if (!ui->setPositionZ->hasFocus()) {
		const QSignalBlocker blocker(ui->setPositionZ);
		ui->setPositionZ->setValue(position.z);
	}
	updateAbsoluteGridStatus();
}

POINT3 BrillouinAcquisition::gridOffsetToAbsoluteTarget(const POINT3& gridOffset, const POINT3& relativeOrigin) const {
	// Goes through Brillouin::resolvedGridOriginUm(), not settings.absoluteGridOriginUm
	// directly, so the on-screen grid/ROI overlay stays glued to where a measurement will
	// actually execute even when the active objective has a calibrated FOV-center offset -
	// an independent re-derivation here could drift from the AOI markers.
	const auto origin = m_Brillouin->settings.gridCoordinatesAbsolute
		? m_Brillouin->resolvedGridOriginUm()
		: relativeOrigin;
	return POINT3{
		origin.x + gridOffset.x,
		origin.y + gridOffset.y,
		origin.z + gridOffset.z
	};
}

POINT3 BrillouinAcquisition::absoluteTargetToGridOffset(const POINT3& absoluteTarget, const POINT3& relativeOrigin) const {
	const auto origin = m_Brillouin->settings.gridCoordinatesAbsolute
		? m_Brillouin->resolvedGridOriginUm()
		: relativeOrigin;
	return POINT3{
		absoluteTarget.x - origin.x,
		absoluteTarget.y - origin.y,
		absoluteTarget.z - origin.z
	};
}

POINT2 BrillouinAcquisition::currentGridOffset(bool gridAbsolute) const {
	if (gridAbsolute == m_currentGridOffsetIsAbsolute) {
		return m_currentGridOffsetUm;
	}
	// Cached snapshot is for the other mode (or none has arrived yet) - fall back to a live
	// fetch. The ordinary overlay redraw path (AOI_changed()) refreshes this cache itself for
	// whichever mode it just computed positions under, so this fallback is only reachable from
	// preservePhysicalGridForAbsoluteMode()'s round-trip through the mode being switched away
	// from.
	if (!m_scanControl) {
		return POINT2{};
	}
	return m_scanControl->getPositionOffset(gridAbsolute);
}

POINT2 BrillouinAcquisition::imagePlaneUmToGridOffset(const POINT2& imagePlaneUm) const {
	return imagePlaneUmToGridOffset(imagePlaneUm, m_Brillouin->settings.gridCoordinatesAbsolute);
}

// gridAbsolute is taken explicitly (rather than always read from the current settings) so
// preservePhysicalGridForAbsoluteMode() can convert through the OLD mode's convention and
// back through the NEW one when the grid-coordinates-absolute setting itself is what's
// changing - it must not silently use "current settings" for both directions of that
// conversion, or the two conversions would disagree with each other.
POINT2 BrillouinAcquisition::imagePlaneUmToGridOffset(const POINT2& imagePlaneUm, bool gridAbsolute) const {
	if (!m_scanControl) {
		return imagePlaneUm;
	}
	// roiPolygonUm is stored in the exact same grid-offset frame as the measurement grid
	// itself (ScanPlanner tests it directly against the pre-origin grid position, see
	// ScanPlanner::isPointInPolygon()). The AOI markers reach that same frame via
	// ScanControl::getPositionOffset() (absolute mode also adds absoluteGridOriginUm first,
	// since ScanPlanner's absolute positions have the origin baked in) - reusing that exact
	// conversion, instead of re-deriving it here, is what keeps the ROI overlay glued to the
	// markers in every grid mode.
	const auto origin = gridAbsolute ? m_Brillouin->resolvedGridOriginUm() : POINT3{};
	const auto offset = currentGridOffset(gridAbsolute);
	return POINT2{
		imagePlaneUm.x - offset.x - origin.x,
		imagePlaneUm.y - offset.y - origin.y
	};
}

POINT2 BrillouinAcquisition::gridOffsetToImagePlaneUm(const POINT2& gridOffset) const {
	return gridOffsetToImagePlaneUm(gridOffset, m_Brillouin->settings.gridCoordinatesAbsolute);
}

POINT2 BrillouinAcquisition::gridOffsetToImagePlaneUm(const POINT2& gridOffset, bool gridAbsolute) const {
	if (!m_scanControl) {
		return gridOffset;
	}
	// See imagePlaneUmToGridOffset() for why this must match ScanControl's own convention.
	const auto origin = gridAbsolute ? m_Brillouin->resolvedGridOriginUm() : POINT3{};
	const auto offset = currentGridOffset(gridAbsolute);
	return POINT2{
		origin.x + gridOffset.x + offset.x,
		origin.y + gridOffset.y + offset.y
	};
}

void BrillouinAcquisition::preservePhysicalGridForAbsoluteMode(bool enabled) {
	if (!m_scanControl || enabled == m_Brillouin->settings.gridCoordinatesAbsolute) {
		return;
	}

	const auto oldAbsoluteMode = m_Brillouin->settings.gridCoordinatesAbsolute;
	if (enabled) {
		m_Brillouin->settings.absoluteGridOriginUm = m_scanControl->getHomePosition();
	}

	// X/Y only - "absolute grid coordinates" only ever meant x/y anchored to a fixed physical
	// point, independent of wherever Start happens to be pressed. Z is deliberately NOT part of
	// that split (see Brillouin::resolvedGridOriginUm()'s own comment): it always follows
	// m_startPosition.z, refreshed at every Start or on demand via "Set plane"
	// (setCurrentFocusAsZOrigin()), regardless of gridCoordinatesAbsolute. zMin/zMax are
	// therefore left untouched by a mode toggle - the operator's configured z sweep survives
	// switching to/from absolute mode instead of being silently reinterpreted.
	//
	// Round-trip each stored x/y point through gridOffsetToImagePlaneUm()/
	// imagePlaneUmToGridOffset() - the exact functions the grid and the ROI polygon are
	// actually drawn with - instead of re-deriving the offset a third time. That guarantees
	// whatever currently renders on screen is preserved exactly, in both directions, because
	// switching modes can no longer disagree with what put it there in the first place.
	auto convertXY = [&](const POINT2& gridOffset) {
		const auto imagePlaneUm = gridOffsetToImagePlaneUm(gridOffset, oldAbsoluteMode);
		return imagePlaneUmToGridOffset(imagePlaneUm, enabled);
	};

	const auto newMinXY = convertXY(POINT2{ m_Brillouin->settings.xMin, m_Brillouin->settings.yMin });
	const auto newMaxXY = convertXY(POINT2{ m_Brillouin->settings.xMax, m_Brillouin->settings.yMax });

	m_Brillouin->settings.setXMin(newMinXY.x);
	m_Brillouin->settings.setXMax(newMaxXY.x);
	m_Brillouin->settings.setYMin(newMinXY.y);
	m_Brillouin->settings.setYMax(newMaxXY.y);

	for (auto& point : m_Brillouin->settings.roiPolygonUm) {
		point = convertXY(point);
	}
}

void BrillouinAcquisition::updateAbsoluteGridStatus() {
	if (!ui->absoluteGridStatusLabel) {
		return;
	}
	// Resolved (offset-applied), not the raw stored setting, so this status readout is
	// directly comparable to the live Stage/Focus positions shown alongside it.
	const auto origin = m_Brillouin->resolvedGridOriginUm();
	const auto currentFocus = m_scanControl ? m_scanControl->getPosition() : POINT3{};
	const auto currentStage = m_scanControl ? m_scanControl->getPosition(PositionType::STAGE) : POINT3{};
	const auto mode = m_Brillouin->settings.gridCoordinatesAbsolute
		? QString("absolute, grid relative to origin")
		: QString("relative, grid relative to acquisition start");

	// Size/spacing from the stored xMin/xMax/xSteps etc. directly - these are already offsets
	// from the origin in both modes (see preservePhysicalGridForAbsoluteMode()), so their
	// difference/step is the same physical extent independent of gridCoordinatesAbsolute or
	// where the origin happens to be. Shown here so "how big is this grid" never requires
	// reading xMin/xMax in absolute mode and mentally subtracting - which is also why those
	// fields are locked to read-only while in absolute mode (see the ACQUISITION_STATUS
	// handler and the m_absoluteGridCheckbox toggle handler).
	const auto& settings = m_Brillouin->settings;
	auto spacing = [](double lo, double hi, int steps) {
		return steps > 1 ? (hi - lo) / (steps - 1) : 0.0;
	};
	const auto sizeX = settings.xMax - settings.xMin;
	const auto sizeY = settings.yMax - settings.yMin;
	const auto sizeZ = settings.zMax - settings.zMin;
	const auto spacingX = spacing(settings.xMin, settings.xMax, settings.xSteps);
	const auto spacingY = spacing(settings.yMin, settings.yMax, settings.ySteps);
	const auto spacingZ = spacing(settings.zMin, settings.zMax, settings.zSteps);

	ui->absoluteGridStatusLabel->setText(
		QString("Grid: %1\nSize: X %2 µm, Y %3 µm, Z %4 µm | Spacing: X %5 µm, Y %6 µm, Z %7 µm\n"
			"Origin: X %8, Y %9, Z %10\nStage: X %11, Y %12, Z %13 | Focus: X %14, Y %15, Z %16")
		.arg(mode)
		.arg(sizeX, 0, 'f', 2)
		.arg(sizeY, 0, 'f', 2)
		.arg(sizeZ, 0, 'f', 2)
		.arg(spacingX, 0, 'f', 2)
		.arg(spacingY, 0, 'f', 2)
		.arg(spacingZ, 0, 'f', 2)
		.arg(origin.x, 0, 'f', 2)
		.arg(origin.y, 0, 'f', 2)
		.arg(origin.z, 0, 'f', 2)
		.arg(currentStage.x, 0, 'f', 2)
		.arg(currentStage.y, 0, 'f', 2)
		.arg(currentStage.z, 0, 'f', 2)
		.arg(currentFocus.x, 0, 'f', 2)
		.arg(currentFocus.y, 0, 'f', 2)
		.arg(currentFocus.z, 0, 'f', 2));
}

void BrillouinAcquisition::setHomePositionBounds(BOUNDS bounds) {
	// set limits on manual stage control
	ui->setPositionX->setMinimum(bounds.xMin);
	ui->setPositionX->setMaximum(bounds.xMax);
	ui->setPositionY->setMinimum(bounds.yMin);
	ui->setPositionY->setMaximum(bounds.yMax);
	ui->setPositionZ->setMinimum(bounds.zMin);
	ui->setPositionZ->setMaximum(bounds.zMax);
}

void BrillouinAcquisition::setCurrentPositionBounds(BOUNDS bounds) {
	// set limits on AOI control
	//x
	ui->startX->setMinimum(bounds.xMin);
	ui->startX->setMaximum(bounds.xMax);
	ui->endX->setMinimum(bounds.xMin);
	ui->endX->setMaximum(bounds.xMax);

	//y
	ui->startY->setMinimum(bounds.yMin);
	ui->startY->setMaximum(bounds.yMax);
	ui->endY->setMinimum(bounds.yMin);
	ui->endY->setMaximum(bounds.yMax);

	//z
	ui->startZ->setMinimum(bounds.zMin);
	ui->startZ->setMaximum(bounds.zMax);
	ui->endZ->setMinimum(bounds.zMin);
	ui->endZ->setMaximum(bounds.zMax);
}

void BrillouinAcquisition::showCalibrationInterval(int value) {
	ui->calibrationProgress->setValue(value);
	ui->calibrationProgress->setFormat("Time to next calibration.");
}

void BrillouinAcquisition::showCalibrationRunning(bool isCalibrating) {
	if (isCalibrating) {
		ui->calibrationProgress->setValue(100);
		ui->calibrationProgress->setFormat("Acquiring calibration.");
	}
}

void BrillouinAcquisition::addListToComboBox(QComboBox* box, const std::vector<std::wstring>& list) {
	// Check whether we have to do anything
	std::vector< std::wstring > currentList(box->count());
	for (gsl::index i{ 0 }; i < currentList.size(); i++) {
		currentList[i] = box->itemText(i).toStdWString();
	}
	if (currentList == list) {
		return;
	}
	const QSignalBlocker blocker(box);
	box->clear();
	for (auto &item : list) {
		box->addItem(QString::fromStdWString(item));
	}
}

void BrillouinAcquisition::cameraSettingsChanged(const CAMERA_SETTINGS& settings) {
	// The driver-reported ROI is the only place .bottom/.right/.width_binned/.height_binned
	// (as opposed to .left/.top/.width_physical/.height_physical, which the live crop UI's own
	// value-changed handlers already push into m_deviceSettings.camera.roi via
	// settingsCameraUpdate()) ever get computed correctly - see andor.cpp's ROI application.
	// Without this, those fields stayed frozen at CAMERA_ROI's defaults forever, silently
	// breaking anything that needs the camera frame's real vertical/right origin, e.g.
	// remapProxyRoi()'s "drawnFrame" vs "currentFrame" comparison for the spectral proxy ROI.
	m_deviceSettings.camera.roi = settings.roi;

	ui->exposureTime->setValue(settings.exposureTime);
	ui->frameCount->setValue(settings.frameCount);
	//ui->ROILeft->setValue(settings.roi.left);
	//ui->ROIWidth->setValue(settings.roi.width);
	//ui->ROITop->setValue(settings.roi.top);
	//ui->ROIHeight->setValue(settings.roi.height);

	ui->triggerMode->setCurrentText(QString::fromStdWString(settings.readout.triggerMode));
	ui->binning->setCurrentText(QString::fromStdWString(settings.roi.binning));

	ui->pixelReadoutRate->setCurrentText(QString::fromStdWString(settings.readout.pixelReadoutRate));
	ui->cycleMode->setCurrentText(QString::fromStdWString(settings.readout.cycleMode));
	ui->preAmpGain->setCurrentText(QString::fromStdWString(settings.readout.preAmpGain));
	ui->pixelEncoding->setCurrentText(QString::fromStdWString(settings.readout.pixelEncoding));
}

void BrillouinAcquisition::cameraODTSettingsChanged(const CAMERA_SETTINGS& settings) {
	const QSignalBlocker blocker1(ui->exposureTimeODT);
	ui->exposureTimeODT->setValue(settings.exposureTime);

	const QSignalBlocker blocker2(ui->gainODT);
	ui->gainODT->setValue(settings.gain);

	//ui->frameCount->setValue(settings.frameCount);
	ui->ROILeftODT->setValue(settings.roi.left);
	ui->ROIWidthODT->setValue(settings.roi.width_physical);
	ui->ROITopODT->setValue(settings.roi.top);
	ui->ROIHeightODT->setValue(settings.roi.height_physical);

	ui->pixelEncodingODT->setCurrentText(QString::fromStdWString(settings.readout.pixelEncoding));
}

void BrillouinAcquisition::updateODTCameraSettings(const CAMERA_SETTINGS& settings) {
	const QSignalBlocker blocker1(ui->exposureTimeCameraODT);
	ui->exposureTimeCameraODT->setValue(settings.exposureTime);

	const QSignalBlocker blocker2(ui->gainCameraODT);
	ui->gainCameraODT->setValue(settings.gain);
}

void BrillouinAcquisition::sensorTemperatureChanged(const SensorTemperature& sensorTemperature) {
	ui->sensorTemp->setValue(sensorTemperature.temperature);
	if (sensorTemperature.status == enCameraTemperatureStatus::COOLER_OFF ||
		sensorTemperature.status == enCameraTemperatureStatus::FAULT || sensorTemperature.status == enCameraTemperatureStatus::DRIFT) {
		ui->settingsWidget->setTabIcon(0, m_icons.standby);
	} else if (sensorTemperature.status == enCameraTemperatureStatus::COOLING
		|| sensorTemperature.status == enCameraTemperatureStatus::NOT_STABILISED) {
		ui->settingsWidget->setTabIcon(0, m_icons.cooling);
	} else if (sensorTemperature.status == enCameraTemperatureStatus::STABILISED) {
		ui->settingsWidget->setTabIcon(0, m_icons.ready);
	} else {
		ui->settingsWidget->setTabIcon(0, m_icons.disconnected);
	}
}

void BrillouinAcquisition::initializeODTVoltagePlot(QCustomPlot *plot) {
	plot->setBackground(QColor(240, 240, 240, 255));
	plot->axisRect()->setBackground(Qt::white);
	//plot->xAxis->setLabel("Ux [V]");
	//plot->yAxis->setLabel("Uy [V]");

	plot->axisRect()->setupFullAxesBox(true);

	plot->axisRect()->setMaximumSize(210, 210); // make bottom right axis rect size fixed 150x150
	plot->axisRect()->setMinimumSize(210, 210);

	plot->addGraph();
	plot->graph(0)->setLineStyle(QCPGraph::LineStyle::lsNone);
	plot->graph(0)->setScatterStyle(QCPScatterStyle::ScatterShape::ssCircle);
	plot->addGraph();
	plot->graph(1)->setLineStyle(QCPGraph::LineStyle::lsNone);
	plot->graph(1)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssDisc, Qt::red, 10));
	plot->replot();
}

/*
* ODT signals
*/

void BrillouinAcquisition::on_alignmentUR_ODT_valueChanged(double voltage) {
	m_ODT->setSettings(ODT_MODE::ALGN, ODT_SETTING::VOLTAGE, voltage);
}

void BrillouinAcquisition::on_alignmentNumber_ODT_valueChanged(int number) {
	QMetaObject::invokeMethod(
		m_ODT,
		[&m_ODT = m_ODT, number]() {
			m_ODT->setSettings(ODT_MODE::ALGN, ODT_SETTING::NRPOINTS, (double)number);
		},
		Qt::AutoConnection
	);
}

void BrillouinAcquisition::on_alignmentRate_ODT_valueChanged(double rate) {
	QMetaObject::invokeMethod(
		m_ODT,
		[&m_ODT = m_ODT, rate]() {
			m_ODT->setSettings(ODT_MODE::ALGN, ODT_SETTING::SCANRATE, rate);
		},
		Qt::AutoConnection
	);
}

void BrillouinAcquisition::on_alignmentStartODT_clicked() {
	QMetaObject::invokeMethod(
		m_ODT,
		[&m_ODT = m_ODT]() {
			m_ODT->startAlignment();
		},
		Qt::AutoConnection
	);
}

void BrillouinAcquisition::on_alignmentCenterODT_clicked() {
	QMetaObject::invokeMethod(
		m_ODT,
		[&m_ODT = m_ODT]() {
			m_ODT->centerAlignment();
		},
		Qt::AutoConnection
	);
}

void BrillouinAcquisition::on_acquisitionUR_ODT_valueChanged(double voltage) {
	m_ODT->setSettings(ODT_MODE::ACQ, ODT_SETTING::VOLTAGE, voltage);
}

void BrillouinAcquisition::on_acquisitionNumber_ODT_valueChanged(int number) {
	m_ODT->setSettings(ODT_MODE::ACQ, ODT_SETTING::NRPOINTS, number);
}

void BrillouinAcquisition::on_acquisitionRate_ODT_valueChanged(double rate) {
	m_ODT->setSettings(ODT_MODE::ACQ, ODT_SETTING::SCANRATE, rate);
}

void BrillouinAcquisition::on_acquisitionStartODT_clicked() {
	if (m_ODT->getStatus() < ACQUISITION_STATUS::STARTED) {
		QMetaObject::invokeMethod(
			m_ODT,
			[&m_ODT = m_ODT]() {
				m_ODT->startRepetitions();
			},
			Qt::AutoConnection
		);
	} else {
		m_ODT->m_abort = true;
	}
}

void BrillouinAcquisition::on_exposureTimeODT_valueChanged(double exposureTime) {
	QMetaObject::invokeMethod(
		m_brightfieldCamera,
		[&m_brightfieldCamera = m_brightfieldCamera, exposureTime]() {
			m_brightfieldCamera->setSetting(CAMERA_SETTING::EXPOSURE, exposureTime);
		},
		Qt::AutoConnection
	);
}

void BrillouinAcquisition::on_gainODT_valueChanged(double gain) {
	QMetaObject::invokeMethod(
		m_brightfieldCamera,
		[&m_brightfieldCamera = m_brightfieldCamera, gain]() {
			m_brightfieldCamera->setSetting(CAMERA_SETTING::GAIN, gain);
		},
		Qt::AutoConnection
	);
}

void BrillouinAcquisition::on_exposureTimeCameraODT_valueChanged(double exposureTime) {
	QMetaObject::invokeMethod(
		m_ODT,
		[&m_ODT = m_ODT, exposureTime]() {
			m_ODT->setCameraSetting(CAMERA_SETTING::EXPOSURE, exposureTime);
		},
		Qt::AutoConnection
	);
}

void BrillouinAcquisition::on_gainCameraODT_valueChanged(double gain) {
	QMetaObject::invokeMethod(
		m_ODT,
		[&m_ODT = m_ODT, gain]() {
			m_ODT->setCameraSetting(CAMERA_SETTING::GAIN, gain);
		},
		Qt::AutoConnection
	);
}

void BrillouinAcquisition::on_pixelEncodingODT_currentIndexChanged(const QString& text) {
	auto encoding = text.toStdWString();
	QMetaObject::invokeMethod(
		m_brightfieldCamera,
		[&m_brightfieldCamera = m_brightfieldCamera, encoding]() {
			m_brightfieldCamera->setSetting(CAMERA_SETTING::ENCODING, encoding);
		},
		Qt::AutoConnection
	);
}

void BrillouinAcquisition::on_camera_displayMode_currentIndexChanged(const QString & text) {
	if (text == "Intensity") {
		m_ODTPlot.mode = DISPLAY_MODE::INTENSITY;
		m_ODTPlot.gradient = CustomGradientPreset::gpGrayscale;
	} else if (text == "Spectrum") {
		m_ODTPlot.mode = DISPLAY_MODE::SPECTRUM;
		m_ODTPlot.gradient = CustomGradientPreset::gpInferno;
	} else if (text == "Phase") {
		m_ODTPlot.mode = DISPLAY_MODE::PHASE;
		m_ODTPlot.gradient = CustomGradientPreset::gpInferno;
	} else {
		m_ODTPlot.mode = DISPLAY_MODE::INTENSITY;
		m_ODTPlot.gradient = CustomGradientPreset::gpGrayscale;
	}
	applyGradient(m_ODTPlot);
}

void BrillouinAcquisition::on_brightfieldRotationButton_clicked() {
	auto rotation = (int)m_brightfieldViewRotation;
	rotation += 1;
	rotation = (rotation + 4) % 4;
	m_brightfieldViewRotation = (BrightfieldViewRotation)rotation;
	applyBrightfieldViewTransformChanged();
}

void BrillouinAcquisition::on_brightfieldMirrorHorizontalButton_clicked() {
	m_brightfieldMirrorHorizontal = ui->brightfieldMirrorHorizontalButton->isChecked();
	applyBrightfieldViewTransformChanged();
}

void BrillouinAcquisition::on_brightfieldMirrorVerticalButton_clicked() {
	m_brightfieldMirrorVertical = ui->brightfieldMirrorVerticalButton->isChecked();
	applyBrightfieldViewTransformChanged();
}

void BrillouinAcquisition::applyBrightfieldViewTransformChanged() {
	updateBrightfieldTransformButtons();
	m_ODTPlot.colorMap->data()->setSize(brightfieldDisplayWidth(), brightfieldDisplayHeight());
	m_ODTPlot.colorMap->data()->setRange(QCPRange(1, brightfieldDisplayWidth()), QCPRange(1, brightfieldDisplayHeight()));
	m_ODTPlot.plotHandle->xAxis->setRange(QCPRange(1, brightfieldDisplayWidth()));
	m_ODTPlot.plotHandle->yAxis->setRange(QCPRange(1, brightfieldDisplayHeight()));
	if (m_scanControl) {
		AOI_changed(m_positionsMicrometer, m_positionsMicrometerIsAbsolute);
		excludedAOI_changed(m_excludedPositionsMicrometer);
		drawPositionScannerMarker(m_positionScanner);
	}
	updateImageODT();
}

QString BrillouinAcquisition::brightfieldRotationText() const {
	return QString("Rot %1 deg").arg((int)m_brightfieldViewRotation * 90);
}

void BrillouinAcquisition::updateBrightfieldTransformButtons() {
	if (ui->brightfieldRotationButton) {
		ui->brightfieldRotationButton->setText(brightfieldRotationText());
	}
	if (ui->brightfieldMirrorHorizontalButton) {
		ui->brightfieldMirrorHorizontalButton->setChecked(m_brightfieldMirrorHorizontal);
	}
	if (ui->brightfieldMirrorVerticalButton) {
		ui->brightfieldMirrorVerticalButton->setChecked(m_brightfieldMirrorVertical);
	}
}

void BrillouinAcquisition::on_setBackground_clicked() {
	QMetaObject::invokeMethod(
		m_converter,
		[&m_converter = m_converter]() {
			m_converter->updateBackground();
		},
		Qt::AutoConnection
	);
}

void BrillouinAcquisition::on_acquisitionStartFluorescence_clicked() {
	if (m_Fluorescence->getStatus() < ACQUISITION_STATUS::STARTED) {
		QMetaObject::invokeMethod(
			m_Fluorescence,
			[&m_Fluorescence = m_Fluorescence]() {
				m_Fluorescence->startRepetitions();
			},
			Qt::AutoConnection
		);
	} else {
		m_Fluorescence->m_abort = true;
	}
}

void BrillouinAcquisition::on_fluoBlueStart_clicked() {
	if (m_Fluorescence->getStatus() < ACQUISITION_STATUS::STARTED) {
		QMetaObject::invokeMethod(
			m_Fluorescence,
			[&m_Fluorescence = m_Fluorescence]() {
				m_Fluorescence->startRepetitions({ FLUORESCENCE_MODE::BLUE });
			},
			Qt::AutoConnection
		);
	}
	else {
		m_Fluorescence->m_abort = true;
	}
}

void BrillouinAcquisition::on_fluoGreenStart_clicked() {
	if (m_Fluorescence->getStatus() < ACQUISITION_STATUS::STARTED) {
		QMetaObject::invokeMethod(
			m_Fluorescence,
			[&m_Fluorescence = m_Fluorescence]() {
				m_Fluorescence->startRepetitions({ FLUORESCENCE_MODE::GREEN });
			},
			Qt::AutoConnection
		);
	}
	else {
		m_Fluorescence->m_abort = true;
	}
}

void BrillouinAcquisition::on_fluoRedStart_clicked() {
	if (m_Fluorescence->getStatus() < ACQUISITION_STATUS::STARTED) {
		QMetaObject::invokeMethod(
			m_Fluorescence,
			[&m_Fluorescence = m_Fluorescence]() {
				m_Fluorescence->startRepetitions({ FLUORESCENCE_MODE::RED });
			},
			Qt::AutoConnection
		);
	}
	else {
		m_Fluorescence->m_abort = true;
	}
}

void BrillouinAcquisition::on_fluoBrightfieldStart_clicked() {
	if (m_Fluorescence->getStatus() < ACQUISITION_STATUS::STARTED) {
		QMetaObject::invokeMethod(
			m_Fluorescence,
			[&m_Fluorescence = m_Fluorescence]() {
				m_Fluorescence->startRepetitions({ FLUORESCENCE_MODE::BRIGHTFIELD });
			},
			Qt::AutoConnection
		);
	}
	else {
		m_Fluorescence->m_abort = true;
	}
}

void BrillouinAcquisition::on_fluoBluePreview_clicked() {
	QMetaObject::invokeMethod(
		m_Fluorescence,
		[&m_Fluorescence = m_Fluorescence]() {
			m_Fluorescence->startStopPreview({ FLUORESCENCE_MODE::BLUE });
		},
		Qt::AutoConnection
	);
}

void BrillouinAcquisition::on_fluoGreenPreview_clicked() {
	QMetaObject::invokeMethod(
		m_Fluorescence,
		[&m_Fluorescence = m_Fluorescence]() {
			m_Fluorescence->startStopPreview({ FLUORESCENCE_MODE::GREEN });
		},
		Qt::AutoConnection
	);
}

void BrillouinAcquisition::on_fluoRedPreview_clicked() {
	QMetaObject::invokeMethod(
		m_Fluorescence,
		[&m_Fluorescence = m_Fluorescence]() {
			m_Fluorescence->startStopPreview({ FLUORESCENCE_MODE::RED });
		},
		Qt::AutoConnection
	);
}

void BrillouinAcquisition::on_fluoBrightfieldPreview_clicked() {
	QMetaObject::invokeMethod(
		m_Fluorescence,
		[&m_Fluorescence = m_Fluorescence]() {
			m_Fluorescence->startStopPreview({ FLUORESCENCE_MODE::BRIGHTFIELD });
		},
		Qt::AutoConnection
	);
}

void BrillouinAcquisition::on_fluoBlueCheckbox_stateChanged(int enabled) {
	m_Fluorescence->setChannel(FLUORESCENCE_MODE::BLUE, enabled);
}

void BrillouinAcquisition::on_fluoGreenCheckbox_stateChanged(int enabled) {
	m_Fluorescence->setChannel(FLUORESCENCE_MODE::GREEN, enabled);
}

void BrillouinAcquisition::on_fluoRedCheckbox_stateChanged(int enabled) {
	m_Fluorescence->setChannel(FLUORESCENCE_MODE::RED, enabled);
}

void BrillouinAcquisition::on_fluoBrightfieldCheckbox_stateChanged(int enabled) {
	m_Fluorescence->setChannel(FLUORESCENCE_MODE::BRIGHTFIELD, enabled);
}

void BrillouinAcquisition::on_fluoBlueExposure_valueChanged(int exposure) {
	m_Fluorescence->setExposure(FLUORESCENCE_MODE::BLUE, exposure);
}

void BrillouinAcquisition::on_fluoGreenExposure_valueChanged(int exposure) {
	m_Fluorescence->setExposure(FLUORESCENCE_MODE::GREEN, exposure);
}

void BrillouinAcquisition::on_fluoRedExposure_valueChanged(int exposure) {
	m_Fluorescence->setExposure(FLUORESCENCE_MODE::RED, exposure);
}

void BrillouinAcquisition::on_fluoBrightfieldExposure_valueChanged(int exposure) {
	m_Fluorescence->setExposure(FLUORESCENCE_MODE::BRIGHTFIELD, exposure);
	m_Brillouin->settings.overviewBrightfieldExposureMs = exposure;
}

void BrillouinAcquisition::on_fluoBlueGain_valueChanged(double gain) {
	m_Fluorescence->setGain(FLUORESCENCE_MODE::BLUE, gain);
}

void BrillouinAcquisition::on_fluoGreenGain_valueChanged(double gain) {
	m_Fluorescence->setGain(FLUORESCENCE_MODE::GREEN, gain);
}

void BrillouinAcquisition::on_fluoRedGain_valueChanged(double gain) {
	m_Fluorescence->setGain(FLUORESCENCE_MODE::RED, gain);
}

void BrillouinAcquisition::on_fluoBrightfieldGain_valueChanged(double gain) {
	m_Fluorescence->setGain(FLUORESCENCE_MODE::BRIGHTFIELD, gain);
	m_Brillouin->settings.overviewBrightfieldGain = gain;
}

void BrillouinAcquisition::updateFluorescenceSettings(const FLUORESCENCE_SETTINGS& settings) {
	bool disableStart{ true };
	if (settings.blue.enabled || settings.red.enabled || settings.green.enabled || settings.brightfield.enabled) {
		disableStart = false;
	}
	ui->acquisitionStartFluorescence->setDisabled(disableStart);
	ui->fluoBlueCheckbox->setChecked(settings.blue.enabled);
	ui->fluoGreenCheckbox->setChecked(settings.green.enabled);
	ui->fluoRedCheckbox->setChecked(settings.red.enabled);
	ui->fluoBrightfieldCheckbox->setChecked(settings.brightfield.enabled);
	ui->fluoBlueExposure->setValue(settings.blue.exposure);
	ui->fluoGreenExposure->setValue(settings.green.exposure);
	ui->fluoRedExposure->setValue(settings.red.exposure);
	ui->fluoBrightfieldExposure->setValue(settings.brightfield.exposure);
	ui->fluoBlueGain->setValue(settings.blue.gain);
	ui->fluoGreenGain->setValue(settings.green.gain);
	ui->fluoRedGain->setValue(settings.red.gain);
	ui->fluoBrightfieldGain->setValue(settings.brightfield.gain);
	m_Brillouin->settings.overviewBrightfieldExposureMs = settings.brightfield.exposure;
	m_Brillouin->settings.overviewBrightfieldGain = settings.brightfield.gain;
}

void BrillouinAcquisition::showEnabledModes(ACQUISITION_MODE modes) {
	m_enabledModes = modes;
	/*
	 * Handle Brillouin and fluorescence mode
	 *
	 * If either Brillouin or fluorescence mode is enabled, disable ODT controls (enable otherwise).
	 */
	bool BrillouinMode = (bool)(m_enabledModes & ACQUISITION_MODE::BRILLOUIN);
	bool FluorescenceMode = (bool)(m_enabledModes & ACQUISITION_MODE::FLUORESCENCE);

	if (BrillouinMode || FluorescenceMode) {
		ui->acquisitionStartODT->setEnabled(false);
		ui->alignmentStartODT->setEnabled(false);
		ui->alignmentCenterODT->setEnabled(false);
	} else {
		ui->acquisitionStartODT->setEnabled(true);
		ui->alignmentStartODT->setEnabled(true);
		ui->alignmentCenterODT->setEnabled(true);
	}

	/*
	* Handle ODT mode
	*
	* If ODT mode is enabled, disable Brillouin and Fluorescence controls (enable otherwise).
	*/
	updateBrillouinStartAvailability();
}

void BrillouinAcquisition::showBrillouinStatus(ACQUISITION_STATUS status) {
	QString string;
	bool running{ false };
	// Only the WAITFORSURFACEREVIEW case (below) (re)starts this - any other status means
	// the pause ended (manual Continue/Full grid, or the timeout itself), so stop it here
	// unconditionally rather than duplicating that in every other case.
	if (status != ACQUISITION_STATUS::WAITFORSURFACEREVIEW) {
		m_surfaceReviewTimer->stop();
	}
	switch (status) {
		case ACQUISITION_STATUS::ABORTED:
			string = "Acquisition aborted.";
			ui->progressBar->setValue(0);
			ui->BrillouinStart->setText("Start");
			if (m_brightfieldPreviewStartedForSurfaceReview) {
				m_brightfieldPreviewStartedForSurfaceReview = false;
				showBrightfieldPreviewRunning(false);
			}
			break;
		case ACQUISITION_STATUS::FINISHED:
			string = "Acquisition finished.";
			ui->progressBar->setValue(100);
			ui->BrillouinStart->setText("Start");
			break;
		case ACQUISITION_STATUS::STARTED:
			string = "Acquisition started.";
			ui->progressBar->setValue(0);
			if (m_brightfieldPreviewStartedForSurfaceReview) {
				// Leaving the review pause (Continue/Full grid was clicked) - stop the
				// live view we auto-started for it, but only that one, not one the user
				// may have started themselves for an unrelated reason.
				m_brightfieldPreviewStartedForSurfaceReview = false;
				showBrightfieldPreviewRunning(false);
			}
			[[fallthrough]];
		case ACQUISITION_STATUS::RUNNING:
			ui->BrillouinStart->setText("Cancel");
			running = true;
			break;
		case ACQUISITION_STATUS::WAITFORREPETITION:
			ui->BrillouinStart->setText("Stop");
			running = true;
			break;
		case ACQUISITION_STATUS::WAITFORSURFACEREVIEW:
			m_surfaceReviewSecondsRemaining = kSurfaceReviewTimeoutS;
			string = QString("Surface scan finished - review the grid, then Continue or Full grid. "
				"Auto-continuing in %1 s...").arg(m_surfaceReviewSecondsRemaining);
			ui->progressBar->setValue(100);
			ui->BrillouinStart->setText("Continue");
			running = true;
			// Live view at the pre-acquisition position/preset already set up by
			// Brillouin::acquire() itself - this just brings the GUI's own preview state
			// (button text, frame grab loop) in sync with it, unless the user already had
			// it running themselves.
			if (!m_brightfieldPreviewRunning) {
				m_brightfieldPreviewStartedForSurfaceReview = true;
				showBrightfieldPreviewRunning(true);
			}
			update_AOI_preview();
			m_surfaceReviewTimer->start();
			break;
		case ACQUISITION_STATUS::STOPPED:
			ui->BrillouinStart->setText("Start");
			break;
		default:
			ui->BrillouinStart->setText("Start");
			break;
	}
	ui->fullGridButton->setEnabled(status == ACQUISITION_STATUS::WAITFORSURFACEREVIEW);
	ui->progressBar->setFormat(string);

	ui->actionOpen_Acquisition->setDisabled(running);
	ui->actionNew_Acquisition->setDisabled(running);
	ui->actionClose_Acquisition->setDisabled(running);

	// Grid range/steps are offsets from resolvedGridOriginUm() in both modes, but in absolute
	// mode that origin is a fixed point set once (not the live stage position - see
	// preservePhysicalGridForAbsoluteMode()), so editing these numbers here has no intuitive
	// physical meaning while standing at the microscope: how big/where the grid actually is
	// can't be read off them without also knowing the (separately displayed) origin. Locked to
	// read-only in absolute mode for that reason, on top of (not instead of) the pre-existing
	// running-state lock.
	// Z is deliberately excluded from the absolute-mode lock - see
	// Brillouin::resolvedGridOriginUm()'s comment for why z is not part of "absolute" at all
	// anymore. Its own read-only meaning issue (editing offsets from a fixed origin you can't
	// see here) doesn't apply: z's origin is always m_startPosition.z, refreshed at every Start
	// or via "Set plane", so zMin/zMax stay exactly as intuitive here as they already are in
	// relative mode.
	const bool gridLockedXY = running || m_Brillouin->settings.gridCoordinatesAbsolute;
	const bool gridLockedZ = running;
	ui->startX->setDisabled(gridLockedXY);
	ui->startY->setDisabled(gridLockedXY);
	ui->startZ->setDisabled(gridLockedZ);
	ui->endX->setDisabled(gridLockedXY);
	ui->endY->setDisabled(gridLockedXY);
	ui->endZ->setDisabled(gridLockedZ);
	ui->stepsX->setDisabled(gridLockedXY);
	ui->stepsY->setDisabled(gridLockedXY);
	ui->stepsZ->setDisabled(gridLockedZ);
	ui->camera_playPause->setDisabled(running);
	ui->camera_singleShot->setDisabled(running);
	// Enabled in both modes now - see on_setHome_clicked()'s comment for why it does something
	// useful (Set plane) in absolute mode too, instead of being disabled there.
	ui->setHome->setDisabled(running);
	ui->setHome->setText(m_Brillouin->settings.gridCoordinatesAbsolute ? "Set plane" : "Set home");
	ui->moveHome->setDisabled(running || m_Brillouin->settings.gridCoordinatesAbsolute);
	ui->setPositionX->setDisabled(running);
	ui->setPositionY->setDisabled(running);
	ui->setPositionZ->setDisabled(running);

	// Relocating the beam marker mid-acquisition would move the reference every remaining grid
	// point measures against - never meaningful while running. Also drop out of an in-progress
	// relocation (armed by a click, not yet confirmed by a second click) rather than leaving the
	// button stuck in its "Ok" state with no way to finish it once disabled.
	if (running && m_locatePositionScanner) {
		setLaserPositionLocationArmed(false);
	}
	ui->addFocusMarker_brightfield->setDisabled(running);

	// The relocation button's whole point is compensating the relative-mode grid so it stays
	// fixed - in absolute mode that compensation is a no-op (the absolute grid formula has no B
	// term at all, see relocateBeamKeepingGridFixed()'s own comment), so there's nothing this
	// button does differently from the plain marker button there - grey it out rather than offer
	// a control with no distinguishing effect.
	const auto relocateMarkerUnusable = running || m_Brillouin->settings.gridCoordinatesAbsolute;
	if (relocateMarkerUnusable && m_relocatePositionScanner) {
		setRelocateFocusMarkerArmed(false);
	}
	ui->relocateFocusMarker_brightfield->setDisabled(relocateMarkerUnusable);

	ui->postCalibration->setDisabled(running);
	ui->preCalibration->setDisabled(running);
	ui->conCalibration->setDisabled(running);
	ui->conCalibrationInterval->setDisabled(running);
	ui->sampleSelection->setDisabled(running);
	ui->nrCalibrationImages->setDisabled(running);
	ui->calibrationExposureTime->setDisabled(running);
	ui->repetitionInterval->setDisabled(running);
	ui->repetitionCount->setDisabled(running);
	updateBrillouinStartAvailability();
}

/*
 * Ticks m_surfaceReviewSecondsRemaining down once a second while paused at
 * WAITFORSURFACEREVIEW (see showBrillouinStatus()), showing the countdown on the same
 * acquisition-progress bar the pause message already uses. Auto-continues (as if
 * "Continue" was clicked) once it reaches zero, so the acquisition doesn't sit paused
 * indefinitely if the user doesn't respond.
 */
void BrillouinAcquisition::onSurfaceReviewTimerTick() {
	if (m_Brillouin->getStatus() != ACQUISITION_STATUS::WAITFORSURFACEREVIEW) {
		// Stale tick racing a status change that already stopped the timer - ignore.
		m_surfaceReviewTimer->stop();
		return;
	}

	m_surfaceReviewSecondsRemaining--;
	if (m_surfaceReviewSecondsRemaining <= 0) {
		m_surfaceReviewTimer->stop();
		ui->progressBar->setValue(0);
		ui->progressBar->setFormat("Surface scan review timed out - continuing automatically.");
		QMetaObject::invokeMethod(
			m_Brillouin,
			[brillouin = m_Brillouin]() { brillouin->continueAfterSurfaceReview(false); },
			Qt::AutoConnection
		);
		return;
	}

	ui->progressBar->setValue((int)(100.0 * m_surfaceReviewSecondsRemaining / kSurfaceReviewTimeoutS));
	ui->progressBar->setFormat(QString("Surface scan finished - review the grid, then Continue or Full grid. "
		"Auto-continuing in %1 s...").arg(m_surfaceReviewSecondsRemaining));
}

void BrillouinAcquisition::showBrillouinProgress(double progress, int seconds) {
	ui->progressBar->setValue(progress);

	QString string;
	QString timeString = formatSeconds(seconds);
	string.sprintf("%02.1f %% finished, ", progress);
	string += timeString;
	string += " remaining.";
	ui->progressBar->setFormat(string);
}

void BrillouinAcquisition::showSurfaceScanProgress(double progress, const QString& message) {
	ui->statusBar->showMessage(QString("%1 (%2% complete)")
		.arg(message)
		.arg(std::clamp(progress, 0.0, 100.0), 0, 'f', 1));
	if (progress >= 100.0) {
		refreshSpectralProxyRoiRects();
		ui->customplot->replot();
	}
}

void BrillouinAcquisition::on_measureSpectralProxyRoiButton_clicked() {
	auto* mapData = m_BrillouinPlot.colorMap ? m_BrillouinPlot.colorMap->data() : nullptr;
	if (!mapData || mapData->isEmpty()) {
		ui->statusBar->showMessage("No spectral image available for ROI measurement.", 5000);
		return;
	}

	const int frameW = std::max(1, mapData->keySize());
	const int frameH = std::max(1, mapData->valueSize());
	auto measureRoi = [mapData, frameW, frameH](int left, int top, int width, int height, double& maxValue) {
		if (width <= 0 || height <= 0) {
			return false;
		}
		const int clampedLeft = std::clamp(left, 0, frameW - 1);
		const int clampedTop = std::clamp(top, 0, frameH - 1);
		const int clampedRight = std::clamp(left + width - 1, clampedLeft, frameW - 1);
		const int clampedBottom = std::clamp(top + height - 1, clampedTop, frameH - 1);

		maxValue = -std::numeric_limits<double>::infinity();
		for (int displayY = clampedTop; displayY <= clampedBottom; displayY++) {
			for (int x = clampedLeft; x <= clampedRight; x++) {
				maxValue = std::max(maxValue, mapData->cell(x, displayY));
			}
		}
		return std::isfinite(maxValue);
	};

	const auto& settings = m_Brillouin->settings;
	// Remap onto the currently displayed frame if it differs from whatever frame the ROI
	// was drawn against - see Brillouin::remapProxyRoi() for why a size-only rescale isn't
	// enough, and estimateFrameMetric() for why the actual surface scan does the same remap
	// (keeps this manual check honest about what that would measure).
	const PROXY_ROI_FRAME currentFrame{
		frameW, frameH,
		m_Brillouin->settings.camera.roi.left, m_Brillouin->settings.camera.roi.bottom,
		m_Brillouin->settings.camera.roi.width_physical, m_Brillouin->settings.camera.roi.height_physical
	};
	auto remapped = [&currentFrame](
		int roiLeft, int roiTop, int roiWidth, int roiHeight, const PROXY_ROI_FRAME& drawnFrame,
		int& outLeft, int& outTop, int& outWidth, int& outHeight
	) {
		outLeft = roiLeft; outTop = roiTop; outWidth = roiWidth; outHeight = roiHeight;
		Brillouin::remapProxyRoi(roiLeft, roiTop, roiWidth, roiHeight, drawnFrame, currentFrame,
			outLeft, outTop, outWidth, outHeight);
	};
	double roi1Mean{ 0.0 };
	double roi2Mean{ 0.0 };
	int roi1Left{ 0 }, roi1Top{ 0 }, roi1Width{ 0 }, roi1Height{ 0 };
	remapped(settings.surfaceProxyRoiLeft, settings.surfaceProxyRoiTop,
		settings.surfaceProxyRoiWidth, settings.surfaceProxyRoiHeight,
		PROXY_ROI_FRAME{
			settings.surfaceProxyRoiFrameWidth, settings.surfaceProxyRoiFrameHeight,
			settings.surfaceProxyRoiFrameOriginLeft, settings.surfaceProxyRoiFrameOriginBottom,
			settings.surfaceProxyRoiFrameWidthPhysical, settings.surfaceProxyRoiFrameHeightPhysical
		},
		roi1Left, roi1Top, roi1Width, roi1Height);
	const bool hasRoi1 = measureRoi(roi1Left, roi1Top, roi1Width, roi1Height, roi1Mean);

	int roi2Left{ 0 }, roi2Top{ 0 }, roi2Width{ 0 }, roi2Height{ 0 };
	remapped(settings.surfaceProxyRoi2Left, settings.surfaceProxyRoi2Top,
		settings.surfaceProxyRoi2Width, settings.surfaceProxyRoi2Height,
		PROXY_ROI_FRAME{
			settings.surfaceProxyRoi2FrameWidth, settings.surfaceProxyRoi2FrameHeight,
			settings.surfaceProxyRoi2FrameOriginLeft, settings.surfaceProxyRoi2FrameOriginBottom,
			settings.surfaceProxyRoi2FrameWidthPhysical, settings.surfaceProxyRoi2FrameHeightPhysical
		},
		roi2Left, roi2Top, roi2Width, roi2Height);
	const bool hasRoi2 = measureRoi(roi2Left, roi2Top, roi2Width, roi2Height, roi2Mean);

	if (!hasRoi1 && !hasRoi2) {
		ui->statusBar->showMessage("No valid spectral ROI to measure.", 5000);
		return;
	}

	const auto average = hasRoi1 && hasRoi2 ? 0.5 * (roi1Mean + roi2Mean) : (hasRoi1 ? roi1Mean : roi2Mean);
	QStringList parts;
	parts << QString("avg %1").arg(average, 0, 'f', 2);
	if (hasRoi1) {
		parts << QString("ROI1 %1").arg(roi1Mean, 0, 'f', 2);
	}
	if (hasRoi2) {
		parts << QString("ROI2 %1").arg(roi2Mean, 0, 'f', 2);
	}
	ui->statusBar->showMessage(QString("Spectral ROI max: %1").arg(parts.join(", ")));
}

void BrillouinAcquisition::showODTStatus(ACQUISITION_STATUS status) {
	QString string;
	if (status == ACQUISITION_STATUS::ABORTED) {
		string = "Acquisition aborted.";
		ui->acquisitionProgress_ODT->setValue(0);
	}
	else if (status == ACQUISITION_STATUS::FINISHED) {
		string = "Acquisition finished.";
		ui->acquisitionProgress_ODT->setValue(100);
	}
	else if (status == ACQUISITION_STATUS::STARTED) {
		string = "Acquisition started.";
		ui->acquisitionProgress_ODT->setValue(0);
	}
	ui->acquisitionProgress_ODT->setFormat(string);

	bool running{ false };
	if (status == ACQUISITION_STATUS::RUNNING || status == ACQUISITION_STATUS::STARTED) {
		ui->acquisitionStartODT->setText("Cancel");
		running = true;
	} else {
		ui->acquisitionStartODT->setText("Start");
	}

	ui->actionOpen_Acquisition->setDisabled(running);
	ui->actionNew_Acquisition->setDisabled(running);
	ui->actionClose_Acquisition->setDisabled(running);

	ui->alignmentStartODT->setDisabled(running);
	ui->acquisitionUR_ODT->setDisabled(running);
	ui->acquisitionNumber_ODT->setDisabled(running);
	ui->acquisitionRate_ODT->setDisabled(running);

	if (status == ACQUISITION_STATUS::ALIGNING) {
		ui->alignmentStartODT->setText("Stop");
	} else {
		ui->alignmentStartODT->setText("Start");
	}

	if (status > ACQUISITION_STATUS::STOPPED) {
		ui->alignmentCenterODT->setEnabled(false);
	} else {
		ui->alignmentCenterODT->setEnabled(true);
	}
}

void BrillouinAcquisition::showODTProgress(double progress, int seconds) {
	ui->acquisitionProgress_ODT->setValue(progress);

	QString string;
	QString timeString = formatSeconds(seconds);
	string.sprintf("%02.1f %% finished, ", progress);
	string += timeString;
	string += " remaining.";
	ui->acquisitionProgress_ODT->setFormat(string);
}

void BrillouinAcquisition::showFluorescenceStatus(ACQUISITION_STATUS status) {
	QString string;
	if (status == ACQUISITION_STATUS::ABORTED) {
		string = "Acquisition aborted.";
		ui->fluoProgress->setValue(0);
	} else if (status == ACQUISITION_STATUS::FINISHED) {
		string = "Acquisition finished.";
		ui->fluoProgress->setValue(100);
	} else if (status == ACQUISITION_STATUS::STARTED) {
		string = "Acquisition started.";
		ui->fluoProgress->setValue(0);
	}
	ui->fluoProgress->setFormat(string);

	bool running{ false };
	if (status == ACQUISITION_STATUS::RUNNING || status == ACQUISITION_STATUS::STARTED) {
		ui->acquisitionStartFluorescence->setText("Cancel");
		running = true;
	} else {
		ui->acquisitionStartFluorescence->setText("Acquire All");
	}
	//startBrightfieldPreview(running);

	ui->fluoBlueStart->setDisabled(running);
	ui->fluoGreenStart->setDisabled(running);
	ui->fluoRedStart->setDisabled(running);
	ui->fluoBrightfieldStart->setDisabled(running);

	// reset all preview buttons
	ui->fluoBluePreview->setText("Preview");
	ui->fluoGreenPreview->setText("Preview");
	ui->fluoRedPreview->setText("Preview");
	ui->fluoBrightfieldPreview->setText("Preview");

	ui->fluoBluePreview->setDisabled(running);
	ui->fluoGreenPreview->setDisabled(running);
	ui->fluoRedPreview->setDisabled(running);
	ui->fluoBrightfieldPreview->setDisabled(running);

	ui->fluoBlueCheckbox->setDisabled(running);
	ui->fluoGreenCheckbox->setDisabled(running);
	ui->fluoRedCheckbox->setDisabled(running);
	ui->fluoBrightfieldCheckbox->setDisabled(running);
	ui->fluoBlueExposure->setDisabled(running);
	ui->fluoGreenExposure->setDisabled(running);
	ui->fluoRedExposure->setDisabled(running);
	ui->fluoBrightfieldExposure->setDisabled(running);
	ui->fluoBlueGain->setDisabled(running);
	ui->fluoGreenGain->setDisabled(running);
	ui->fluoRedGain->setDisabled(running);
	ui->fluoBrightfieldGain->setDisabled(running);
}

void BrillouinAcquisition::showFluorescenceProgress(double progress, int seconds) {
	ui->fluoProgress->setValue(progress);

	QString string;
	QString timeString = formatSeconds(seconds);
	string.sprintf("%02.1f %% finished, ", progress);
	string += timeString;
	string += " remaining.";
	ui->fluoProgress->setFormat(string);
}

QString BrillouinAcquisition::formatSeconds(int seconds) {
	QString string;
	if (seconds > 3600) {
		int hours = floor((double)seconds / 3600);
		int minutes = floor((seconds - hours * 3600) / 60);
		string.sprintf("%02.0f:%02.0f hours", (double)hours, (double)minutes);
	}
	else if (seconds > 60) {
		int minutes = floor(seconds / 60);
		seconds = floor(seconds - minutes * 60);
		string.sprintf("%02.0f:%02.0f minutes", (double)minutes, (double)seconds);
	}
	else {
		string.sprintf("%2.0f seconds", (double)seconds);
	}
	return string;
}

void BrillouinAcquisition::plotODTVoltages(const ODT_SETTINGS& settings, const ODT_MODE& mode) {
	QCustomPlot *plot = nullptr;
	switch (mode) {
		case ODT_MODE::ALGN: {
			plot = ui->alignmentVoltagesODT;
			const QSignalBlocker blocker1(ui->alignmentUR_ODT);
			const QSignalBlocker blocker2(ui->alignmentNumber_ODT);
			const QSignalBlocker blocker3(ui->alignmentRate_ODT);
			ui->alignmentUR_ODT->setValue(settings.radialVoltage);
			ui->alignmentNumber_ODT->setValue(settings.numberPoints);
			ui->alignmentRate_ODT->setValue(settings.scanRate);
			break;
		}
		case ODT_MODE::ACQ: {
			plot = ui->acquisitionVoltagesODT;
			const QSignalBlocker blocker1(ui->acquisitionUR_ODT);
			const QSignalBlocker blocker2(ui->acquisitionNumber_ODT);
			const QSignalBlocker blocker3(ui->acquisitionRate_ODT);
			ui->acquisitionUR_ODT->setValue(settings.radialVoltage);
			ui->acquisitionNumber_ODT->setValue(settings.numberPoints);
			ui->acquisitionRate_ODT->setValue(settings.scanRate);
			break;
		}
		default:
			return;
	}

	std::vector<VOLTAGE2> voltages = settings.voltages;
	QVector<double> Ux(voltages.size()), Uy(voltages.size());
	for (gsl::index index{ 0 }; index < voltages.size(); ++index) {
		Ux[index] = voltages[index].Ux;
		Uy[index] = voltages[index].Uy;
	}

	plot->graph(0)->setData(Ux, Uy);
	// scale the axis appropriately
	plot->xAxis->setRange(-1.1*settings.radialVoltage, 1.1*settings.radialVoltage);
	plot->yAxis->setRange(-1.1*settings.radialVoltage, 1.1*settings.radialVoltage);
	
	// set the aspect ratio
	//plot->yAxis->setScaleRatio(plot->xAxis, 1.0); // somehow makes it worse
	plot->replot();
}

void BrillouinAcquisition::plotODTVoltage(const VOLTAGE2& voltage, const ODT_MODE& mode) {
	QCustomPlot *plot;
	switch (mode) {
		case ODT_MODE::ALGN:
			plot = ui->alignmentVoltagesODT;
			break;
		case ODT_MODE::ACQ:
			plot = ui->acquisitionVoltagesODT;
			break;
		default:
			return;
	}
	QVector<double> Ux{ voltage.Ux }, Uy{ voltage.Uy };
	plot->graph(1)->setData(Ux, Uy);
	plot->replot();
}

void BrillouinAcquisition::initializePlot(PLOT_SETTINGS plotSettings) {
	// configure axis rect

	plotSettings.plotHandle->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom); // this will also allow rescaling the color scale by dragging/zooming
	plotSettings.plotHandle->axisRect()->setupFullAxesBox(true);
	//customPlot->xAxis->setLabel("x");
	//customPlot->yAxis->setLabel("y");

	// this are the selection modes
	plotSettings.plotHandle->setSelectionRectMode(QCP::srmZoom);	// allows to select region by rectangle
	plotSettings.plotHandle->setSelectionRectMode(QCP::srmNone);	// allows to drag the position

	// set background color to default light gray
	plotSettings.plotHandle->setBackground(QColor(240, 240, 240, 255));
	plotSettings.plotHandle->axisRect()->setBackground(Qt::white);

	// fill map with zero
	plotSettings.colorMap->data()->fill(0);

	// turn off interpolation
	plotSettings.colorMap->setInterpolate(false);

	// add a color scale:
	QCPColorScale *colorScale = new QCPColorScale(plotSettings.plotHandle);
	plotSettings.plotHandle->plotLayout()->addElement(0, 1, colorScale); // add it to the right of the main axis rect
	colorScale->setType(QCPAxis::atRight); // scale shall be vertical bar with tick/axis labels right (actually atRight is already the default)
	plotSettings.colorMap->setColorScale(colorScale); // associate the color map with the color scale
	colorScale->axis()->setLabel("Intensity");

	auto connection = QWidget::connect<void(QCPColorMap::*)(const QCPRange &)>(
		plotSettings.colorMap,
		&QCPColorMap::dataRangeChanged,
		this,
		[this, plotSettings](QCPRange newRange) { (plotSettings.dataRangeCallback)(newRange); }
	);

	// set the color gradient of the color map to one of the presets:
	applyGradient(plotSettings);

	plotSettings.colorMap->setDataRange(plotSettings.cLim);

	(plotSettings.dataRangeCallback)(plotSettings.cLim);
	// rescale the data dimension (color) such that all data points lie in the span visualized by the color gradient:
	if (plotSettings.autoscale) {
		plotSettings.colorMap->rescaleDataRange(true);
		plotSettings.cLim = plotSettings.colorMap->dataRange();
		(plotSettings.dataRangeCallback)(plotSettings.cLim);
	}

	// make sure the axis rect and color scale synchronize their bottom and top margins (so they line up):
	QCPMarginGroup *marginGroup = new QCPMarginGroup(plotSettings.plotHandle);
	plotSettings.plotHandle->axisRect()->setMarginGroup(QCP::msBottom | QCP::msTop, marginGroup);
	colorScale->setMarginGroup(QCP::msBottom | QCP::msTop, marginGroup);

	// rescale the key (x) and value (y) axes so the whole color map is visible:
	plotSettings.plotHandle->rescaleAxes();
}

void BrillouinAcquisition::applyGradient(const PLOT_SETTINGS& plotSettings) {
	QCPColorGradient gradient = QCPColorGradient();
	setColormap(&gradient, plotSettings.gradient);
	plotSettings.colorMap->setGradient(gradient);
}

void BrillouinAcquisition::initializeLaserPositionLocation() {
	// If the scanControl supports capability LaserScanner, there is no need to set the laser
	// position manually - neither button applies (a galvo system steers the beam itself).
	if (m_scanControl != nullptr && m_scanControl->supportsCapability(Capabilities::LaserScanner)) {
		ui->addFocusMarker_brightfield->hide();
		ui->relocateFocusMarker_brightfield->hide();
	} else {
		ui->addFocusMarker_brightfield->show();
		ui->relocateFocusMarker_brightfield->show();
	}
}

void BrillouinAcquisition::on_addFocusMarker_brightfield_clicked() {
	// If the scanControl supports capability LaserScanner, there is no need to set the laser position manually.
	if (m_scanControl != nullptr && m_scanControl->supportsCapability(Capabilities::LaserScanner)) {
		return;
	}
	// Plain, uncompensated relocation - initial one-time setup (or startup restore, via
	// ScanControl::setPendingRestoredMarker()). The relative-mode grid follows B here, same as
	// it always has (relative idle mode is not permanently sample-anchored - see the grid-math
	// comment on ScanPlanner::buildLegacyCartesianPlan()). See on_relocateFocusMarker_brightfield_
	// clicked() for the other, compensating button.
	setLaserPositionLocationArmed(!m_locatePositionScanner);
}

void BrillouinAcquisition::setLaserPositionLocationArmed(bool armed) {
	m_locatePositionScanner = armed;
	if (armed) {
		// The two relocation buttons are mutually exclusive - arming one cancels the other,
		// so a subsequent image click always has exactly one unambiguous meaning.
		setRelocateFocusMarkerArmed(false);
	}
	ui->addFocusMarker_brightfield->setIcon(m_icons.fluoBlue);
	ui->addFocusMarker_brightfield->setText(armed ? "Ok" : "");
}

void BrillouinAcquisition::on_relocateFocusMarker_brightfield_clicked() {
	if (m_scanControl != nullptr && m_scanControl->supportsCapability(Capabilities::LaserScanner)) {
		return;
	}
	if (!m_relocatePositionScanner) {
		// Redefines the beam-to-sample offset (B) for the objective that's active right now - it
		// is a physical property of that objective's own optical path, not something a
		// FOV-registration calibration can infer for a different one (see the grid-math comment
		// on ScanPlanner::buildLegacyCartesianPlan()). Require an explicit acknowledgement before
		// arming, the same way starting an absolute-mode grid with no FOV-offset calibration does
		// (see Brillouin::startRepetitions()).
		const auto reply = QMessageBox::warning(this, "Set laser spot",
			"This will redefine the laser spot position for the currently active objective only.\n\n"
			"It is NOT applied retroactively to any grid or points already measured with the "
			"previous position - only to measurements taken after you confirm the new spot.\n\n"
			"Continue?",
			QMessageBox::Ok | QMessageBox::Cancel, QMessageBox::Cancel);
		if (reply != QMessageBox::Ok) {
			return;
		}
	}
	setRelocateFocusMarkerArmed(!m_relocatePositionScanner);
}

void BrillouinAcquisition::setRelocateFocusMarkerArmed(bool armed) {
	m_relocatePositionScanner = armed;
	if (armed) {
		setLaserPositionLocationArmed(false);
	}
	ui->relocateFocusMarker_brightfield->setIcon(m_icons.fluoGreen);
	ui->relocateFocusMarker_brightfield->setText(armed ? "Ok" : "");
}

// Relocates the beam marker (B) the way an explicit, mid-session correction should behave: in
// relative mode, every already-configured grid target stays physically fixed on screen, and the
// AOI numbers (xMin/xMax/yMin/yMax) plus both ROI polygons are compensated instead. This is
// deliberately NOT what ScanControl::locatePositionScanner() itself does (that path - initial
// one-time setup, and the startup restore from settings - leaves the grid following B, since
// relative idle mode is not permanently sample-anchored - see the grid-math comment on
// ScanPlanner::buildLegacyCartesianPlan()). Only this explicit relocation button gets the
// compensating behavior.
//
// Absolute mode's grid formula has no B term at all (same comment), so there is nothing to
// compensate there - relocating B only moves the marker itself.
//
// Implementation reuses gridOffsetToImagePlaneUm()/imagePlaneUmToGridOffset() - the same two
// functions preservePhysicalGridForAbsoluteMode() uses to preserve physical targets across a
// mode change - held at the relative-mode convention throughout, with B (not the mode) changing
// in between the two calls. Deliberately not implemented by toggling the real
// gridCoordinatesAbsolute setting and back: that would re-trigger the full async mode-switch
// chain (queued Brillouin::updatePositions(), spinbox enable/disable, redraw) twice in a row as
// an "invisible" implementation detail, risking exactly the kind of GUI-thread/worker-thread
// race the rest of the coordinate-system cleanup eliminated.
void BrillouinAcquisition::relocateBeamKeepingGridFixed(POINT2 newMarkerPix) {
	if (!m_scanControl) {
		return;
	}
	if (m_Brillouin->settings.gridCoordinatesAbsolute) {
		m_scanControl->locatePositionScanner(newMarkerPix);
		return;
	}

	auto& settings = m_Brillouin->settings;
	const auto oldMinUm = gridOffsetToImagePlaneUm(POINT2{ settings.xMin, settings.yMin }, false);
	const auto oldMaxUm = gridOffsetToImagePlaneUm(POINT2{ settings.xMax, settings.yMax }, false);
	const std::array<RoiTarget, 2> roiTargets{ mainRoiTarget(), backgroundRoiTarget() };
	std::array<std::vector<POINT2>, 2> oldRoiUm;
	for (size_t t = 0; t < roiTargets.size(); t++) {
		oldRoiUm[t].reserve(roiTargets[t].polygon->size());
		for (const auto& p : *roiTargets[t].polygon) {
			oldRoiUm[t].push_back(gridOffsetToImagePlaneUm(p, false));
		}
	}

	m_scanControl->locatePositionScanner(newMarkerPix);

	const auto newMinXY = imagePlaneUmToGridOffset(oldMinUm, false);
	const auto newMaxXY = imagePlaneUmToGridOffset(oldMaxUm, false);
	settings.setXMin(newMinXY.x);
	settings.setXMax(newMaxXY.x);
	settings.setYMin(newMinXY.y);
	settings.setYMax(newMaxXY.y);
	for (size_t t = 0; t < roiTargets.size(); t++) {
		auto& polygon = *roiTargets[t].polygon;
		for (size_t i = 0; i < polygon.size(); i++) {
			polygon[i] = imagePlaneUmToGridOffset(oldRoiUm[t][i], false);
		}
		updateRoiPolygonPreviewFor(roiTargets[t]);
	}

	updateBrillouinSettings();
	QMetaObject::invokeMethod(m_Brillouin, "updatePositions", Qt::AutoConnection);
}

void BrillouinAcquisition::drawPositionScannerMarker(POINT2 positionScanner) {
	m_positionScanner = positionScanner;
	if (m_scanControl) {
		m_positionScannerObjectiveSlot = m_scanControl->getActiveObjectiveSlot();
	}
	const auto positionScannerDisplay = brightfieldRawToDisplay(positionScanner);
	// Don't draw if outside of image
	if (positionScannerDisplay.x < 1 || positionScannerDisplay.y < 1
		|| positionScannerDisplay.x > brightfieldDisplayWidth()
		|| positionScannerDisplay.y > brightfieldDisplayHeight()) {
		return;
	}

	// Add a marker to the plot to indicate the laser focus
	if (!m_positionScannerMarker) {
		m_positionScannerMarker = m_ODTPlot.plotHandle->addGraph();
		QPen pen;
		pen.setColor(Qt::blue);
		pen.setWidth(2);
		QCPScatterStyle scatterStyle;
		scatterStyle.setShape(QCPScatterStyle::ssCircle);
		scatterStyle.setPen(pen);
		scatterStyle.setSize(8);
		m_positionScannerMarker->setScatterStyle(scatterStyle);
	}
	m_positionScannerMarker->setData(QVector<double>{positionScannerDisplay.x}, QVector<double>{positionScannerDisplay.y});
	ui->customplot_brightfield->replot();
}

bool BrillouinAcquisition::isBrightfieldRotated90() const {
	return m_brightfieldViewRotation == BrightfieldViewRotation::Rot90
		|| m_brightfieldViewRotation == BrightfieldViewRotation::Rot270;
}

bool BrillouinAcquisition::hasBrightfieldViewTransform() const {
	return m_brightfieldViewRotation != BrightfieldViewRotation::Rot0
		|| m_brightfieldMirrorHorizontal
		|| m_brightfieldMirrorVertical;
}

int BrillouinAcquisition::brightfieldDisplayWidth() const {
	return isBrightfieldRotated90()
		? std::max(1, m_brightfieldRawHeight)
		: std::max(1, m_brightfieldRawWidth);
}

int BrillouinAcquisition::brightfieldDisplayHeight() const {
	return isBrightfieldRotated90()
		? std::max(1, m_brightfieldRawWidth)
		: std::max(1, m_brightfieldRawHeight);
}

POINT2 BrillouinAcquisition::brightfieldRawToDisplay(POINT2 point) const {
	const auto rawWidth = std::max(1, m_brightfieldRawWidth);
	const auto rawHeight = std::max(1, m_brightfieldRawHeight);
	POINT2 displayPoint;
	switch (m_brightfieldViewRotation) {
	case BrightfieldViewRotation::Rot90:
		displayPoint = POINT2{ rawHeight - point.y + 1.0, point.x };
		break;
	case BrightfieldViewRotation::Rot180:
		displayPoint = POINT2{ rawWidth - point.x + 1.0, rawHeight - point.y + 1.0 };
		break;
	case BrightfieldViewRotation::Rot270:
		displayPoint = POINT2{ point.y, rawWidth - point.x + 1.0 };
		break;
	case BrightfieldViewRotation::Rot0:
	default:
		displayPoint = point;
		break;
	}
	if (m_brightfieldMirrorHorizontal) {
		displayPoint.x = brightfieldDisplayWidth() - displayPoint.x + 1.0;
	}
	if (m_brightfieldMirrorVertical) {
		displayPoint.y = brightfieldDisplayHeight() - displayPoint.y + 1.0;
	}
	return displayPoint;
}

POINT2 BrillouinAcquisition::brightfieldDisplayToRaw(POINT2 point) const {
	const auto rawWidth = std::max(1, m_brightfieldRawWidth);
	const auto rawHeight = std::max(1, m_brightfieldRawHeight);
	auto unmirroredPoint = point;
	if (m_brightfieldMirrorHorizontal) {
		unmirroredPoint.x = brightfieldDisplayWidth() - unmirroredPoint.x + 1.0;
	}
	if (m_brightfieldMirrorVertical) {
		unmirroredPoint.y = brightfieldDisplayHeight() - unmirroredPoint.y + 1.0;
	}
	switch (m_brightfieldViewRotation) {
	case BrightfieldViewRotation::Rot90:
		return POINT2{ unmirroredPoint.y, rawHeight - unmirroredPoint.x + 1.0 };
	case BrightfieldViewRotation::Rot180:
		return POINT2{ rawWidth - unmirroredPoint.x + 1.0, rawHeight - unmirroredPoint.y + 1.0 };
	case BrightfieldViewRotation::Rot270:
		return POINT2{ rawWidth - unmirroredPoint.y + 1.0, unmirroredPoint.x };
	case BrightfieldViewRotation::Rot0:
	default:
		return unmirroredPoint;
	}
}

void BrillouinAcquisition::on_rangeLower_valueChanged(int value) {
	m_BrillouinPlot.cLim.lower = value;
	updatePlot(m_BrillouinPlot);
}

void BrillouinAcquisition::on_rangeUpper_valueChanged(int value) {
	m_BrillouinPlot.cLim.upper = value;
	updatePlot(m_BrillouinPlot);
}

void BrillouinAcquisition::on_rangeLowerODT_valueChanged(int value) {
	m_ODTPlot.cLim.lower = value;
	updatePlot(m_ODTPlot);
}

void BrillouinAcquisition::on_rangeUpperODT_valueChanged(int value) {
	m_ODTPlot.cLim.upper = value;
	updatePlot(m_ODTPlot);
}

void BrillouinAcquisition::updatePlot(const PLOT_SETTINGS& plotSettings) {
	plotSettings.colorMap->setDataRange(plotSettings.cLim);
	plotSettings.plotHandle->replot();
	updateCLimRange(plotSettings.lowerBox, plotSettings.upperBox, plotSettings.cLim);
}

void BrillouinAcquisition::updateCLimRange(QSpinBox *lower, QSpinBox *upper, QCPRange range) {
	const QSignalBlocker blocker1(lower);
	lower->setValue(range.lower);
	lower->setMaximum(range.upper);
	const QSignalBlocker blocker2(upper);
	upper->setValue(range.upper);
	upper->setMinimum(range.lower);
}

void BrillouinAcquisition::xAxisRangeChangedODT(const QCPRange &newRange) {
	m_ODTPlot.plotHandle->xAxis->setRange(newRange.bounded(1, brightfieldDisplayWidth()));
}

void BrillouinAcquisition::yAxisRangeChangedODT(const QCPRange &newRange) {
	m_ODTPlot.plotHandle->yAxis->setRange(newRange.bounded(1, brightfieldDisplayHeight()));
}

void BrillouinAcquisition::xAxisRangeChanged(QCPRange &newRange) {
	// We need rounded values as this represents pixels
	newRange.lower = floor(newRange.lower);
	newRange.upper = ceil(newRange.upper);

	ui->customplot->xAxis->setRange(newRange.bounded(1, m_cameraOptions.ROIWidthLimits[1]));
	m_deviceSettings.camera.roi.left = newRange.lower;
	m_deviceSettings.camera.roi.width_physical = newRange.upper - newRange.lower + 1;
	settingsCameraUpdate(ROI_SOURCE::PLOT);
}

void BrillouinAcquisition::yAxisRangeChanged(QCPRange &newRange) {
	// We need rounded values as this represents pixels
	newRange.lower = floor(newRange.lower);
	newRange.upper = ceil(newRange.upper);

	ui->customplot->yAxis->setRange(newRange.bounded(1, m_cameraOptions.ROIHeightLimits[1]));
	m_deviceSettings.camera.roi.top = m_cameraOptions.ROIHeightLimits[1] - newRange.upper + 1;
	m_deviceSettings.camera.roi.height_physical = newRange.upper - newRange.lower + 1;
	settingsCameraUpdate(ROI_SOURCE::PLOT);
}

void BrillouinAcquisition::on_ROILeft_valueChanged(int left) {
	m_deviceSettings.camera.roi.left = left;
	settingsCameraUpdate(ROI_SOURCE::BOX);
}

void BrillouinAcquisition::on_ROIWidth_valueChanged(int width) {
	m_deviceSettings.camera.roi.width_physical = width;
	settingsCameraUpdate(ROI_SOURCE::BOX);
}

void BrillouinAcquisition::on_ROITop_valueChanged(int top) {
	m_deviceSettings.camera.roi.top = top;
	settingsCameraUpdate(ROI_SOURCE::BOX);
}

void BrillouinAcquisition::on_ROIHeight_valueChanged(int height) {
	m_deviceSettings.camera.roi.height_physical = height;
	settingsCameraUpdate(ROI_SOURCE::BOX);
}

void BrillouinAcquisition::settingsCameraUpdate(int source) {
	// check that values are valid
	// start must be >= than 1 and <= than (imageSize - minROIsize + 1)
	// ROIsize must be within [minROIsize, maxROIsize]
	// end must be >= than <= than the size of the image

	// check these requirements
	// for x
	std::vector<AT_64> xIn = { m_deviceSettings.camera.roi.left, m_deviceSettings.camera.roi.width_physical };
	std::vector<AT_64> xOut = checkROI(xIn, m_cameraOptions.ROIWidthLimits);
	bool xChanged = (xIn != xOut);
	if (xChanged) {
		m_deviceSettings.camera.roi.left = xOut[0];
		m_deviceSettings.camera.roi.width_physical = xOut[1];
	}

	//for y
	std::vector<AT_64> yIn = { m_deviceSettings.camera.roi.top, m_deviceSettings.camera.roi.height_physical };
	std::vector<AT_64> yOut = checkROI(yIn, m_cameraOptions.ROIHeightLimits);
	bool yChanged = (yIn != yOut);
	if (yChanged) {
		m_deviceSettings.camera.roi.top = yOut[0];
		m_deviceSettings.camera.roi.height_physical = yOut[1];
	}

	// set values of manual input fields
	if (source == ROI_SOURCE::PLOT || xChanged) {
		// dont retrigger a round of checking
		const QSignalBlocker blocker1(ui->ROILeft);
		ui->ROILeft->setValue(m_deviceSettings.camera.roi.left);
		const QSignalBlocker blocker2(ui->ROIWidth);
		ui->ROIWidth->setValue(m_deviceSettings.camera.roi.width_physical);
	}
	if (source == ROI_SOURCE::PLOT || yChanged) {
		// dont retrigger a round of checking
		const QSignalBlocker blocker1(ui->ROITop);
		ui->ROITop->setValue(m_deviceSettings.camera.roi.top);
		const QSignalBlocker blocker2(ui->ROIHeight);
		ui->ROIHeight->setValue(m_deviceSettings.camera.roi.height_physical);
	}
	// set plot range
	//ui->customplot->yAxis->setRange(newRange.bounded(1, m_cameraOptions.ROIHeightLimits[1]));
	if (source == ROI_SOURCE::BOX || xChanged) {
		ui->customplot->xAxis->setRange(QCPRange(m_deviceSettings.camera.roi.left, m_deviceSettings.camera.roi.left + m_deviceSettings.camera.roi.width_physical - 1));
	}
	if (source == ROI_SOURCE::BOX || yChanged) {
		double l = m_cameraOptions.ROIHeightLimits[1] - m_deviceSettings.camera.roi.top - m_deviceSettings.camera.roi.height_physical + 2;
		auto l1 = m_deviceSettings.camera.roi.bottom;
		double h = m_cameraOptions.ROIHeightLimits[1] - m_deviceSettings.camera.roi.top + 1;
		auto h1 = m_deviceSettings.camera.roi.bottom + m_deviceSettings.camera.roi.height_physical - 1;
		ui->customplot->yAxis->setRange(QCPRange(l, h));
	}
	if (source == ROI_SOURCE::BOX || xChanged || yChanged) {
		ui->customplot->replot();
	}
	// Crop/zoom just changed - the spectral proxy ROI overlay's "current frame" reference
	// (currentSpectralCameraRoi()) tracks this live, so refresh it now instead of leaving it to
	// silently resync at the next measurement Start.
	refreshSpectralProxyRoiRects();
}

std::vector<AT_64> BrillouinAcquisition::checkROI(std::vector<AT_64> values, std::vector<AT_64> requirements) {
	// check that lower value is in valid range
	if (values[0] < 1) {
	// must be larger than 0, counting camera pixels starts at 1
		values[0] = 1;
	} else if (values[0] > (requirements[1] - requirements[0] + 1)) {
	// must be equal to or smaller than the image size minus the minimal ROI size + 1
		values[0] = requirements[1] - requirements[0] + 1;
	}
	// check that ROI size is valid
	if (values[1] < requirements[0]) {
	// size must at least be equal to the minmal allowed ROI size
		values[1] = requirements[0];
	} else if (values[1] > (requirements[1] - values[0] + 1)) {
		values[1] = requirements[1] - values[0] + 1;
	}
	return values;
}

/*
 * Brillouin camera settings
 */
 // Binning
void BrillouinAcquisition::on_binning_currentIndexChanged(const QString& text) {
	m_Brillouin->settings.camera.roi.binning = text.toStdWString();
	applyCameraSettings();
}
// Readout parameters
void BrillouinAcquisition::on_pixelReadoutRate_currentIndexChanged(const QString& text) {
	m_Brillouin->settings.camera.readout.pixelReadoutRate = text.toStdWString();
	applyCameraSettings();
}

void BrillouinAcquisition::on_preAmpGain_currentIndexChanged(const QString& text) {
	m_Brillouin->settings.camera.readout.preAmpGain = text.toStdWString();
	applyCameraSettings();
}

void BrillouinAcquisition::on_pixelEncoding_currentIndexChanged(const QString& text) {
	m_Brillouin->settings.camera.readout.pixelEncoding = text.toStdWString();
	applyCameraSettings();
}

void BrillouinAcquisition::on_cycleMode_currentIndexChanged(const QString& text) {
	m_Brillouin->settings.camera.readout.cycleMode = text.toStdWString();
	applyCameraSettings();
}

void BrillouinAcquisition::applyCameraSettings() {
	if (!m_andor->m_isPreviewRunning && !m_andor->m_isAcquisitionRunning) {
		m_andor->setSettings(m_Brillouin->settings.camera);
	}
}


void BrillouinAcquisition::updatePlotLimits(const PLOT_SETTINGS& plotSettings,	const CAMERA_OPTIONS& options, const CAMERA_ROI& roi) {
	// set the properties of the colormap to the correct values of the preview buffer
	auto displayWidth = (int)roi.width_binned;
	auto displayHeight = (int)roi.height_binned;
	auto xRange = QCPRange(roi.left, roi.left + roi.width_physical - 1);
	auto yRange = QCPRange(roi.bottom, roi.bottom + roi.height_physical - 1);
	if (plotSettings.plotHandle == ui->customplot_brightfield) {
		m_brightfieldRawWidth = std::max(1, (int)roi.width_binned);
		m_brightfieldRawHeight = std::max(1, (int)roi.height_binned);
		displayWidth = brightfieldDisplayWidth();
		displayHeight = brightfieldDisplayHeight();
		xRange = QCPRange(1, displayWidth);
		yRange = QCPRange(1, displayHeight);
	}
	plotSettings.colorMap->data()->setSize(displayWidth, displayHeight);
	plotSettings.colorMap->data()->setRange(xRange, yRange);

	QCPRange xRangeCurrent = plotSettings.plotHandle->xAxis->range();
	QCPRange yRangeCurrent = plotSettings.plotHandle->yAxis->range();

	QCPRange xRangeNew = QCPRange(
		floor(simplemath::maximum({ xRangeCurrent.lower, xRange.lower })),
		ceil(simplemath::minimum({ xRangeCurrent.upper, xRange.upper }))
	);
	QCPRange yRangeNew = QCPRange(
		floor(simplemath::maximum({ yRangeCurrent.lower, yRange.lower })),
		ceil(simplemath::minimum({ yRangeCurrent.upper, yRange.upper }))
	);

	plotSettings.plotHandle->xAxis->setRange(xRangeNew);
	plotSettings.plotHandle->yAxis->setRange(yRangeNew);
}

void BrillouinAcquisition::showPreviewRunning(bool isRunning) {
	if (isRunning) {
		ui->camera_playPause->setText("Stop");
	} else {
		ui->camera_playPause->setText("Play");
	}
	startPreview(isRunning);
}

void BrillouinAcquisition::showBrightfieldPreviewRunning(bool isRunning) {
	if (isRunning) {
		ui->camera_playPause_brightfield->setText("Stop");
	} else {
		ui->camera_playPause_brightfield->setText("Play");
	}
	startBrightfieldPreview(isRunning);
}

void BrillouinAcquisition::showFluorescencePreviewRunning(const FLUORESCENCE_MODE& mode) {
	// reset all preview buttons
	ui->fluoBluePreview->setText("Preview");
	ui->fluoGreenPreview->setText("Preview");
	ui->fluoRedPreview->setText("Preview");
	ui->fluoBrightfieldPreview->setText("Preview");

	// show currently running mode
	switch (mode) {
	case FLUORESCENCE_MODE::BLUE:
		ui->fluoBluePreview->setText("Stop");
		break;
	case FLUORESCENCE_MODE::GREEN:
		ui->fluoGreenPreview->setText("Stop");
		break;
	case FLUORESCENCE_MODE::RED:
		ui->fluoRedPreview->setText("Stop");
		break;
	case FLUORESCENCE_MODE::BRIGHTFIELD:
		ui->fluoBrightfieldPreview->setText("Stop");
		break;
	}
}

void BrillouinAcquisition::startPreview(bool isRunning) {
	// if preview was not running, start it, else leave it running (don't start it twice)
	if (!m_previewRunning && isRunning) {
		updateImageBrillouin();
	}
	m_previewRunning = isRunning;
}

void BrillouinAcquisition::startBrightfieldPreview(bool isRunning) {
	// if preview was not running, start it, else leave it running (don't start it twice)
	if (!m_brightfieldPreviewRunning && isRunning) {
		updateImageODT();
	}
	m_brightfieldPreviewRunning = isRunning;
}

void BrillouinAcquisition::updateImageBrillouin() {
	// Abort updating images when camera is not available
	if (m_andor == nullptr) {
		m_previewRunning = false;
		return;
	}
	updateImage(m_andor->m_previewBuffer, &m_BrillouinPlot);
}

void BrillouinAcquisition::updateImageODT() {
	// Abort updating images when camera is not available
	if (m_brightfieldCamera == nullptr) {
		m_brightfieldPreviewRunning = false;
		return;
	}
	updateImage(m_brightfieldCamera->m_previewBuffer, &m_ODTPlot);
}

template <typename T>
void BrillouinAcquisition::updateImage(PreviewBuffer<T>* previewBuffer, PLOT_SETTINGS *plotSettings) {
	QMetaObject::invokeMethod(
		m_converter,
		[&m_converter = m_converter, previewBuffer, plotSettings]() {
			m_converter->convert(previewBuffer, plotSettings);
		},
		Qt::QueuedConnection
	);
}

void BrillouinAcquisition::plot(PLOT_SETTINGS* plotSettings, long long dim_x, long long dim_y, const std::vector<unsigned char>& unpackedBuffer) {
	plotting(plotSettings, dim_x, dim_y, unpackedBuffer);
}

void BrillouinAcquisition::plot(PLOT_SETTINGS* plotSettings, long long dim_x, long long dim_y, const std::vector<unsigned short>& unpackedBuffer) {
	plotting(plotSettings, dim_x, dim_y, unpackedBuffer);
}

void BrillouinAcquisition::plot(PLOT_SETTINGS* plotSettings, long long dim_x, long long dim_y, const std::vector<double>& unpackedBuffer) {
	plotting(plotSettings, dim_x, dim_y, unpackedBuffer);
}

void BrillouinAcquisition::plot(PLOT_SETTINGS* plotSettings, long long dim_x, long long dim_y, const std::vector<float>& unpackedBuffer) {
	plotting(plotSettings, dim_x, dim_y, unpackedBuffer);
}

void BrillouinAcquisition::plot(PLOT_SETTINGS* plotSettings, long long dim_x, long long dim_y, const std::vector<int>& unpackedBuffer) {
	plotting(plotSettings, dim_x, dim_y, unpackedBuffer);
}

template <typename T>
void BrillouinAcquisition::plotting(PLOT_SETTINGS* plotSettings, long long dim_x, long long dim_y, const std::vector<T>& unpackedBuffer) {
	// images are given row by row, starting at the top left
	const bool transformBrightfield = plotSettings == &m_ODTPlot && hasBrightfieldViewTransform();
	if (plotSettings == &m_ODTPlot) {
		m_brightfieldRawWidth = std::max(1, (int)dim_x);
		m_brightfieldRawHeight = std::max(1, (int)dim_y);
		if (transformBrightfield) {
			plotSettings->colorMap->data()->setSize(brightfieldDisplayWidth(), brightfieldDisplayHeight());
			plotSettings->colorMap->data()->setRange(QCPRange(1, brightfieldDisplayWidth()), QCPRange(1, brightfieldDisplayHeight()));
		} else if (plotSettings->colorMap->data()->keySize() != dim_x || plotSettings->colorMap->data()->valueSize() != dim_y) {
			// Keep the color map's size/axis-range in sync with the frame that's actually
			// arriving, rather than relying solely on the separate, signal-driven
			// updatePlotLimits() (Camera::s_previewBufferSettingsChanged) - that signal and
			// this per-frame delivery are independent, so a preset/ROI switch (e.g. entering
			// WAITFORSURFACEREVIEW's SCAN_BRIGHTFIELD preset) could otherwise deliver
			// differently-shaped frames before/without the size ever being refreshed, leaving
			// the display's aspect ratio and pixel bounds stale relative to the real frame.
			plotSettings->colorMap->data()->setSize((int)dim_x, (int)dim_y);
			plotSettings->colorMap->data()->setRange(QCPRange(1, (double)dim_x), QCPRange(1, (double)dim_y));
		}
	}
	int tIndex{ 0 };
	for (gsl::index yIndex{ 0 }; yIndex < dim_y; ++yIndex) {
		for (gsl::index xIndex{ 0 }; xIndex < dim_x; ++xIndex) {
			tIndex = yIndex * dim_x + xIndex;
			if (transformBrightfield) {
				const auto rawPoint = POINT2{ (double)xIndex + 1.0, (double)(dim_y - yIndex) };
				const auto displayPoint = brightfieldRawToDisplay(rawPoint);
				plotSettings->colorMap->data()->setCell((int)displayPoint.x - 1, (int)displayPoint.y - 1, unpackedBuffer[tIndex]);
			} else {
				plotSettings->colorMap->data()->setCell(xIndex, dim_y - yIndex - 1, unpackedBuffer[tIndex]);
			}
		}
	}
	if (plotSettings->autoscale) {
		plotSettings->colorMap->rescaleDataRange(true);
		plotSettings->cLim = plotSettings->colorMap->dataRange();
		(plotSettings->dataRangeCallback)(plotSettings->cLim);
	}
	if (plotSettings == &m_BrillouinPlot) {
		refreshSpectralProxyRoiRects();
	}
	plotSettings->plotHandle->replot();
}

void BrillouinAcquisition::on_actionConnect_Camera_triggered() {
	if (m_andor->getConnectionStatus()) {
		QMetaObject::invokeMethod(
			m_andor,
			[&m_andor = m_andor]() {
				m_andor->disconnectDevice();
			},
			Qt::QueuedConnection
		);
	} else {
		QMetaObject::invokeMethod(
			m_andor,
			[&m_andor = m_andor]() {
				m_andor->connectDevice();
			},
			Qt::QueuedConnection
		);
	}
}

void BrillouinAcquisition::cameraConnectionChanged(bool isConnected) {
	if (isConnected) {
		ui->actionConnect_Camera->setText("Disconnect Camera");
		ui->settingsWidget->setTabIcon(0, m_icons.standby);
		ui->actionEnable_Cooling->setEnabled(true);
		ui->camera_playPause->setEnabled(true);
		ui->camera_singleShot->setEnabled(true);
		// switch on cooling automatically
		QMetaObject::invokeMethod(
			m_andor,
			[&m_andor = m_andor]() {
				m_andor->setSensorCooling(true);
			},
			Qt::QueuedConnection
		);
		this->restoreCameraSettings();
	} else {
		ui->actionConnect_Camera->setText("Connect Camera");
		ui->actionEnable_Cooling->setText("Enable Cooling");
		ui->settingsWidget->setTabIcon(0, m_icons.disconnected);
		ui->actionEnable_Cooling->setEnabled(false);
		ui->camera_playPause->setEnabled(false);
		ui->camera_singleShot->setEnabled(false);
	}
}

void BrillouinAcquisition::restoreCameraSettings() {
	QSettings settings(QSettings::IniFormat, QSettings::UserScope,
		kSettingsOrg, kSettingsApp);

	settings.beginGroup("devices-settings");
	auto left = settings.value("brillouin-camera-roi-left", 1).toInt();
	this->on_ROILeft_valueChanged(left);
	auto top = settings.value("brillouin-camera-roi-top", 1).toInt();
	this->on_ROITop_valueChanged(top);
	auto width_phsical = settings.value("brillouin-camera-roi-width-physical").toInt();
	this->on_ROIWidth_valueChanged(width_phsical);
	auto height_phsical = settings.value("brillouin-camera-roi-height-physical").toInt();
	this->on_ROIHeight_valueChanged(height_phsical);
	auto exposureTime = settings.value("brillouin-camera-exposure-time", 0.5).toDouble();
	ui->exposureTime->setValue(exposureTime);
	auto frameCount = settings.value("brillouin-camera-frame-count", 2).toInt();
	ui->frameCount->setValue(frameCount);

	settings.endGroup();
}

void BrillouinAcquisition::showNoCameraFound() {
	QMessageBox::critical(this, "Selected camera not found.", "The selected camera was not found. Switch on the camera and restart the program or select a different camera.");
}

void BrillouinAcquisition::on_actionEnable_Cooling_triggered() {
	if (m_andor->getConnectionStatus()) {
		if (m_andor->getSensorCooling()) {
			QMetaObject::invokeMethod(
				m_andor,
				[&m_andor = m_andor]() {
					m_andor->setSensorCooling(false);
				},
				Qt::QueuedConnection
			);
		} else {
			QMetaObject::invokeMethod(
				m_andor,
				[&m_andor = m_andor]() {
					m_andor->setSensorCooling(true);
				},
				Qt::QueuedConnection
			);
		}
	}
}

void BrillouinAcquisition::cameraCoolingChanged(bool isCooling) {
	if (isCooling) {
		ui->actionEnable_Cooling->setText("Disable Cooling");
		ui->settingsWidget->setTabIcon(0, m_icons.cooling);
	} else {
		ui->actionEnable_Cooling->setText("Enable Cooling");
		ui->settingsWidget->setTabIcon(0, m_icons.standby);
	}
}

void BrillouinAcquisition::on_actionConnect_Stage_triggered() {
	if (m_scanControl->getConnectionStatus()) {
		QMetaObject::invokeMethod(
			m_scanControl,
			[&m_scanControl = m_scanControl]() {
				m_scanControl->disconnectDevice();
			},
			Qt::QueuedConnection
		);
	} else {
		QMetaObject::invokeMethod(
			m_scanControl,
			[&m_scanControl = m_scanControl]() {
				m_scanControl->connectDevice();
			},
			Qt::QueuedConnection
		);
	}
}

void BrillouinAcquisition::microscopeConnectionChanged(bool isConnected) {
	if (isConnected) {
		ui->actionConnect_Stage->setText("Disconnect Microscope");
		ui->settingsWidget->setTabIcon(1, m_icons.ready);
		ui->settingsWidget->setTabIcon(2, m_icons.ready);
	} else {
		ui->actionConnect_Stage->setText("Connect Microscope");
		ui->settingsWidget->setTabIcon(1, m_icons.disconnected);
		ui->settingsWidget->setTabIcon(2, m_icons.disconnected);
	}
}

void BrillouinAcquisition::on_actionConnect_Brightfield_camera_triggered() {
	if (m_brightfieldCamera->getConnectionStatus()) {
		QMetaObject::invokeMethod(
			m_brightfieldCamera,
			[&m_brightfieldCamera = m_brightfieldCamera]() {
				m_brightfieldCamera->disconnectDevice();
			},
			Qt::QueuedConnection
		);
	} else {
		QMetaObject::invokeMethod(
			m_brightfieldCamera,
			[&m_brightfieldCamera = m_brightfieldCamera]() {
				m_brightfieldCamera->connectDevice();
			},
			Qt::QueuedConnection
		);
	}
}

void BrillouinAcquisition::brightfieldCameraConnectionChanged(bool isConnected) {
	if (isConnected) {
		ui->actionConnect_Brightfield_camera->setText("Disconnect Brightfield Camera");
		ui->brightfieldImage->show();
		ui->camera_playPause_brightfield->setEnabled(true);
		ui->settingsWidget->setTabIcon(3, m_icons.ready);
	} else {
		ui->actionConnect_Brightfield_camera->setText("Connect Brightfield Camera");
		ui->brightfieldImage->hide();
		ui->camera_playPause_brightfield->setEnabled(false);
		ui->settingsWidget->setTabIcon(3, m_icons.disconnected);
	}
	updateBrillouinSettings();
}

void BrillouinAcquisition::on_camera_playPause_brightfield_clicked() {
	if (!m_brightfieldCamera->m_isPreviewRunning) {
		QMetaObject::invokeMethod(
			m_brightfieldCamera,
			[&m_brightfieldCamera = m_brightfieldCamera]() {
				m_brightfieldCamera->startPreview();
			},
			Qt::QueuedConnection
		);
	} else {
		m_brightfieldCamera->m_stopPreview = true;
		m_Fluorescence->startStopPreview(FLUORESCENCE_MODE::NONE);
	}
}

void BrillouinAcquisition::on_actionSettings_Stage_triggered() {
	m_scanControlDropdown->setCurrentIndex((int)m_scanControllerType);

	auto numberCameras_Brillouin = m_andor->getNumberCameras();
	m_numberCameras_BrillouinDropdown->clear();
	for (gsl::index i{ 0 }; i < numberCameras_Brillouin; i++) {
		m_numberCameras_BrillouinDropdown->insertItem(i, QString::number(i));
	}
	m_numberCameras_BrillouinDropdown->setCurrentIndex((int)m_andor->getCameraNumber());

	m_settingsDialog->show();
}

void BrillouinAcquisition::saveSettings() {
	if (m_scanControllerType != m_scanControllerTypeTemporary) {
		m_scanControllerType = m_scanControllerTypeTemporary;
		initScanControl();
		initBeampathButtons();
	}
	if (m_cameraType != m_cameraTypeTemporary) {
		m_cameraType = m_cameraTypeTemporary;
		initCamera();
	}
	if (m_cameraBrillouinType != m_cameraBrillouinTypeTemporary ||
		m_cameraBrillouinNumber != m_cameraBrillouinNumberTemporary) {
		m_cameraBrillouinType = m_cameraBrillouinTypeTemporary;
		m_cameraBrillouinNumber = m_cameraBrillouinNumberTemporary;
		initCameraBrillouin();
	}
	m_settingsDialog->hide();
}

void BrillouinAcquisition::cancelSettings() {
	m_scanControllerTypeTemporary = m_scanControllerType;
	m_cameraTypeTemporary = m_cameraType;
	m_cameraBrillouinTypeTemporary = m_cameraBrillouinType;
	m_settingsDialog->hide();
}

void BrillouinAcquisition::initSettingsDialog() {

	if (m_settingsDialog) {
		m_settingsDialog->deleteLater();
		m_settingsDialog = nullptr;
	}
	m_settingsDialog = new QDialog(this, Qt::WindowTitleHint | Qt::WindowCloseButtonHint);
	m_settingsDialog->setWindowTitle("Settings");
	m_settingsDialog->setWindowModality(Qt::ApplicationModal);

	QVBoxLayout* vLayout = new QVBoxLayout(m_settingsDialog);
	/*
	 * Widget for Brillouin camera selection
	 */
	m_cameraBrillouinTypeTemporary = m_cameraBrillouinType;

	QWidget* cameraBrillouinWidget = new QWidget();
	cameraBrillouinWidget->setMinimumHeight(120);
	cameraBrillouinWidget->setMinimumWidth(250);
	vLayout->addWidget(cameraBrillouinWidget);

	QGroupBox* camBrillouinBox = new QGroupBox(cameraBrillouinWidget);
	camBrillouinBox->setTitle("Brillouin camera");
	camBrillouinBox->setMinimumHeight(50);
	camBrillouinBox->setMinimumWidth(250);

	QVBoxLayout* vCameraLayout = new QVBoxLayout(camBrillouinBox);

	QWidget* cameraTypeWidget = new QWidget();
	vCameraLayout->addWidget(cameraTypeWidget);

	QHBoxLayout* camBrillouinLayout = new QHBoxLayout(cameraTypeWidget);

	QLabel* camera_BrillouinLabel = new QLabel("Camera type");
	camBrillouinLayout->addWidget(camera_BrillouinLabel);

	m_camera_BrillouinDropdown = new QComboBox();
	camBrillouinLayout->addWidget(m_camera_BrillouinDropdown);
	gsl::index i{ 0 };
	for (auto type : CAMERA_BRILLOUIN_DEVICE_NAMES) {
		m_camera_BrillouinDropdown->insertItem(i, QString::fromStdString(type));
		i++;
	}
	m_camera_BrillouinDropdown->setCurrentIndex((int)m_cameraBrillouinType);

	if (m_andor) {
		QWidget* cameraNumberWidget = new QWidget();
		vCameraLayout->addWidget(cameraNumberWidget);
		QHBoxLayout* numberCameras_BrillouinLayout = new QHBoxLayout(cameraNumberWidget);

		QLabel* numberCameras_BrillouinLabel = new QLabel("Camera number");
		numberCameras_BrillouinLayout->addWidget(numberCameras_BrillouinLabel);

		m_numberCameras_BrillouinDropdown = new QComboBox();
		numberCameras_BrillouinLayout->addWidget(m_numberCameras_BrillouinDropdown);
		auto numberCameras_Brillouin = m_andor->getNumberCameras();
		for (gsl::index i{ 0 }; i < numberCameras_Brillouin; i++) {
			m_numberCameras_BrillouinDropdown->insertItem(i, QString::number(i));
		}
		m_numberCameras_BrillouinDropdown->setCurrentIndex((int)m_andor->getCameraNumber());
	}

	static QMetaObject::Connection connection = QWidget::connect<void(QComboBox::*)(int)>(
		m_camera_BrillouinDropdown,
		&QComboBox::currentIndexChanged,
		this,
		[this](int index) { selectCameraBrillouinDevice(index); }
	);

	connection = QWidget::connect<void(QComboBox::*)(int)>(
		m_numberCameras_BrillouinDropdown,
		&QComboBox::currentIndexChanged,
		this,
		[this](int index) { selectCameraBrillouinNumber(index); }
	);

	/*
	 * Widget for scan controller selection
	 */
	m_scanControllerTypeTemporary = m_scanControllerType;

	QWidget *daqWidget = new QWidget();
	daqWidget->setMinimumHeight(60);
	daqWidget->setMinimumWidth(250);
	QGroupBox *box = new QGroupBox(daqWidget);
	box->setTitle("Scanning device");
	box->setMinimumHeight(50);
	box->setMinimumWidth(250);

	vLayout->addWidget(daqWidget);

	QHBoxLayout *layout = new QHBoxLayout(box);

	QLabel *label = new QLabel("Currently selected device");
	layout->addWidget(label);

	m_scanControlDropdown = new QComboBox();
	layout->addWidget(m_scanControlDropdown);
	i = 0;
	for (auto type : ScanControl::SCAN_DEVICE_NAMES) {
		m_scanControlDropdown->insertItem(i, QString::fromStdString(type));
		i++;
	}
	m_scanControlDropdown->setCurrentIndex((int)m_scanControllerType);

	connection = QWidget::connect<void(QComboBox::*)(int)>(
		m_scanControlDropdown,
		&QComboBox::currentIndexChanged,
		this,
		[this](int index) { selectScanningDevice(index); }
	);

	/*
	 * Widget for ODT/Fluorescence camera selection
	 */
	m_cameraTypeTemporary = m_cameraType;

	QWidget *cameraWidget = new QWidget();
	cameraWidget->setMinimumHeight(60);
	cameraWidget->setMinimumWidth(250);
	QGroupBox *camBox = new QGroupBox(cameraWidget);
	camBox->setTitle("ODT/Fluorescence camera");
	camBox->setMinimumHeight(50);
	camBox->setMinimumWidth(250);

	vLayout->addWidget(cameraWidget);

	QHBoxLayout *camLayout = new QHBoxLayout(camBox);

	QLabel *camLabel = new QLabel("Camera type");
	camLayout->addWidget(camLabel);

	m_cameraDropdown = new QComboBox();
	camLayout->addWidget(m_cameraDropdown);
	i = 0;
	for (auto type : CAMERA_DEVICE_NAMES) {
		m_cameraDropdown->insertItem(i, QString::fromStdString(type));
		i++;
	}
	m_cameraDropdown->setCurrentIndex((int)m_cameraType);

	connection = QWidget::connect<void(QComboBox::*)(int)>(
		m_cameraDropdown,
		&QComboBox::currentIndexChanged,
		this,
		[this](int index) { selectCameraDevice(index); }
	);

	/*
	 * Ok and Cancel buttons
	 */
	QWidget *buttonWidget = new QWidget();
	vLayout->addWidget(buttonWidget);

	QHBoxLayout *buttonLayout = new QHBoxLayout(buttonWidget);
	buttonLayout->setMargin(0);

	QPushButton *okButton = new QPushButton();
	okButton->setText(tr("OK"));
	okButton->setMinimumWidth(60);
	okButton->setMaximumWidth(60);
	buttonLayout->addWidget(okButton);
	buttonLayout->setAlignment(okButton, Qt::AlignRight);

	connection = QWidget::connect(
		okButton,
		&QPushButton::clicked,
		this,
		[this]() { saveSettings(); }
	);

	QPushButton *cancelButton = new QPushButton();
	cancelButton->setText(tr("Cancel"));
	cancelButton->setMinimumWidth(60);
	cancelButton->setMaximumWidth(60);
	buttonLayout->addWidget(cancelButton);

	connection = QWidget::connect(
		cancelButton,
		&QPushButton::clicked,
		this,
		[this]() { cancelSettings(); }
	);

	m_settingsDialog->layout()->setSizeConstraint(QLayout::SetFixedSize);
}

void BrillouinAcquisition::selectScanningDevice(int index) {
	m_scanControllerTypeTemporary = (ScanControl::SCAN_DEVICE)index;
}

void BrillouinAcquisition::selectCameraDevice(int index) {
	m_cameraTypeTemporary = (CAMERA_DEVICE)index;
}

void BrillouinAcquisition::selectCameraBrillouinDevice(int index) {
	m_cameraBrillouinTypeTemporary = (CAMERA_BRILLOUIN_DEVICE)index;
}

void BrillouinAcquisition::selectCameraBrillouinNumber(int index) {
	m_cameraBrillouinNumberTemporary = index;
}

void BrillouinAcquisition::on_action_Voltage_calibration_acquire_triggered() {
	QMetaObject::invokeMethod(
		m_voltageCalibration,
		[&m_voltageCalibration = m_voltageCalibration]() {
			m_voltageCalibration->startRepetitions();
		},
		Qt::AutoConnection
	);
}

void BrillouinAcquisition::on_action_Voltage_calibration_load_triggered() {
	m_voltageCalibrationFilePath = QFileDialog::getOpenFileName(this, tr("Select Voltage-Position map"),
		QString::fromStdString(m_voltageCalibrationFilePath), tr("Calibration map (*.h5)")).toStdString();
	m_voltageCalibration->load(m_voltageCalibrationFilePath);
}

void BrillouinAcquisition::on_action_Scale_calibration_acquire_triggered() {
	// setupUi()/connect() must run exactly once per Ui object - calling setupUi() again on an
	// already-set-up QDialog builds an entirely new, unparented set of child widgets each time
	// (the existing layout refuses to be replaced), leaving the *visible* dialog showing stale
	// widgets that the (now repointed) Ui struct - and everything wired to it - no longer
	// touches; reconnecting every "connect()" here on every open would additionally pile up
	// duplicate signal/slot connections. So: only the widget construction/wiring happens inside
	// this guard, everything below it re-runs on every open to refresh values.
	if (!m_scaleCalibrationDialog) {
		m_scaleCalibrationDialog = new QDialog(this, Qt::WindowTitleHint | Qt::WindowCloseButtonHint);
		m_scaleCalibrationDialogUi.setupUi(m_scaleCalibrationDialog);
		m_scaleCalibrationDialog->setWindowTitle("Scale calibration");
		m_scaleCalibrationDialog->setWindowModality(Qt::ApplicationModal);

		// Connect close signal
		auto connection = QWidget::connect(
			m_scaleCalibrationDialog,
			&QDialog::rejected,
			this,
			[this]() { closeScaleCalibrationDialog(); }
		);

		// Connect push buttons
		connection = QWidget::connect(
			m_scaleCalibrationDialogUi.button_cancel,
			&QPushButton::clicked,
			this,
			[this]() { closeScaleCalibrationDialog(); }
		);
		connection = QWidget::connect(
			m_scaleCalibrationDialogUi.button_acquire,
			&QPushButton::clicked,
			this,
			[this]() { scaleCalibrationButtonStartScaleCycle_clicked(); }
		);
		connection = QWidget::connect(
			m_scaleCalibrationDialogUi.button_save,
			&QPushButton::clicked,
			this,
			[this]() { scaleCalibrationButtonSaveFov_clicked(); }
		);
		connection = QWidget::connect(
			m_scaleCalibrationDialogUi.button_saveScale,
			&QPushButton::clicked,
			this,
			[this]() { scaleCalibrationButtonSaveScale_clicked(); }
		);
		connection = QWidget::connect(
			m_scaleCalibration,
			&ScaleCalibration::s_scaleCalibrationCycleProgress,
			this,
			[this](int currentCycle, int totalCycles) { updateScaleCalibrationCycleProgress(currentCycle, totalCycles); }
		);

		// Connect translation distance boxes
		connection = QWidget::connect<void(QDoubleSpinBox::*)(double)>(
			m_scaleCalibrationDialogUi.dx,
			&QDoubleSpinBox::valueChanged,
			this,
			[this](double dx) { setTranslationDistanceX(dx); }
		);
		connection = QWidget::connect<void(QDoubleSpinBox::*)(double)>(
			m_scaleCalibrationDialogUi.dy,
			&QDoubleSpinBox::valueChanged,
			this,
			[this](double dy) { setTranslationDistanceY(dy); }
		);

		// Connect scale calibration boxes. Only pixToMicrometer is shown/editable in the dialog
		// now (micrometerToPix is still stored internally - ScaleCalibrationHelper keeps both
		// directions in sync from whichever one is edited - just not displayed twice) so there
		// is no micrometerToPix wiring here anymore.
		connection = QWidget::connect<void(QDoubleSpinBox::*)(double)>(
			m_scaleCalibrationDialogUi.pixToMicrometerX_x,
			&QDoubleSpinBox::valueChanged,
			this,
			[this](double value) { setPixToMicrometerX_x(value); }
		);
		connection = QWidget::connect<void(QDoubleSpinBox::*)(double)>(
			m_scaleCalibrationDialogUi.pixToMicrometerX_y,
			&QDoubleSpinBox::valueChanged,
			this,
			[this](double value) { setPixToMicrometerX_y(value); }
		);
		connection = QWidget::connect<void(QDoubleSpinBox::*)(double)>(
			m_scaleCalibrationDialogUi.pixToMicrometerY_x,
			&QDoubleSpinBox::valueChanged,
			this,
			[this](double value) { setPixToMicrometerY_x(value); }
		);
		connection = QWidget::connect<void(QDoubleSpinBox::*)(double)>(
			m_scaleCalibrationDialogUi.pixToMicrometerY_y,
			&QDoubleSpinBox::valueChanged,
			this,
			[this](double value) { setPixToMicrometerY_y(value); }
		);

		// Connect FOV-offset boxes - objectiveName/magnification are read-only (disabled in the
		// .ui, refreshed from m_objectiveSlotNames by refreshScaleCalibrationObjectiveDisplay()),
		// not user-editable, so unlike every other box here they have no valueChanged/textEdited
		// wiring at all.
		connection = QWidget::connect(
			m_scaleCalibrationDialogUi.hasFovOffsetCheckbox,
			&QCheckBox::toggled,
			this,
			[this](bool checked) { setHasFovOffset(checked); }
		);
		connection = QWidget::connect<void(QDoubleSpinBox::*)(double)>(
			m_scaleCalibrationDialogUi.fovOffsetX,
			&QDoubleSpinBox::valueChanged,
			this,
			[this](double value) { setFovOffsetX(value); }
		);
		connection = QWidget::connect<void(QDoubleSpinBox::*)(double)>(
			m_scaleCalibrationDialogUi.fovOffsetY,
			&QDoubleSpinBox::valueChanged,
			this,
			[this](double value) { setFovOffsetY(value); }
		);
		connection = QWidget::connect<void(QDoubleSpinBox::*)(double)>(
			m_scaleCalibrationDialogUi.fovOffsetSigma,
			&QDoubleSpinBox::valueChanged,
			this,
			[this](double value) { setFovOffsetSigma(value); }
		);
		// Automated multi-cycle FOV-offset calibration
		connection = QWidget::connect(
			m_scaleCalibrationDialogUi.button_startObjectiveCycle,
			&QPushButton::clicked,
			this,
			[this]() { scaleCalibrationButtonStartObjectiveCycle_clicked(); }
		);
		connection = QWidget::connect(
			m_scaleCalibrationDialogUi.button_continueObjectiveCycle,
			&QPushButton::clicked,
			this,
			[this]() { scaleCalibrationButtonContinueObjectiveCycle_clicked(); }
		);
		connection = QWidget::connect(
			m_scaleCalibrationDialogUi.button_abortObjectiveCycle,
			&QPushButton::clicked,
			this,
			[this]() { scaleCalibrationButtonAbortObjectiveCycle_clicked(); }
		);
		connection = QWidget::connect(
			m_scaleCalibration,
			&ScaleCalibration::s_objectiveCycleProgress,
			this,
			[this](int currentCycle, int totalCycles, bool waitingForContinue) {
				updateObjectiveCycleProgress(currentCycle, totalCycles, waitingForContinue);
			}
		);
	}

	if (!m_brightfieldCamera) {
		m_scaleCalibrationDialogUi.button_acquire->setDisabled(true);
	} else {
		m_scaleCalibrationDialogUi.button_acquire->setDisabled(false);
	}

	// Initialize the scaleCalibration
	m_scaleCalibration->initialize();

	populateObjectivePickerCombos();
	refreshScaleCalibrationObjectiveDisplay();
	m_scaleCalibrationDialogUi.button_continueObjectiveCycle->setEnabled(false);
	m_scaleCalibrationDialogUi.button_abortObjectiveCycle->setEnabled(false);
	m_scaleCalibrationDialogUi.objectiveCycleStatusLabel->setText("");
	m_scaleCalibrationDialogUi.scaleCalibrationCycleStatusLabel->setText("");

	m_scaleCalibrationDialog->show();
}

void BrillouinAcquisition::updateScaleCalibrationTranslationValue(POINT2 translation) {
	// Initialize all values
	m_scaleCalibrationDialogUi.dx->setValue(translation.x);
	m_scaleCalibrationDialogUi.dy->setValue(translation.y);
}

void BrillouinAcquisition::updateScaleCalibrationData(ScaleCalibrationData scaleCalibration) {
	// We have to block the signals so that programmatically setting new values
	// doesn't trigger a new round of calculations. Only pixToMicrometer is shown in the dialog -
	// micrometerToPix is still tracked internally (see the .ui connect() comment above) but has
	// no widget here to update.
	const QSignalBlocker blocker5(m_scaleCalibrationDialogUi.pixToMicrometerX_x);
	const QSignalBlocker blocker6(m_scaleCalibrationDialogUi.pixToMicrometerX_y);
	const QSignalBlocker blocker7(m_scaleCalibrationDialogUi.pixToMicrometerY_x);
	const QSignalBlocker blocker8(m_scaleCalibrationDialogUi.pixToMicrometerY_y);

	m_scaleCalibrationDialogUi.pixToMicrometerX_x->setValue(scaleCalibration.pixToMicrometerX.x);
	m_scaleCalibrationDialogUi.pixToMicrometerX_y->setValue(scaleCalibration.pixToMicrometerX.y);
	m_scaleCalibrationDialogUi.pixToMicrometerY_x->setValue(scaleCalibration.pixToMicrometerY.x);
	m_scaleCalibrationDialogUi.pixToMicrometerY_y->setValue(scaleCalibration.pixToMicrometerY.y);
}

void BrillouinAcquisition::updateObjectiveCalibrationData(ObjectiveCalibrationData calibration) {
	// objectiveName/magnification are NOT set here - they are read-only, driven by
	// m_objectiveSlotNames (refreshScaleCalibrationObjectiveDisplay()), not by whatever a
	// calibration file/measurement happens to have stored for them.
	const QSignalBlocker blocker4(m_scaleCalibrationDialogUi.hasFovOffsetCheckbox);
	const QSignalBlocker blocker5(m_scaleCalibrationDialogUi.fovOffsetX);
	const QSignalBlocker blocker6(m_scaleCalibrationDialogUi.fovOffsetY);
	const QSignalBlocker blocker7(m_scaleCalibrationDialogUi.fovOffsetSigma);
	const QSignalBlocker blocker8(m_scaleCalibrationDialogUi.objectiveCycleFovOffsetSigma);
	const QSignalBlocker blocker9(m_scaleCalibrationDialogUi.scaleCalibrationSigma);

	m_scaleCalibrationDialogUi.hasFovOffsetCheckbox->setChecked(calibration.hasFovOffset);
	m_scaleCalibrationDialogUi.fovOffsetX->setValue(calibration.fovOffsetUm.x);
	m_scaleCalibrationDialogUi.fovOffsetY->setValue(calibration.fovOffsetUm.y);
	m_scaleCalibrationDialogUi.fovOffsetSigma->setValue(calibration.fovOffsetSigmaUm);
	// Read-only mirror of the same FOV-offset sigma, shown inside "Automated calibration: FOV"
	// itself so it is visible right next to Start/Continue/Abort without also scrolling up to
	// the FOV-center offset box.
	m_scaleCalibrationDialogUi.objectiveCycleFovOffsetSigma->setValue(calibration.fovOffsetSigmaUm);
	// Read-only display of the scale calibration's own repeatability (see
	// ScaleCalibration::startScaleCalibrationCycle()) inside "Automated calibration: Scale".
	m_scaleCalibrationDialogUi.scaleCalibrationSigma->setValue(calibration.scaleCalibrationSigmaUm);
}

void BrillouinAcquisition::closeScaleCalibrationDialog() {
	if (m_scaleCalibrationDialog) {
		m_scaleCalibrationDialog->hide();
	}
}

void BrillouinAcquisition::updateScaleCalibrationAcquisitionProgress(double progress) {
	m_scaleCalibrationDialogUi.progress->setValue(progress);
}

void BrillouinAcquisition::showScaleCalibrationStatus(std::string title, std::string message) {
	auto msgBox = QMessageBox(
		QMessageBox::Icon::Warning,
		QString::fromStdString(title),
		QString::fromStdString(message),
		QMessageBox::StandardButton::Close,
		this,
		Qt::WindowTitleHint | Qt::WindowCloseButtonHint
	);
	msgBox.exec();
}

void BrillouinAcquisition::scaleCalibrationButtonSaveScale_clicked() {
	QMetaObject::invokeMethod(
		m_scaleCalibration,
		[&m_scaleCalibration = m_scaleCalibration]() {
			m_scaleCalibration->saveScaleCalibration();
		},
		Qt::AutoConnection
	);
}

void BrillouinAcquisition::scaleCalibrationButtonSaveFov_clicked() {
	QMetaObject::invokeMethod(
		m_scaleCalibration,
		[&m_scaleCalibration = m_scaleCalibration]() {
			m_scaleCalibration->saveFovOffsetCalibration();
		},
		Qt::AutoConnection
	);
}

void BrillouinAcquisition::scaleCalibrationButtonStartObjectiveCycle_clicked() {
	auto referenceSlot = m_scaleCalibrationDialogUi.referenceObjectiveCombo->currentData().toInt();
	auto targetSlot = m_scaleCalibrationDialogUi.targetObjectiveCombo->currentData().toInt();
	auto cycles = m_scaleCalibrationDialogUi.cyclesSpinBox->value();
	auto retractUm = m_scaleCalibrationDialogUi.zRetractDistanceUm->value();

	// currentData() is a valid, non-zero slot number even for a combo entry showing "Empty" -
	// the slot number is always populated (populateObjectivePickerCombos()), only the name
	// text differs - so check the underlying name directly, not just for a 0/invalid slot.
	auto isNamed = [this](int slot) {
		return slot >= 1 && slot <= (int)m_objectiveSlotNames.size() && !m_objectiveSlotNames[slot - 1].empty();
	};
	if (!isNamed(referenceSlot) || !isNamed(targetSlot)) {
		m_scaleCalibrationDialogUi.objectiveCycleStatusLabel->setText(
			"Name both objectives under Devices > Objective Setup before running an automated calibration."
		);
		return;
	}
	if (referenceSlot == targetSlot) {
		m_scaleCalibrationDialogUi.objectiveCycleStatusLabel->setText(
			"Reference and target objective must be different."
		);
		return;
	}

	QMetaObject::invokeMethod(
		m_scaleCalibration,
		[&m_scaleCalibration = m_scaleCalibration, referenceSlot, targetSlot, cycles, retractUm]() {
			m_scaleCalibration->startObjectiveCycleCalibration(referenceSlot, targetSlot, cycles, retractUm);
		},
		Qt::AutoConnection
	);
}

void BrillouinAcquisition::scaleCalibrationButtonContinueObjectiveCycle_clicked() {
	QMetaObject::invokeMethod(
		m_scaleCalibration,
		[&m_scaleCalibration = m_scaleCalibration]() {
			m_scaleCalibration->continueObjectiveCycle();
		},
		Qt::AutoConnection
	);
}

void BrillouinAcquisition::scaleCalibrationButtonAbortObjectiveCycle_clicked() {
	QMetaObject::invokeMethod(
		m_scaleCalibration,
		[&m_scaleCalibration = m_scaleCalibration]() {
			m_scaleCalibration->abortObjectiveCycle();
		},
		Qt::AutoConnection
	);
}

void BrillouinAcquisition::updateObjectiveCycleProgress(int currentCycle, int totalCycles, bool waitingForContinue) {
	// currentCycle == 0 is the idle/finished state (see ScaleCalibration::finishObjectiveCycle())
	// - everything else (1..totalCycles) is a run in progress, either actively switching/
	// capturing (waitingForContinue == false) or paused for the operator to refocus
	// (waitingForContinue == true).
	auto running = currentCycle != 0;
	m_suppressObjectiveSwitchWarnings = running;

	m_scaleCalibrationDialogUi.button_startObjectiveCycle->setEnabled(!running);
	m_scaleCalibrationDialogUi.button_continueObjectiveCycle->setEnabled(waitingForContinue);
	m_scaleCalibrationDialogUi.button_abortObjectiveCycle->setEnabled(running);
	// referenceObjectiveCombo: locked permanently once a global reference objective is set (see
	// populateObjectivePickerCombos()'s own doc comment) - only still an ordinary, run-lockable
	// combo for the "no reference set yet" case.
	auto hasGlobalReference = m_scanControl && [this]() {
		for (size_t ii = 0; ii < m_objectiveSlotNames.size(); ii++) {
			if (m_scanControl->getObjectiveCalibration((int)ii + 1).isReferenceObjective) {
				return true;
			}
		}
		return false;
	}();
	m_scaleCalibrationDialogUi.referenceObjectiveCombo->setEnabled(!hasGlobalReference && !running);
	m_scaleCalibrationDialogUi.targetObjectiveCombo->setEnabled(!running);
	m_scaleCalibrationDialogUi.cyclesSpinBox->setEnabled(!running);
	m_scaleCalibrationDialogUi.zRetractDistanceUm->setEnabled(!running);

	if (running) {
		m_scaleCalibrationDialogUi.objectiveCycleStatusLabel->setText(
			"Cycle " + QString::number(currentCycle) + " of " + QString::number(totalCycles) +
			(waitingForContinue ? " - refocus, then click Continue." : " - switching objectives...")
		);
	}
}

void BrillouinAcquisition::scaleCalibrationButtonStartScaleCycle_clicked() {
	auto cycles = m_scaleCalibrationDialogUi.scaleCyclesSpinBox->value();
	QMetaObject::invokeMethod(
		m_scaleCalibration,
		[&m_scaleCalibration = m_scaleCalibration, cycles]() {
			m_scaleCalibration->startScaleCalibrationCycle(cycles);
		},
		Qt::AutoConnection
	);
}

void BrillouinAcquisition::updateScaleCalibrationCycleProgress(int currentCycle, int totalCycles) {
	// currentCycle == 0 is the idle/finished state (see ScaleCalibration::startScaleCalibrationCycle()).
	// Unlike the FOV-offset cycle, there is no "waiting for continue" pause - this run is fully
	// autonomous - so only a running/not-running distinction is needed here.
	auto running = currentCycle != 0;
	m_scaleCalibrationDialogUi.button_acquire->setEnabled(!running && m_brightfieldCamera);
	m_scaleCalibrationDialogUi.scaleCyclesSpinBox->setEnabled(!running);

	if (running) {
		m_scaleCalibrationDialogUi.scaleCalibrationCycleStatusLabel->setText(
			"Cycle " + QString::number(currentCycle) + " of " + QString::number(totalCycles) + "..."
		);
	}
}

void BrillouinAcquisition::setTranslationDistanceX(double dx) {
	m_scaleCalibration->setTranslationDistanceX(dx);
}

void BrillouinAcquisition::setTranslationDistanceY(double dy) {
	m_scaleCalibration->setTranslationDistanceY(dy);
}

void BrillouinAcquisition::setPixToMicrometerX_x(double value) {
	m_scaleCalibration->setPixToMicrometerX_x(value);
}

void BrillouinAcquisition::setPixToMicrometerX_y(double value) {
	m_scaleCalibration->setPixToMicrometerX_y(value);
}

void BrillouinAcquisition::setPixToMicrometerY_x(double value) {
	m_scaleCalibration->setPixToMicrometerY_x(value);
}

void BrillouinAcquisition::setPixToMicrometerY_y(double value) {
	m_scaleCalibration->setPixToMicrometerY_y(value);
}

void BrillouinAcquisition::setObjectiveName(QString name) {
	m_scaleCalibration->setObjectiveName(name);
}

void BrillouinAcquisition::setMagnification(double value) {
	m_scaleCalibration->setMagnification(value);
}

void BrillouinAcquisition::setHasFovOffset(bool hasFovOffset) {
	m_scaleCalibration->setHasFovOffset(hasFovOffset);
}

void BrillouinAcquisition::setFovOffsetX(double value) {
	m_scaleCalibration->setFovOffsetX(value);
}

void BrillouinAcquisition::setFovOffsetY(double value) {
	m_scaleCalibration->setFovOffsetY(value);
}

void BrillouinAcquisition::setFovOffsetSigma(double value) {
	m_scaleCalibration->setFovOffsetSigma(value);
}

std::string BrillouinAcquisition::defaultCalibrationsFolderPath() const {
	// The exe always lands at "<repo>/x64/<Debug|Release>/BrillouinAcquisition.exe" for this
	// project's build layout, so two directories up from applicationDirPath() is always the
	// repo root regardless of configuration.
	auto path = QDir::cleanPath(QCoreApplication::applicationDirPath() + "/../../BrillouinAcquisition/scaleCalibrationFiles");
	return path.toStdString();
}

void BrillouinAcquisition::loadLinkedObjectiveCalibrations() {
	// Registers whatever Objective Setup has linked for each slot, straight into ScanControl -
	// the only calibration-loading mechanism left (the older folder-scan auto-load was removed
	// as redundant/ambiguous once every slot has an explicit link). Only ever registers slots
	// that actually have a link configured; slots with none are left exactly as a prior session
	// (or the default-constructed empty state) set them.
	for (size_t ii = 0; ii < m_objectiveSlotCalibrationPaths.size(); ii++) {
		auto path = m_objectiveSlotCalibrationPaths[ii];
		if (path.empty()) {
			continue;
		}
		auto slot = (int)ii + 1;
		auto name = ii < m_objectiveSlotNames.size() ? m_objectiveSlotNames[ii] : std::string{};
		auto magnification = magnificationFromObjectiveName(name);
		QMetaObject::invokeMethod(
			m_scaleCalibration,
			[&m_scaleCalibration = m_scaleCalibration, slot, path, name, magnification]() {
				m_scaleCalibration->loadCalibrationForSlot(slot, path, name, magnification);
			},
			Qt::AutoConnection
		);
	}
}

void BrillouinAcquisition::on_action_Objective_setup_triggered() {
	// setupUi()/connect() must run exactly once per Ui object - see the identical comment in
	// on_action_Scale_calibration_acquire_triggered(). Calling setupUi() again on every open was
	// the actual cause of the Apply button appearing to render in the wrong place and typed
	// values (magnification, calibration links) not visibly updating until a full app restart:
	// each reopen silently built a fresh, unparented set of child widgets while the still-
	// visible dialog kept showing the previous ones.
	if (!m_objectiveSetupDialog) {
		m_objectiveSetupDialog = new QDialog(this, Qt::WindowTitleHint | Qt::WindowCloseButtonHint);
		m_objectiveSetupDialogUi.setupUi(m_objectiveSetupDialog);
		m_objectiveSetupDialog->setWindowTitle("Objective setup");
		m_objectiveSetupDialog->setWindowModality(Qt::ApplicationModal);

		std::array<QPushButton*, 6> browseButtons{
			m_objectiveSetupDialogUi.button_browseCalibration_1, m_objectiveSetupDialogUi.button_browseCalibration_2,
			m_objectiveSetupDialogUi.button_browseCalibration_3, m_objectiveSetupDialogUi.button_browseCalibration_4,
			m_objectiveSetupDialogUi.button_browseCalibration_5, m_objectiveSetupDialogUi.button_browseCalibration_6
		};
		std::array<QPushButton*, 6> newButtons{
			m_objectiveSetupDialogUi.button_newCalibration_1, m_objectiveSetupDialogUi.button_newCalibration_2,
			m_objectiveSetupDialogUi.button_newCalibration_3, m_objectiveSetupDialogUi.button_newCalibration_4,
			m_objectiveSetupDialogUi.button_newCalibration_5, m_objectiveSetupDialogUi.button_newCalibration_6
		};
		std::array<QPushButton*, 6> clearButtons{
			m_objectiveSetupDialogUi.button_clearCalibration_1, m_objectiveSetupDialogUi.button_clearCalibration_2,
			m_objectiveSetupDialogUi.button_clearCalibration_3, m_objectiveSetupDialogUi.button_clearCalibration_4,
			m_objectiveSetupDialogUi.button_clearCalibration_5, m_objectiveSetupDialogUi.button_clearCalibration_6
		};
		std::array<QCheckBox*, 6> referenceCheckboxes{
			m_objectiveSetupDialogUi.objectiveReference_1, m_objectiveSetupDialogUi.objectiveReference_2,
			m_objectiveSetupDialogUi.objectiveReference_3, m_objectiveSetupDialogUi.objectiveReference_4,
			m_objectiveSetupDialogUi.objectiveReference_5, m_objectiveSetupDialogUi.objectiveReference_6
		};

		auto connection = QWidget::connect(
			m_objectiveSetupDialog,
			&QDialog::rejected,
			this,
			[this]() { m_objectiveSetupDialog->hide(); }
		);
		connection = QWidget::connect(
			m_objectiveSetupDialogUi.button_cancel,
			&QPushButton::clicked,
			this,
			[this]() { m_objectiveSetupDialog->hide(); }
		);
		connection = QWidget::connect(
			m_objectiveSetupDialogUi.button_apply,
			&QPushButton::clicked,
			this,
			[this]() { objectiveSetupButtonApply_clicked(); }
		);
		for (int ii = 0; ii < (int)browseButtons.size(); ii++) {
			connection = QWidget::connect(
				browseButtons[ii],
				&QPushButton::clicked,
				this,
				[this, ii]() { objectiveSetupBrowseCalibration_clicked(ii); }
			);
			connection = QWidget::connect(
				newButtons[ii],
				&QPushButton::clicked,
				this,
				[this, ii]() { objectiveSetupNewCalibration_clicked(ii); }
			);
			connection = QWidget::connect(
				clearButtons[ii],
				&QPushButton::clicked,
				this,
				[this, ii]() { objectiveSetupClearCalibration_clicked(ii); }
			);
			connection = QWidget::connect(
				referenceCheckboxes[ii],
				&QCheckBox::toggled,
				this,
				[this, ii](bool checked) { objectiveSetupReferenceToggled(ii, checked); }
			);
		}
	}

	std::array<QLineEdit*, 6> nameFields{
		m_objectiveSetupDialogUi.objectiveName_1, m_objectiveSetupDialogUi.objectiveName_2,
		m_objectiveSetupDialogUi.objectiveName_3, m_objectiveSetupDialogUi.objectiveName_4,
		m_objectiveSetupDialogUi.objectiveName_5, m_objectiveSetupDialogUi.objectiveName_6
	};
	std::array<QLabel*, 6> slotLabels{
		m_objectiveSetupDialogUi.slot1_label, m_objectiveSetupDialogUi.slot2_label,
		m_objectiveSetupDialogUi.slot3_label, m_objectiveSetupDialogUi.slot4_label,
		m_objectiveSetupDialogUi.slot5_label, m_objectiveSetupDialogUi.slot6_label
	};
	std::array<QLabel*, 6> calibrationPathLabels{
		m_objectiveSetupDialogUi.calibrationPath_1, m_objectiveSetupDialogUi.calibrationPath_2,
		m_objectiveSetupDialogUi.calibrationPath_3, m_objectiveSetupDialogUi.calibrationPath_4,
		m_objectiveSetupDialogUi.calibrationPath_5, m_objectiveSetupDialogUi.calibrationPath_6
	};
	std::array<QPushButton*, 6> browseButtons{
		m_objectiveSetupDialogUi.button_browseCalibration_1, m_objectiveSetupDialogUi.button_browseCalibration_2,
		m_objectiveSetupDialogUi.button_browseCalibration_3, m_objectiveSetupDialogUi.button_browseCalibration_4,
		m_objectiveSetupDialogUi.button_browseCalibration_5, m_objectiveSetupDialogUi.button_browseCalibration_6
	};
	std::array<QPushButton*, 6> newButtons{
		m_objectiveSetupDialogUi.button_newCalibration_1, m_objectiveSetupDialogUi.button_newCalibration_2,
		m_objectiveSetupDialogUi.button_newCalibration_3, m_objectiveSetupDialogUi.button_newCalibration_4,
		m_objectiveSetupDialogUi.button_newCalibration_5, m_objectiveSetupDialogUi.button_newCalibration_6
	};
	std::array<QPushButton*, 6> clearButtons{
		m_objectiveSetupDialogUi.button_clearCalibration_1, m_objectiveSetupDialogUi.button_clearCalibration_2,
		m_objectiveSetupDialogUi.button_clearCalibration_3, m_objectiveSetupDialogUi.button_clearCalibration_4,
		m_objectiveSetupDialogUi.button_clearCalibration_5, m_objectiveSetupDialogUi.button_clearCalibration_6
	};
	std::array<QCheckBox*, 6> referenceCheckboxes{
		m_objectiveSetupDialogUi.objectiveReference_1, m_objectiveSetupDialogUi.objectiveReference_2,
		m_objectiveSetupDialogUi.objectiveReference_3, m_objectiveSetupDialogUi.objectiveReference_4,
		m_objectiveSetupDialogUi.objectiveReference_5, m_objectiveSetupDialogUi.objectiveReference_6
	};

	auto hasObjectiveElement = !m_objectiveSlotNames.empty();
	m_objectiveSetupDialogUi.noObjectiveElementLabel->setVisible(!hasObjectiveElement);
	m_objectiveSetupDialogUi.slotsContainer->setVisible(hasObjectiveElement);
	m_objectiveSetupDialogUi.instructionsLabel->setVisible(hasObjectiveElement);
	m_objectiveSetupDialogUi.button_apply->setEnabled(hasObjectiveElement);
	m_objectiveSetupDialogUi.statusLabel->setText("");

	// Kept in sync with m_objectiveSlotNames' size everywhere it changes (initScanControl(),
	// objectiveSetupButtonApply_clicked()) - this resize is just a defensive fallback so a
	// mismatch (should not happen) shows empty links rather than crashing on an out-of-range
	// index below.
	m_objectiveSlotCalibrationPaths.resize(m_objectiveSlotNames.size());

	for (size_t ii = 0; ii < nameFields.size(); ii++) {
		auto rowExists = ii < m_objectiveSlotNames.size();
		slotLabels[ii]->setVisible(rowExists);
		nameFields[ii]->setVisible(rowExists);
		calibrationPathLabels[ii]->setVisible(rowExists);
		browseButtons[ii]->setVisible(rowExists);
		newButtons[ii]->setVisible(rowExists);
		clearButtons[ii]->setVisible(rowExists);
		referenceCheckboxes[ii]->setVisible(rowExists);
		if (rowExists) {
			nameFields[ii]->setText(QString::fromStdString(m_objectiveSlotNames[ii]));
			auto path = m_objectiveSlotCalibrationPaths[ii];
			// Full path lives in the tooltip - objectiveSetupButtonApply_clicked() reads that
			// back, the visible text is just the filename so a long path doesn't blow out the
			// column width.
			calibrationPathLabels[ii]->setToolTip(QString::fromStdString(path));
			calibrationPathLabels[ii]->setText(path.empty() ? "(none)"
				: QFileInfo(QString::fromStdString(path)).fileName());
			// Reflects whatever is currently registered live (loadLinkedObjectiveCalibrations()
			// already populated every linked slot at startup, not just the active one - see its
			// own doc comment) - blocked so re-populating the dialog never itself triggers
			// objectiveSetupReferenceToggled().
			const QSignalBlocker blocker(*referenceCheckboxes[ii]);
			referenceCheckboxes[ii]->setChecked(
				m_scanControl && m_scanControl->getObjectiveCalibration((int)ii + 1).isReferenceObjective);
		}
	}

	m_objectiveSetupDialog->show();
}

void BrillouinAcquisition::objectiveSetupBrowseCalibration_clicked(int slotIndex) {
	if (slotIndex < 0 || slotIndex >= (int)m_objectiveSlotCalibrationPaths.size()) {
		return;
	}
	auto currentPath = m_objectiveSlotCalibrationPaths[slotIndex];
	auto path = QFileDialog::getOpenFileName(m_objectiveSetupDialog, tr("Select scale calibration"),
		QString::fromStdString(currentPath), tr("Scale calibration (*.h5)"));
	if (path.isEmpty()) {
		return;
	}
	// Staged in the label only, same as the name fields - not written to
	// m_objectiveSlotCalibrationPaths or loaded until Apply is clicked.
	std::array<QLabel*, 6> calibrationPathLabels{
		m_objectiveSetupDialogUi.calibrationPath_1, m_objectiveSetupDialogUi.calibrationPath_2,
		m_objectiveSetupDialogUi.calibrationPath_3, m_objectiveSetupDialogUi.calibrationPath_4,
		m_objectiveSetupDialogUi.calibrationPath_5, m_objectiveSetupDialogUi.calibrationPath_6
	};
	calibrationPathLabels[slotIndex]->setToolTip(path);
	calibrationPathLabels[slotIndex]->setText(QFileInfo(path).fileName());
}

void BrillouinAcquisition::objectiveSetupNewCalibration_clicked(int slotIndex) {
	if (slotIndex < 0 || slotIndex >= (int)m_objectiveSlotCalibrationPaths.size()) {
		return;
	}
	std::array<QLineEdit*, 6> nameFields{
		m_objectiveSetupDialogUi.objectiveName_1, m_objectiveSetupDialogUi.objectiveName_2,
		m_objectiveSetupDialogUi.objectiveName_3, m_objectiveSetupDialogUi.objectiveName_4,
		m_objectiveSetupDialogUi.objectiveName_5, m_objectiveSetupDialogUi.objectiveName_6
	};
	// The row's current (possibly not-yet-Applied) name - fine to use as-is, this only affects
	// the new file's own recorded metadata, not m_objectiveSlotNames.
	auto name = nameFields[slotIndex]->text().trimmed().toStdString();
	auto magnification = magnificationFromObjectiveName(name);

	auto folder = QString::fromStdString(defaultCalibrationsFolderPath());
	try {
		create_directories(folder.toStdString());
	} catch (const filesystem_error&) {
		// Not fatal - getSaveFileName below still works against a non-existent default folder,
		// the operator just has to navigate to/create one themselves.
	}
	auto shortDate = QDateTime::currentDateTime().toString("yyyy-MM-ddTHHmmss");
	auto defaultFileName = QString("_scaleCalibration_%1_%2.h5")
		.arg(name.empty() ? QString("unnamed") : QString::fromStdString(name))
		.arg(shortDate);
	auto path = QFileDialog::getSaveFileName(m_objectiveSetupDialog, tr("Create scale calibration file"),
		folder + "/" + defaultFileName, tr("Scale calibration (*.h5)"));
	if (path.isEmpty()) {
		return;
	}

	auto slot = slotIndex + 1;
	auto pathStd = path.toStdString();
	QMetaObject::invokeMethod(
		m_scaleCalibration,
		[&m_scaleCalibration = m_scaleCalibration, slot, name, magnification, pathStd]() {
			m_scaleCalibration->createEmptyCalibrationFile(slot, name, magnification, pathStd);
		},
		Qt::AutoConnection
	);

	// Staged in the label only, same as Browse - not written to m_objectiveSlotCalibrationPaths
	// until Apply is clicked (even though the file itself, and its live registration in
	// ScanControl above, already happened - Apply only governs what gets persisted/reloaded at
	// the next startup).
	std::array<QLabel*, 6> calibrationPathLabels{
		m_objectiveSetupDialogUi.calibrationPath_1, m_objectiveSetupDialogUi.calibrationPath_2,
		m_objectiveSetupDialogUi.calibrationPath_3, m_objectiveSetupDialogUi.calibrationPath_4,
		m_objectiveSetupDialogUi.calibrationPath_5, m_objectiveSetupDialogUi.calibrationPath_6
	};
	calibrationPathLabels[slotIndex]->setToolTip(path);
	calibrationPathLabels[slotIndex]->setText(QFileInfo(path).fileName());
}

void BrillouinAcquisition::objectiveSetupClearCalibration_clicked(int slotIndex) {
	if (slotIndex < 0 || slotIndex >= (int)m_objectiveSlotCalibrationPaths.size()) {
		return;
	}
	std::array<QLabel*, 6> calibrationPathLabels{
		m_objectiveSetupDialogUi.calibrationPath_1, m_objectiveSetupDialogUi.calibrationPath_2,
		m_objectiveSetupDialogUi.calibrationPath_3, m_objectiveSetupDialogUi.calibrationPath_4,
		m_objectiveSetupDialogUi.calibrationPath_5, m_objectiveSetupDialogUi.calibrationPath_6
	};
	calibrationPathLabels[slotIndex]->setToolTip("");
	calibrationPathLabels[slotIndex]->setText("(none)");
}

void BrillouinAcquisition::objectiveSetupReferenceToggled(int slotIndex, bool checked) {
	if (!m_scanControl || slotIndex < 0 || slotIndex >= (int)m_objectiveSlotCalibrationPaths.size()) {
		return;
	}
	auto slot = slotIndex + 1;
	std::array<QLineEdit*, 6> nameFields{
		m_objectiveSetupDialogUi.objectiveName_1, m_objectiveSetupDialogUi.objectiveName_2,
		m_objectiveSetupDialogUi.objectiveName_3, m_objectiveSetupDialogUi.objectiveName_4,
		m_objectiveSetupDialogUi.objectiveName_5, m_objectiveSetupDialogUi.objectiveName_6
	};
	std::array<QCheckBox*, 6> referenceCheckboxes{
		m_objectiveSetupDialogUi.objectiveReference_1, m_objectiveSetupDialogUi.objectiveReference_2,
		m_objectiveSetupDialogUi.objectiveReference_3, m_objectiveSetupDialogUi.objectiveReference_4,
		m_objectiveSetupDialogUi.objectiveReference_5, m_objectiveSetupDialogUi.objectiveReference_6
	};

	if (!checked) {
		// Only reachable by unchecking the row that currently IS the reference (every other row
		// is already unchecked - see the mutual-exclusion loop below) - demote it back to
		// "unmeasured" (see this function's own doc comment in the header for why demotion
		// resets rather than tries to guess a value relative to some other objective).
		auto data = m_scanControl->getObjectiveCalibration(slot);
		data.isReferenceObjective = false;
		data.hasFovOffset = false;
		data.fovOffsetUm = POINT2{ 0, 0 };
		data.fovOffsetSigmaUm = 0.0;
		data.referenceObjectiveName = "";
		auto path = m_objectiveSlotCalibrationPaths[slotIndex];
		QMetaObject::invokeMethod(
			m_scaleCalibration,
			[&m_scaleCalibration = m_scaleCalibration, slot, path, data]() {
				m_scaleCalibration->writeCalibrationToSlot(slot, path, data);
			},
			Qt::AutoConnection
		);
		m_objectiveSetupDialogUi.statusLabel->setText(
			"No reference objective set - FOV-center offsets for every objective are now unmeasured "
			"relative to nothing in particular until a new reference is chosen.");
		refreshScaleCalibrationObjectiveDisplay();
		populateObjectivePickerCombos();
		return;
	}

	// Demote whichever OTHER slot currently holds the flag (if any) - only one reference at a
	// time. A demoted objective's own fovOffsetUm was only ever valid relative to the OLD
	// reference, so it is reset to "unmeasured" (not silently kept as a now-wrong value) -
	// see this function's own doc comment in the header.
	auto demotedAny = false;
	for (size_t ii = 0; ii < m_objectiveSlotNames.size(); ii++) {
		auto otherSlot = (int)ii + 1;
		if (otherSlot == slot) {
			continue;
		}
		auto otherData = m_scanControl->getObjectiveCalibration(otherSlot);
		if (!otherData.isReferenceObjective) {
			continue;
		}
		demotedAny = true;
		otherData.isReferenceObjective = false;
		otherData.hasFovOffset = false;
		otherData.fovOffsetUm = POINT2{ 0, 0 };
		otherData.fovOffsetSigmaUm = 0.0;
		otherData.referenceObjectiveName = "";
		auto otherPath = m_objectiveSlotCalibrationPaths[ii];
		QMetaObject::invokeMethod(
			m_scaleCalibration,
			[&m_scaleCalibration = m_scaleCalibration, otherSlot, otherPath, otherData]() {
				m_scaleCalibration->writeCalibrationToSlot(otherSlot, otherPath, otherData);
			},
			Qt::AutoConnection
		);
		const QSignalBlocker blocker(*referenceCheckboxes[ii]);
		referenceCheckboxes[ii]->setChecked(false);
	}

	auto data = m_scanControl->getObjectiveCalibration(slot);
	data.isReferenceObjective = true;
	data.hasFovOffset = true;
	data.fovOffsetUm = POINT2{ 0, 0 };
	data.fovOffsetSigmaUm = 0.0;
	// The reference IS the baseline everything else is measured relative to - it has no
	// reference of its own.
	data.referenceObjectiveName = "";
	// The row's current (possibly not-yet-Applied) name, same convention
	// objectiveSetupNewCalibration_clicked() already uses - only affects this file's own
	// recorded identity metadata, not m_objectiveSlotNames.
	auto name = nameFields[slotIndex]->text().trimmed().toStdString();
	data.objectiveName = name;
	data.magnification = magnificationFromObjectiveName(name);
	auto path = m_objectiveSlotCalibrationPaths[slotIndex];
	QMetaObject::invokeMethod(
		m_scaleCalibration,
		[&m_scaleCalibration = m_scaleCalibration, slot, path, data]() {
			m_scaleCalibration->writeCalibrationToSlot(slot, path, data);
		},
		Qt::AutoConnection
	);

	m_objectiveSetupDialogUi.statusLabel->setText(demotedAny
		? QString("\"%1\" is now the reference objective (FOV-center offset 0, 0). Any other "
			"objective's previously-saved FOV offset was measured relative to the OLD reference "
			"and needs re-measuring.").arg(QString::fromStdString(name))
		: QString("\"%1\" is now the reference objective (FOV-center offset 0, 0).").arg(QString::fromStdString(name)));
	refreshScaleCalibrationObjectiveDisplay();
	populateObjectivePickerCombos();
}

void BrillouinAcquisition::objectiveSetupButtonApply_clicked() {
	std::array<QLineEdit*, 6> nameFields{
		m_objectiveSetupDialogUi.objectiveName_1, m_objectiveSetupDialogUi.objectiveName_2,
		m_objectiveSetupDialogUi.objectiveName_3, m_objectiveSetupDialogUi.objectiveName_4,
		m_objectiveSetupDialogUi.objectiveName_5, m_objectiveSetupDialogUi.objectiveName_6
	};

	// Validate every row first, so a single bad entry doesn't leave m_objectiveSlotNames
	// half-updated - either all rows apply, or none do. One or two digits (1x-99x, no leading
	// zero) - formatObjectiveNamesForBeampath() pads a one-digit name to the same 3-character
	// beampath width as a two-digit one, so the button width stays consistent either way.
	static const QRegularExpression namePattern("^[1-9][0-9]?x$");
	auto candidates = std::vector<std::string>(m_objectiveSlotNames.size());
	for (size_t ii = 0; ii < m_objectiveSlotNames.size(); ii++) {
		auto text = nameFields[ii]->text().trimmed();
		if (text.isEmpty()) {
			candidates[ii] = "";
			continue;
		}
		if (!namePattern.match(text).hasMatch()) {
			m_objectiveSetupDialogUi.statusLabel->setText(
				"Slot " + QString::number(ii + 1) + ": \"" + text +
				"\" is not valid - use one or two digits followed by \"x\" (1x-99x), or leave blank."
			);
			return;
		}
		candidates[ii] = text.toStdString();
	}

	std::array<QLabel*, 6> calibrationPathLabels{
		m_objectiveSetupDialogUi.calibrationPath_1, m_objectiveSetupDialogUi.calibrationPath_2,
		m_objectiveSetupDialogUi.calibrationPath_3, m_objectiveSetupDialogUi.calibrationPath_4,
		m_objectiveSetupDialogUi.calibrationPath_5, m_objectiveSetupDialogUi.calibrationPath_6
	};
	auto calibrationPathCandidates = std::vector<std::string>(m_objectiveSlotCalibrationPaths.size());
	for (size_t ii = 0; ii < calibrationPathCandidates.size(); ii++) {
		// Full path lives in the tooltip (see on_action_Objective_setup_triggered()/
		// objectiveSetupBrowseCalibration_clicked()) - the visible text is just the filename.
		calibrationPathCandidates[ii] = calibrationPathLabels[ii]->toolTip().toStdString();
	}

	m_objectiveSlotNames = candidates;
	m_objectiveSlotCalibrationPaths = calibrationPathCandidates;
	writeSettings();
	pushObjectiveOptionNames();
	loadLinkedObjectiveCalibrations();
	m_objectiveSetupDialog->hide();
}

void BrillouinAcquisition::pushObjectiveOptionNames() {
	auto formatted = formatObjectiveNamesForBeampath(m_objectiveSlotNames);

	if (m_scanControl && !formatted.empty()) {
		QMetaObject::invokeMethod(
			m_scanControl,
			[scanControl = m_scanControl, formatted]() { scanControl->setObjectiveOptionNames(formatted); },
			Qt::AutoConnection
		);
	}

	updateElementButtonLabels(formatted);
	populateObjectivePickerCombos();
}

void BrillouinAcquisition::populateObjectivePickerCombos() {
	// Safe to call whether or not the dialog is currently visible/shown (e.g. this also runs
	// as part of on_action_Scale_calibration_acquire_triggered()'s own setup, before show() is
	// called) - only guards against the dialog never having been constructed at all.
	if (!m_scaleCalibrationDialog) {
		return;
	}
	const QSignalBlocker b1(m_scaleCalibrationDialogUi.referenceObjectiveCombo);
	const QSignalBlocker b2(m_scaleCalibrationDialogUi.targetObjectiveCombo);
	m_scaleCalibrationDialogUi.referenceObjectiveCombo->clear();
	m_scaleCalibrationDialogUi.targetObjectiveCombo->clear();
	// The global reference slot (Objective Setup's "Reference" checkbox - see
	// ObjectiveCalibrationData::isReferenceObjective), found here rather than cached anywhere
	// since it can change (objectiveSetupReferenceToggled()) independently of this dialog.
	auto referenceSlot = -1;
	if (m_scanControl) {
		for (size_t ii = 0; ii < m_objectiveSlotNames.size(); ii++) {
			if (m_scanControl->getObjectiveCalibration((int)ii + 1).isReferenceObjective) {
				referenceSlot = (int)ii + 1;
				break;
			}
		}
	}
	auto referenceComboIndex = -1;
	for (size_t ii = 0; ii < m_objectiveSlotNames.size(); ii++) {
		auto slot = (int)ii + 1;
		auto text = m_objectiveSlotNames[ii].empty()
			? QString("Empty")
			: QString::fromStdString(m_objectiveSlotNames[ii]);
		m_scaleCalibrationDialogUi.referenceObjectiveCombo->addItem(text, QVariant(slot));
		m_scaleCalibrationDialogUi.targetObjectiveCombo->addItem(text, QVariant(slot));
		if (slot == referenceSlot) {
			referenceComboIndex = (int)ii;
		}
	}
	// There is only ever one true reference objective (see Objective Setup's mutually-exclusive
	// "Reference" checkbox) - measuring a NEW objective's FOV offset against anything else would
	// just need composing back through that reference anyway (measureFovOffset() already does
	// this automatically for an indirect chain), so picking any other objective here was never
	// actually useful, only a source of operator error. Locked to the reference and disabled
	// outright if one is set; left as an ordinary, operator-picked combo (same as before) if none
	// is set yet, so an operator can still run the very first calibration cycle before any
	// objective has been marked as the reference.
	if (referenceComboIndex >= 0) {
		m_scaleCalibrationDialogUi.referenceObjectiveCombo->setCurrentIndex(referenceComboIndex);
	}
	m_scaleCalibrationDialogUi.referenceObjectiveCombo->setDisabled(referenceComboIndex >= 0);
}

void BrillouinAcquisition::refreshScaleCalibrationObjectiveDisplay() {
	if (!m_scaleCalibrationDialog || !m_scanControl) {
		return;
	}
	// Direct, same-thread-unsafe-in-principle but already-precedented read of ScanControl's
	// live state from the GUI thread (see objectiveSwitched()'s own
	// m_scanControl->getObjectiveCalibration() call) - a trivial int/struct getter, not worth a
	// round trip for.
	auto activeSlot = m_scanControl->getActiveObjectiveSlot();
	auto activeName = (activeSlot >= 1 && activeSlot <= (int)m_objectiveSlotNames.size())
		? m_objectiveSlotNames[activeSlot - 1] : std::string{};
	auto activeMagnification = magnificationFromObjectiveName(activeName);

	const QSignalBlocker blocker1(m_scaleCalibrationDialogUi.objectiveName);
	const QSignalBlocker blocker2(m_scaleCalibrationDialogUi.magnification);
	m_scaleCalibrationDialogUi.objectiveName->setText(QString::fromStdString(activeName));
	m_scaleCalibrationDialogUi.magnification->setValue(activeMagnification);
	// Keep ScaleCalibration's own edit buffer in sync too, so Save persists the right
	// name/magnification even though nothing in the dialog lets the operator type them anymore.
	m_scaleCalibration->setObjectiveName(QString::fromStdString(activeName));
	m_scaleCalibration->setMagnification(activeMagnification);

	// Same idea for the linked calibration file - shown read-only so the operator can see which
	// file Save will write into, and pushed into ScaleCalibration so it actually does.
	auto activePath = (activeSlot >= 1 && activeSlot <= (int)m_objectiveSlotCalibrationPaths.size())
		? m_objectiveSlotCalibrationPaths[activeSlot - 1] : std::string{};
	m_scaleCalibrationDialogUi.objectiveCalibrationFile->setToolTip(QString::fromStdString(activePath));
	m_scaleCalibrationDialogUi.objectiveCalibrationFile->setText(activePath.empty() ? "(none)"
		: QFileInfo(QString::fromStdString(activePath)).fileName());
	m_scaleCalibration->setLinkedCalibrationFilePath(activePath);

	// The reference objective's FOV-center offset is locked at {0,0}, sigma 0 - see
	// ObjectiveCalibrationData::isReferenceObjective's own doc comment. Editing it here would be
	// meaningless (it is set exclusively via Objective Setup's "Reference" checkbox); grey the
	// fields out rather than let the operator type a value that the next objectiveSetupReference
	// Toggled()/switch would just overwrite anyway.
	auto activeIsReference = m_scanControl->getObjectiveCalibration(activeSlot).isReferenceObjective;
	m_scaleCalibrationDialogUi.hasFovOffsetCheckbox->setDisabled(activeIsReference);
	m_scaleCalibrationDialogUi.fovOffsetX->setDisabled(activeIsReference);
	m_scaleCalibrationDialogUi.fovOffsetY->setDisabled(activeIsReference);
	m_scaleCalibrationDialogUi.fovOffsetSigma->setDisabled(activeIsReference);
	// The reference's own offset is locked at {0,0} - nothing to persist by re-saving it, and
	// leaving Save enabled invited clicking it under the impression it did something here.
	m_scaleCalibrationDialogUi.button_save->setDisabled(activeIsReference);
}

void BrillouinAcquisition::initBeampathButtons() {
	if (m_scanControl == nullptr) {
		return;
	}

	for (auto widget : ui->beamPathBox->findChildren<QWidget*>(QString(), Qt::FindDirectChildrenOnly)) {
		delete widget;
	}

	QLayout* layout = ui->beamPathBox->layout();
	if (layout != 0) {
		QLayoutItem *item;
		while ((item = layout->takeAt(0)) != 0) {
			layout->removeItem(item);
		}
		delete item;
		delete layout;
	}
	int maxWidgetsPerRow{ 4 };
	QMetaObject::Connection connection;
	QVBoxLayout *verticalLayout = new QVBoxLayout;
	verticalLayout->setAlignment(Qt::AlignTop);
	std::string buttonLabel;
	presetButtons.clear();
	auto presets = m_scanControl->m_presets;
	QWidget* presetWidget = new QWidget();
	int rows{ 0 };
	if (presets.size() > 0) {
		// Create preset Widget and set vertical layout
		verticalLayout->addWidget(presetWidget);
		QVBoxLayout* presetWidgetLayout = new QVBoxLayout(presetWidget);
		presetWidgetLayout->setMargin(0);

		// Horizontal layout for preset label
		QHBoxLayout * presetLabelLayout = new QHBoxLayout();
		QLabel *presetLabel = new QLabel("Presets:");
		presetLabelLayout->addWidget(presetLabel);
		presetWidgetLayout->addLayout(presetLabelLayout);

		// Grid layout for preset buttons
		QGridLayout *layout = new QGridLayout();
		layout->setAlignment(Qt::AlignLeft);
		for (gsl::index ii = 0; ii < presets.size(); ii++) {
			QPushButton *button = new QPushButton(presets[ii].name.c_str());
			button->setSizePolicy(QSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed));
			button->setMinimumWidth(24);
			button->setMaximumWidth(64);
			button->setMinimumHeight(18);
			layout->addWidget(button, 1 + floor(ii/maxWidgetsPerRow), ii%maxWidgetsPerRow, Qt::AlignLeft);

			connection = QObject::connect(button, &QPushButton::clicked, [=] {
				setPreset(presets[ii].index);
			});
			presetButtons.push_back(button);
		}
		int presetRows = (1 + ceil(presets.size() / (double)maxWidgetsPerRow));
		rows += presetRows;
		int height = presetRows * 20;
		presetWidget->setMinimumHeight(height);
		presetWidgetLayout->addLayout(layout);
	}

	QWidget *elementButtonWidget = new QWidget();
	elementButtonWidget->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
	verticalLayout->addWidget(elementButtonWidget);
	QVBoxLayout* elementButtonWidgetLayout = new QVBoxLayout(elementButtonWidget);
	elementButtonWidgetLayout->setMargin(0);

	elementButtons.clear();
	elementIntBox.clear();
	elementDoubleBox.clear();
	elementSlider.clear();
	elementSliderInput.clear();
	auto elements = m_scanControl->m_deviceElements;
	rows += elements.size();
	elementButtonWidget->setMinimumHeight(elements.size() * 20);
	for (gsl::index ii{ 0 }; ii < elements.size(); ii++) {
		DeviceElement element = elements[ii];
		QHBoxLayout *layout = new QHBoxLayout();

		layout->setAlignment(Qt::AlignLeft);
		QLabel *groupLabel = new QLabel(element.name.c_str());
		groupLabel->setSizePolicy(QSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed));
		groupLabel->setMinimumWidth(40);
		groupLabel->setMaximumWidth(40);
		layout->addWidget(groupLabel);
		if (element.inputType == DEVICE_INPUT_TYPE::PUSHBUTTON) {
			std::vector<QPushButton*> buttons;
			for (gsl::index jj = 0; jj < element.maxOptions; jj++) {
				QPushButton *button = new QPushButton(element.optionNames[jj].c_str());
				button->setSizePolicy(QSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed));
				button->setMinimumWidth(16);
				button->setMaximumWidth(48);
				layout->addWidget(button);

				connection = QObject::connect(button, &QPushButton::clicked, [=] {
					setElement(elements[ii], (double)(jj + 1));
				});
				buttons.push_back(button);
			}
			elementButtons.push_back(buttons);
		} else if (element.inputType == DEVICE_INPUT_TYPE::INTBOX) {
			QSpinBox *intBox = new QSpinBox();
			intBox->setSizePolicy(QSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed));
			intBox->setMinimum(-10000);
			intBox->setMaximum(10000);
			intBox->setMinimumWidth(48);
			intBox->setMaximumWidth(96);
			layout->addWidget(intBox);

			connection = QObject::connect<void(QSpinBox::*)(const int)>(
				intBox,
				&QSpinBox::valueChanged,
				[=](int value) {setElement(elements[ii], (double)value);}
			);
			elementIntBox.push_back(intBox);
		} else if (element.inputType == DEVICE_INPUT_TYPE::DOUBLEBOX) {
			QDoubleSpinBox *doubleBox = new QDoubleSpinBox();
			doubleBox->setSizePolicy(QSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed));
			doubleBox->setMinimum(-10000.0);
			doubleBox->setMaximum(10000.0);
			doubleBox->setMinimumWidth(48);
			doubleBox->setMaximumWidth(96);
			layout->addWidget(doubleBox);

			connection = QObject::connect<void(QDoubleSpinBox::*)(const double)>(
				doubleBox,
				&QDoubleSpinBox::valueChanged,
				[=](double value) {setElement(elements[ii], value); }
			);
			elementDoubleBox.push_back(doubleBox);
		} else if (element.inputType == DEVICE_INPUT_TYPE::SLIDER) {
			QSlider* slider = new QSlider();
			slider->setTracking(false);
			slider->setSizePolicy(QSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed));
			slider->setMinimum(0);
			slider->setMaximum(100);
			slider->setSingleStep(1.0);
			slider->setPageStep(5.0);
			slider->setOrientation(Qt::Horizontal);
			slider->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
			layout->addWidget(slider);

			QSpinBox* intBox = new QSpinBox();
			intBox->setSizePolicy(QSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed));
			intBox->setMinimumWidth(48);
			intBox->setMaximumWidth(96);
			intBox->setMinimum(0);
			intBox->setMaximum(100);
			intBox->setSingleStep(1.0);
			layout->addWidget(intBox);

			connection = QObject::connect<void(QSlider::*)(const int)>(
				slider,
				&QSlider::valueChanged,
				[=](int value) {
					setElement(elements[ii], value);
					const QSignalBlocker blocker(intBox);
					intBox->setValue(value);
				}
			);

			connection = QObject::connect<void(QSpinBox::*)(const int)>(
				intBox,
				&QSpinBox::valueChanged,
				[=](int value) {
					setElement(elements[ii], (double)value);
					const QSignalBlocker blocker(slider);
					slider->setValue(value);
				}
			);

			elementSlider.push_back(slider);
			elementSliderInput.push_back(intBox);
		}
		elementButtonWidgetLayout->addLayout(layout);
	}

	ui->beamPathBox->setLayout(verticalLayout);
	ui->beamPathBox->show();
	ui->beamPathBox->setMinimumHeight(rows * 20 + 15);
}

void BrillouinAcquisition::initScanControl() {
	// deinitialize scanner if necessary
	if (m_scanControl) {
		m_scanControl->deleteLater();
		m_scanControl = nullptr;
	}

	// initialize correct scanner type
	switch (m_scanControllerType) {
		case ScanControl::SCAN_DEVICE::ZEISSECU:
			m_scanControl = new ZeissECU();
			break;
		case ScanControl::SCAN_DEVICE::NIDAQ:
			m_scanControl = new NIDAQ();
			break;
		case ScanControl::SCAN_DEVICE::ZEISSMTB:
			m_scanControl = new ZeissMTB();
			break;
		case ScanControl::SCAN_DEVICE::ZEISSMTBERLANGEN:
			m_scanControl = new ZeissMTB_Erlangen();
			break;
		case ScanControl::SCAN_DEVICE::ZEISSMTBERLANGEN2:
			m_scanControl = new ZeissMTB_Erlangen2();
			break;
		default:
			m_scanControl = new ZeissECU();
			break;
	}

	// Resize m_objectiveSlotNames to this backend's "Objective" element (0 if it has none, e.g.
	// NIDAQ - see formatObjectiveNamesForBeampath()/pushObjectiveOptionNames()), preserving
	// whatever was already loaded from settings for slots that still exist and defaulting any
	// newly-appearing slot to "". Then push directly - m_scanControl is still exclusively
	// GUI-thread-owned here, before m_acquisitionThread.startWorker(m_scanControl) below, so
	// this does not need QMetaObject::invokeMethod the way the same call does everywhere else
	// (see pushObjectiveOptionNames()). This must run before initBeampathButtons() (called from
	// initBrillouin(), later than this function) so the beampath is built with real names from
	// the start rather than flashing "1".."6" first.
	{
		auto maxOptions = 0;
		for (const auto& element : m_scanControl->m_deviceElements) {
			if (element.name == "Objective") {
				maxOptions = element.maxOptions;
				break;
			}
		}
		m_objectiveSlotNames.resize(maxOptions);
		m_objectiveSlotCalibrationPaths.resize(maxOptions);
		if (maxOptions > 0) {
			m_scanControl->setObjectiveOptionNames(formatObjectiveNamesForBeampath(m_objectiveSlotNames));
		}
	}

	// init or de-init ODT
	initODT();
	initVoltageCalibration();
	initScaleCalibration();

	initializeLaserPositionLocation();

	// Update positions preview
	AOI_changed(m_positionsMicrometer, m_positionsMicrometerIsAbsolute);
	excludedAOI_changed(m_excludedPositionsMicrometer);

	// reestablish m_scanControl connections
	static QMetaObject::Connection connection;
	connection = QWidget::connect(
		m_scanControl,
		&ScanControl::connectedDevice,
		this,
		[this](bool isConnected) { microscopeConnectionChanged(isConnected); }
	);

	// slot to update microscope element button background color
	connection = QWidget::connect(
		m_scanControl,
		&ScanControl::elementPositionsChanged,
		this,
		[this](std::vector<double> positions) { microscopeElementPositionsChanged(positions); }
	);
	connection = QWidget::connect(
		m_scanControl,
		&ScanControl::elementPositionChanged,
		this,
		[this](DeviceElement element, double position) { microscopeElementPositionChanged(element, position); }
	);
	connection = QWidget::connect(
		m_scanControl,
		&ScanControl::s_objectiveSwitched,
		this,
		[this](int previousSlot, int newSlot, bool hasCalibration, bool hasFovOffset, POINT2 offsetUm, double offsetSigmaUm) {
			objectiveSwitched(previousSlot, newSlot, hasCalibration, hasFovOffset, offsetUm, offsetSigmaUm);
		}
	);
	connection = QWidget::connect(
		m_scanControl,
		&ScanControl::currentPosition,
		this,
		[this](POINT3 position) { showPosition(position); }
	);
	connection = QWidget::connect(
		&buttonDelegate,
		&ButtonDelegate::deletePosition,
		this->m_scanControl,
		[this](int index) { this->m_scanControl->deleteSavedPosition(index); }
	);
	connection = QWidget::connect(
		&buttonDelegate,
		&ButtonDelegate::moveToPosition,
		this->m_scanControl,
		[this](int index) { this->m_scanControl->moveToSavedPosition(index); }
	);
	connection = QWidget::connect(
		m_scanControl,
		&ScanControl::savedPositionsChanged,
		this->tableModel,
		[this](std::vector<POINT3> storage) { this->tableModel->setStorage(storage); }
	);
	connection = QWidget::connect(
		m_scanControl,
		&ScanControl::homePositionBoundsChanged,
		this,
		[this](BOUNDS bounds) { setHomePositionBounds(bounds); }
	);
	connection = QWidget::connect(
		m_scanControl,
		&ScanControl::currentPositionBoundsChanged,
		this,
		[this](BOUNDS bounds) { setCurrentPositionBounds(bounds); }
	);
	connection = QWidget::connect(
		m_scanControl,
		&ScanControl::s_gridOffsetChanged,
		this,
		[this](POINT2 offsetUm, bool positionIsAbsolute) { on_gridOffsetChanged(offsetUm, positionIsAbsolute); }
	);
	connection = QWidget::connect(
		m_scanControl,
		&ScanControl::s_scaleCalibrationChanged,
		this,
		[this](std::vector<POINT2> positions) { on_scaleCalibrationChanged(positions); }
	);
	connection = QWidget::connect(
		m_scanControl,
		&ScanControl::s_positionScannerChanged,
		this,
		[this](POINT2 position) { drawPositionScannerMarker(position); }
	);
	tableModel->setStorage(m_scanControl->getSavedPositionsNormalized());

	m_acquisitionThread.startWorker(m_scanControl);

	QMetaObject::invokeMethod(
		m_scanControl,
		[&m_scanControl = m_scanControl]() {
			m_scanControl->connectDevice();
		},
		Qt::AutoConnection
	);

	loadLinkedObjectiveCalibrations();

	// Deferred: the restored marker is only valid for the objective it was saved under, and
	// that objective's slot/calibration aren't necessarily known yet at this point (both
	// connectDevice() and loadLinkedObjectiveCalibrations() above are asynchronous) - see
	// ScanControl::setPendingRestoredMarker()'s own comment.
	m_scanControl->setPendingRestoredMarker(m_positionScanner, m_positionScannerObjectiveSlot);
}

void BrillouinAcquisition::initODT() {
	if (m_ODT) {
		m_ODT->deleteLater();
		m_ODT = nullptr;
	}
	if (!m_scanControl->supportsCapability(Capabilities::ODT)) {
		int tabIndexODT = ui->acquisitionModeTabs->indexOf(ui->ODT);
		if (tabIndexODT > -1) {
			ui->acquisitionModeTabs->removeTab(tabIndexODT);
		}
	} else {
		m_ODT = new ODT(nullptr, m_acquisition, m_brightfieldCamera, (ODTControl*&)m_scanControl);
		// Index 2, not 1: the static "Surface scanning" tab always occupies index 1
		// (right after "Grid"), so ODT's home position - when re-inserted after an
		// earlier removeTab() - is 2, keeping the intended Grid/Surface scanning/ODT/
		// Fluorescence order. insertTab() clamps out-of-range indices to "append", so
		// this stays correct even if Surface scanning were ever removed too.
		ui->acquisitionModeTabs->insertTab(2, ui->ODT, "ODT");

		static QMetaObject::Connection connection;
		connection = QWidget::connect(
			m_ODT,
			&ODT::s_acqSettingsChanged,
			this,
			[this](ODT_SETTINGS settings) { plotODTVoltages(settings, ODT_MODE::ACQ); }
		);

		connection = QWidget::connect(
			m_ODT,
			&ODT::s_algnSettingsChanged,
			this,
			[this](ODT_SETTINGS settings) { plotODTVoltages(settings, ODT_MODE::ALGN); }
		);

		connection = QWidget::connect(
			m_ODT,
			&ODT::s_mirrorVoltageChanged,
			this,
			[this](VOLTAGE2 voltage, ODT_MODE mode) { plotODTVoltage(voltage, mode); }
		);

		connection = QWidget::connect(
			m_ODT,
			&ODT::s_cameraSettingsChanged,
			this,
			[this](CAMERA_SETTINGS settings) { updateODTCameraSettings(settings); }
		);

		// slot to show current acquisition state
		connection = QWidget::connect(
			m_ODT,
			&ODT::s_acquisitionStatus,
			this,
			[this](ACQUISITION_STATUS state) { showODTStatus(state); }
		);

		// slot to show current repetition progress
		connection = QWidget::connect(
			m_ODT,
			&ODT::s_repetitionProgress,
			this,
			[this](double progress, int seconds) { showODTProgress(progress, seconds); }
		);

		// start ODT thread
		m_acquisitionThread.startWorker(m_ODT);
		m_ODT->initialize();
	}
}

void BrillouinAcquisition::initVoltageCalibration() {
	if (m_voltageCalibration) {
		m_voltageCalibration->deleteLater();
		m_voltageCalibration = nullptr;
	}
	if (!m_scanControl->supportsCapability(Capabilities::VoltageCalibration)) {
		ui->menu_Voltage_calibration->menuAction()->setVisible(false);
	} else {
		m_voltageCalibration = new VoltageCalibration(nullptr, m_acquisition, m_brightfieldCamera, (ODTControl*&)m_scanControl);
		ui->menu_Voltage_calibration->menuAction()->setVisible(true);

		static QMetaObject::Connection connection;
		connection = QWidget::connect(
			m_voltageCalibration,
			&VoltageCalibration::s_cameraSettingsChanged,
			this,
			[this](CAMERA_SETTINGS settings) { updateODTCameraSettings(settings); }
		);
		connection = QWidget::connect(
			m_voltageCalibration,
			&VoltageCalibration::s_voltageCalibrationStatus,
			this,
			[this](std::string title, std::string message) { showScaleCalibrationStatus(title, message); }
		);

		// start Calibration thread
		m_acquisitionThread.startWorker(m_voltageCalibration);
	}
}

void BrillouinAcquisition::initScaleCalibration() {
	// If the scanControl does not support ScaleCalibration, we hide the "Acquire" entry.
	if (!m_scanControl->supportsCapability(Capabilities::ScaleCalibration)) {
		ui->action_Scale_calibration_acquire->setVisible(false);
	} else {
		ui->action_Scale_calibration_acquire->setVisible(true);
	}

	// Initialize scaleCalibration if it is not running already.
	if (!m_scaleCalibration) {
		m_scaleCalibration = new ScaleCalibration(nullptr, m_acquisition, m_brightfieldCamera, m_scanControl);
		// start Calibration thread
		m_acquisitionThread.startWorker(m_scaleCalibration);

		// Connect signals/slots for updating values
		auto connection = QWidget::connect(
			m_scaleCalibration,
			&ScaleCalibration::s_Ds_changed,
			this,
			[this](POINT2 translation) { updateScaleCalibrationTranslationValue(translation); }
		);
		connection = QWidget::connect(
			m_scaleCalibration,
			&ScaleCalibration::s_scaleCalibrationChanged,
			this,
			[this](ScaleCalibrationData scaleCalibration) { updateScaleCalibrationData(scaleCalibration); }
		);
		connection = QWidget::connect(
			m_scaleCalibration,
			&ScaleCalibration::s_objectiveCalibrationChanged,
			this,
			[this](ObjectiveCalibrationData calibration) { updateObjectiveCalibrationData(calibration); }
		);
		connection = QWidget::connect(
			m_scaleCalibration,
			&ScaleCalibration::s_scaleCalibrationAcquisitionProgress,
			this,
			[this](double progress) { updateScaleCalibrationAcquisitionProgress(progress); }
		);
		connection = QWidget::connect(
			m_scaleCalibration,
			&ScaleCalibration::s_scaleCalibrationStatus,
			this,
			[this](std::string title, std::string message) { showScaleCalibrationStatus(title, message); }
		);
		connection = QWidget::connect(
			m_scaleCalibration,
			&ScaleCalibration::s_fovOffsetSaved,
			this,
			[this](int slot, POINT2 oldOffsetUm, bool oldHasFovOffset, POINT2 newOffsetUm, bool newHasFovOffset) {
				onFovOffsetSaved(slot, oldOffsetUm, oldHasFovOffset, newOffsetUm, newHasFovOffset);
			}
		);
	}
}

void BrillouinAcquisition::initFluorescence() {
	if (m_Fluorescence) {
		m_Fluorescence->deleteLater();
		m_Fluorescence = nullptr;
	}
	if (!m_hasFluorescence) {
		int tabIndexFluorescence = ui->acquisitionModeTabs->indexOf(ui->Fluorescence);
		if (tabIndexFluorescence > -1) {
			ui->acquisitionModeTabs->removeTab(tabIndexFluorescence);
		}
	} else {
		m_Fluorescence = new Fluorescence(nullptr, m_acquisition, m_brightfieldCamera, m_scanControl, m_Brillouin);
		// Index 3, not 2: see the matching comment in initODT() - "Surface scanning" (1)
		// and, when present, ODT (2) both come before Fluorescence now. insertTab()
		// clamps out-of-range indices to "append", so this still lands right after
		// whichever of those tabs actually exist.
		ui->acquisitionModeTabs->insertTab(3, ui->Fluorescence, "Fluorescence");

		static QMetaObject::Connection connection;
		connection = QWidget::connect(
			m_Fluorescence,
			&Fluorescence::s_acqSettingsChanged,
			this,
			[this](FLUORESCENCE_SETTINGS settings) { updateFluorescenceSettings(settings); }
		);

		// slot to show current acquisition state of Fluorescence mode
		connection = QWidget::connect(
			m_Fluorescence,
			&Fluorescence::s_acquisitionStatus,
			this,
			[this](ACQUISITION_STATUS state) { showFluorescenceStatus(state); }
		);

		// slot to show current repetition progress
		connection = QWidget::connect(
			m_Fluorescence,
			&Fluorescence::s_repetitionProgress,
			this,
			[this](double progress, int seconds) { showFluorescenceProgress(progress, seconds); }
		);

		// slot to show current repetition progress
		connection = QWidget::connect(
			m_Fluorescence,
			&Fluorescence::s_previewRunning,
			this,
			[this](FLUORESCENCE_MODE mode) { showFluorescencePreviewRunning(mode); }
		);

		// start Fluorescence thread
		m_acquisitionThread.startWorker(m_Fluorescence);
		m_Fluorescence->initialize();

	}
}

void BrillouinAcquisition::initCameraBrillouin() {
	// deinitialize camera if necessary
	if (m_andor) {
		m_andor->deleteLater();
		m_andor = nullptr;
	}

	// initialize correct camera type
	switch (m_cameraBrillouinType) {
		case CAMERA_BRILLOUIN_DEVICE::ANDOR:
			m_andor = new Andor();
			break;
		case CAMERA_BRILLOUIN_DEVICE::PVCAM:
			m_andor = new PVCamera();
			break;
#ifdef _DEBUG
		case CAMERA_BRILLOUIN_DEVICE::MOCK:
			m_andor = new MockCamera();
			break;
#endif
		default:
			m_andor = new Andor();
			break;
	}
	// Select which camera to connect to
	m_andor->setCameraNumber(m_cameraBrillouinNumber);

	// slot camera connection
	static QMetaObject::Connection connection;
	connection = QWidget::connect(
		m_andor,
		&Camera::s_previewBufferSettingsChanged,
		this,
		[this] { updatePlotLimits(m_BrillouinPlot, m_cameraOptions, m_andor->m_previewBuffer->m_bufferSettings.roi); }
	);

	connection = QWidget::connect(
		m_andor,
		&Camera::connectedDevice,
		this,
		[this](bool isConnected) { cameraConnectionChanged(isConnected); }
	);

	connection = QWidget::connect(
		m_andor,
		&Camera::noCameraFound,
		this,
		[this] { showNoCameraFound(); }
	);

	connection = QWidget::connect(
		m_andor,
		&Camera::cameraCoolingChanged,
		this,
		[this](bool isCooling) { cameraCoolingChanged(isCooling); }
	);

	connection = QWidget::connect(
		m_andor,
		&Camera::s_imageReady,
		this,
		[this] { updateImageBrillouin(); }
	);

	connection = QWidget::connect(
		m_andor,
		&Camera::s_previewRunning,
		this,
		[this](bool isRunning) { showPreviewRunning(isRunning); }
	);

	connection = QWidget::connect(
		m_andor,
		&Camera::s_acquisitionRunning,
		this,
		[this](bool isRunning) { startPreview(isRunning); }
	);

	connection = QWidget::connect(
		m_andor,
		&Camera::optionsChanged,
		this,
		[this](CAMERA_OPTIONS options) { cameraOptionsChanged(options); }
	);

	connection = QWidget::connect(
		m_andor,
		&Camera::settingsChanged,
		this,
		[this](CAMERA_SETTINGS settings) { cameraSettingsChanged(settings); }
	);

	connection = QWidget::connect(
		m_andor,
		&Camera::s_sensorTemperatureChanged,
		this,
		[this](SensorTemperature sensorTemperature) { sensorTemperatureChanged(sensorTemperature); }
	);

	// start andor thread
	m_andorThread.startWorker(m_andor);

	QMetaObject::invokeMethod(
		m_andor,
		[&m_andor = m_andor]() {
			m_andor->connectDevice();
		},
		Qt::AutoConnection
	);
}

void BrillouinAcquisition::initCamera() {
	// deinitialize camera if necessary
	if (m_brightfieldCamera) {
		m_brightfieldCamera->deleteLater();
		m_brightfieldCamera = nullptr;
	}

	// initialize correct camera type
	int tabIndexCamera = ui->settingsWidget->indexOf(ui->ODTcameraTab);
	switch (m_cameraType) {
		case CAMERA_DEVICE::NONE:
			m_brightfieldCamera = nullptr;
			ui->actionConnect_Brightfield_camera->setVisible(false);
			if (tabIndexCamera > -1) {
				ui->settingsWidget->removeTab(tabIndexCamera);
			}
			m_hasFluorescence = false;
			break;
		case CAMERA_DEVICE::POINTGREY:
			m_brightfieldCamera = new PointGrey();
			ui->actionConnect_Brightfield_camera->setVisible(true);
			ui->settingsWidget->addTab(ui->ODTcameraTab, "ODT Camera");
			ui->settingsWidget->setTabIcon(3, m_icons.disconnected);
			m_hasFluorescence = true;
			break;
		case CAMERA_DEVICE::UEYE:
			m_brightfieldCamera = new uEyeCam();
			ui->actionConnect_Brightfield_camera->setVisible(true);
			ui->settingsWidget->addTab(ui->ODTcameraTab, "ODT Camera");
			ui->settingsWidget->setTabIcon(3, m_icons.disconnected);
			m_hasFluorescence = true;
			break;
#ifdef _DEBUG
		case CAMERA_DEVICE::MOCK:
			m_brightfieldCamera = new MockCamera();
			ui->actionConnect_Brightfield_camera->setVisible(true);
			ui->settingsWidget->addTab(ui->ODTcameraTab, "ODT Camera");
			ui->settingsWidget->setTabIcon(3, m_icons.disconnected);
			m_hasFluorescence = true;
			break;
#endif
		default:
			m_brightfieldCamera = nullptr;
			ui->actionConnect_Brightfield_camera->setVisible(false);
			if (tabIndexCamera > -1) {
				ui->settingsWidget->removeTab(tabIndexCamera);
			}
			m_hasFluorescence = false;
			break;
	}

	// init or de-init fluorescence
	initFluorescence();

	// don't do anything if no camera is connected
	if (m_cameraType == CAMERA_DEVICE::NONE) {
		return;
	}

	// reestablish camera connections
	QMetaObject::Connection connection = QWidget::connect(
		m_brightfieldCamera,
		&Camera::connectedDevice,
		this,
		[this](bool isConnected) { brightfieldCameraConnectionChanged(isConnected); }
	);

	connection = QWidget::connect(
		m_brightfieldCamera,
		&Camera::s_imageReady,
		this,
		[this] { updateImageODT(); }
	);

	connection = QWidget::connect(
		m_brightfieldCamera,
		&Camera::s_previewBufferSettingsChanged,
		this,
		[this] { updatePlotLimits(m_ODTPlot, m_cameraOptionsODT, m_brightfieldCamera->m_previewBuffer->m_bufferSettings.roi); }
	);

	connection = QWidget::connect(
		m_brightfieldCamera,
		&Camera::s_previewRunning,
		this,
		[this](bool isRunning) { showBrightfieldPreviewRunning(isRunning); }
	);

	connection = QWidget::connect(
		m_brightfieldCamera,
		&Camera::s_acquisitionRunning,
		this,
		[this](bool isRunning) { startBrightfieldPreview(isRunning); }
	);

	connection = QWidget::connect(
		m_brightfieldCamera,
		&Camera::settingsChanged,
		this,
		[this](CAMERA_SETTINGS settings) { cameraODTSettingsChanged(settings); }
	);

	connection = QWidget::connect(
		m_brightfieldCamera,
		&Camera::optionsChanged,
		this,
		[this](CAMERA_OPTIONS options) { cameraODTOptionsChanged(options); }
	);

	m_brightfieldCameraThread.startWorker(m_brightfieldCamera);

	QMetaObject::invokeMethod(
		m_brightfieldCamera,
		[&m_brightfieldCamera = m_brightfieldCamera]() {
			m_brightfieldCamera->connectDevice();
		},
		Qt::AutoConnection
	);
}

void BrillouinAcquisition::microscopeElementPositionsChanged(const std::vector<double>& positions) {
	m_deviceElementPositions = positions;
	checkElementButtons();
}

void BrillouinAcquisition::microscopeElementPositionChanged(DeviceElement element, double position) {
	if (m_deviceElementPositions.size() <= element.index) {
		m_deviceElementPositions.resize((size_t)element.index + 1);
	}
	m_deviceElementPositions[element.index] = position;
	checkElementButtons();
}

void BrillouinAcquisition::objectiveSwitched(int previousSlot, int newSlot, bool hasCalibration, bool hasFovOffset, POINT2 offsetUm, double offsetSigmaUm) {
	// Unconditional and first - the read-only name/magnification display and the "compare to"
	// pairwise offset need to follow the active slot regardless of whether this switch triggers
	// one of the warnings below (a no-op if the Scale Calibration dialog is not currently open).
	refreshScaleCalibrationObjectiveDisplay();

	if (!hasCalibration) {
		// Suppressed during an automated multi-cycle run (see
		// ScaleCalibration::startObjectiveCycleCalibration()) - a blocking popup on every
		// switch would defeat the point of automating this. The early return below is
		// unaffected either way: with no scale calibration at all for this objective, there is
		// nothing meaningful for updatePositions()/the stage-nudge logic further down to
		// recompute.
		if (!m_suppressObjectiveSwitchWarnings) {
			QMessageBox::warning(
				this,
				"No Scale Calibration For This Objective",
				QString("No scale calibration is registered for the objective in slot %1.\n\n"
					"Pixel<->micrometer conversions, grids, ROIs and overview tiles for this objective "
					"will be wrong until a calibration is loaded for it.").arg(newSlot)
			);
		}
		return;
	}
	if (!hasFovOffset) {
		if (m_suppressObjectiveSwitchWarnings) {
			// The target slot of an automated run lacking a FOV offset is expected - it is the
			// thing being calibrated - not something to interrupt for. Treat it as accepted,
			// same as the operator clicking "Yes" below would.
			QMetaObject::invokeMethod(
				m_scanControl,
				[scanControl = m_scanControl]() { scanControl->acceptMissingObjectiveOffset(); },
				Qt::AutoConnection
			);
		} else {
			// Plain acknowledgment, not a "Continue anyway?" choice - the objective switch has
			// already physically happened by the time this fires, so declining doesn't undo or
			// prevent anything (unlike, say, a confirmation before a destructive action).
			// Leaving the offset unaccepted would only make this same dialog reappear next time,
			// not add a meaningful safeguard - so always accept it.
			QMessageBox::warning(
				this,
				"No FOV-Center Offset For This Objective Switch",
				QString("No calibrated FOV-center offset is stored for this objective switch (slot %1 -> %2).\n\n"
					"In absolute grid-coordinate mode, grids, ROIs and overview tiles will NOT be translated "
					"to compensate for this objective's field-of-view center shift, and measurements may no "
					"longer target the same physical sample location as before the switch. Relative-mode grids "
					"are not affected, since they anchor to wherever the stage is when a measurement starts.")
					.arg(previousSlot).arg(newSlot)
			);
			QMetaObject::invokeMethod(
				m_scanControl,
				[scanControl = m_scanControl]() { scanControl->acceptMissingObjectiveOffset(); },
				Qt::AutoConnection
			);
		}
	} else {
		// hasCalibration && hasFovOffset: nothing needs the operator's attention beyond a log
		// line - deliberately not a dialog, that would be a needless interruption on the
		// common, correctly-calibrated path.
		qInfo(logInfo()) << "Objective switch" << previousSlot << "->" << newSlot
			<< ": applying calibrated FOV-center offset (" << offsetUm.x << "," << offsetUm.y
			<< ") um, sigma" << offsetSigmaUm << "um.";
	}

	// A pure objective switch deliberately does not recompute m_orderedPositions or nudge any
	// relative-mode grid target by the FOV-offset delta: grid points are a plan of fixed
	// PHYSICAL sample targets, and since a pure objective switch never moves the stage, those
	// targets must not move either - not on screen (see getPositionOffset()'s own comment) and
	// not in the real stage-move target computed from m_startPosition. Brillouin::acquire()
	// already calls updatePositions() fresh, right before the real measurement loop starts,
	// using whatever objective is active at that moment, so nothing here needs to pre-empt that.
	// The preview still updates via ScanControl::setScaleCalibration()'s own
	// convertPositionsToPix() re-projection (fires on every switch regardless), just without
	// re-baking the underlying stored µm values. The blue marker is still allowed to show a
	// genuine small shift - it tracks where the beam actually, physically lands, which a
	// non-reference objective's real parcentricity error does change.
}

void BrillouinAcquisition::onFovOffsetSaved(int slot, POINT2 oldOffsetUm, bool oldHasFovOffset, POINT2 newOffsetUm, bool newHasFovOffset) {
	// A no-op by design: fovOffsetUm is not folded into any real measurement target or into
	// resolvedGridOriginUm()/m_startPosition - it only affects how the marker itself is drawn
	// (announcePositionScanner()), which already re-reads the active objective's calibration
	// live on every redraw. Saving a new FOV-offset value has nothing left to recompute or
	// re-anchor.
}

void BrillouinAcquisition::checkElementButtons() {
	if ((elementButtons.size() + elementIntBox.size() + elementDoubleBox.size() + elementSlider.size()) != m_deviceElementPositions.size()) {
		return;
	}

	auto elements = m_scanControl->m_deviceElements;
	int indButton{ 0 };
	int indIntBox{ 0 };
	int indDoubleBox{ 0 };
	int indSlider{ 0 };
	for (gsl::index ii = 0; ii < elements.size(); ii++) {
		if (elements[ii].inputType == DEVICE_INPUT_TYPE::PUSHBUTTON) {
			for (gsl::index jj = 0; jj < elementButtons[indButton].size(); jj++) {
				if ((int)m_deviceElementPositions[indButton] == jj + 1) {
					elementButtons[indButton][jj]->setProperty("class", "active");
				} else {
					elementButtons[indButton][jj]->setProperty("class", "");
				}
				elementButtons[indButton][jj]->style()->unpolish(elementButtons[indButton][jj]);
				elementButtons[indButton][jj]->style()->polish(elementButtons[indButton][jj]);
				elementButtons[indButton][jj]->update();
			}
			indButton++;
		} else if (elements[ii].inputType == DEVICE_INPUT_TYPE::INTBOX) {
			const QSignalBlocker blocker(elementIntBox[indIntBox]);
			elementIntBox[indIntBox]->setValue((int)m_deviceElementPositions[ii]);
			indIntBox++;
		} else if (elements[ii].inputType == DEVICE_INPUT_TYPE::DOUBLEBOX) {
			const QSignalBlocker blocker(elementDoubleBox[indDoubleBox]);
			elementDoubleBox[indDoubleBox]->setValue((double)m_deviceElementPositions[ii]);
			indDoubleBox++;
		} else if (elements[ii].inputType == DEVICE_INPUT_TYPE::SLIDER) {
			const QSignalBlocker blocker1(elementSlider[indSlider]);
			elementSlider[indSlider]->setValue((double)m_deviceElementPositions[ii]);

			const QSignalBlocker blocker2(elementSliderInput[indSlider]);
			elementSliderInput[indSlider]->setValue((double)m_deviceElementPositions[ii]);
			indSlider++;
		}
	}
	auto presets = m_scanControl->m_presets;
	for (gsl::index ii = 0; ii < presets.size(); ii++) {
		if (m_scanControl->isPresetActive(presets[ii].index)) {
			presetButtons[ii]->setProperty("class", "active");
		} else {
			presetButtons[ii]->setProperty("class", "");
		}
		presetButtons[ii]->style()->unpolish(presetButtons[ii]);
		presetButtons[ii]->style()->polish(presetButtons[ii]);
		presetButtons[ii]->update();
	}
}

std::vector<std::string> BrillouinAcquisition::formatObjectiveNamesForBeampath(const std::vector<std::string>& names) const {
	auto formatted = std::vector<std::string>(names.size());
	for (size_t ii = 0; ii < names.size(); ii++) {
		if (names[ii].empty()) {
			formatted[ii] = "E  ";
		} else if (names[ii].size() == 2) {
			// One-digit magnification (e.g. "5x") - left-pad to the same 3-character width as
			// a two-digit name ("10x") so the beampath buttons stay a consistent size.
			formatted[ii] = " " + names[ii];
		} else {
			formatted[ii] = names[ii];
		}
	}
	return formatted;
}

double BrillouinAcquisition::magnificationFromObjectiveName(const std::string& name) const {
	if (name.size() < 2 || name.back() != 'x') {
		return 0.0;
	}
	try {
		return std::stod(name.substr(0, name.size() - 1));
	} catch (...) {
		return 0.0;
	}
}

void BrillouinAcquisition::updateElementButtonLabels(const std::vector<std::string>& objectiveOptionNames) {
	if (!m_scanControl) {
		return;
	}
	// Mirrors checkElementButtons()'s exact indexing: indButton only increments on a
	// PUSHBUTTON element, so it stays correct regardless of how many non-pushbutton elements
	// (or other pushbutton elements, e.g. "RL Shutter") precede "Objective" for this backend.
	auto elements = m_scanControl->m_deviceElements;
	int indButton{ 0 };
	for (gsl::index ii = 0; ii < elements.size(); ii++) {
		if (elements[ii].inputType != DEVICE_INPUT_TYPE::PUSHBUTTON) {
			continue;
		}
		if (elements[ii].name == "Objective" && indButton < (int)elementButtons.size()) {
			for (gsl::index jj = 0; jj < elementButtons[indButton].size() && jj < (gsl::index)objectiveOptionNames.size(); jj++) {
				elementButtons[indButton][jj]->setText(objectiveOptionNames[jj].c_str());
			}
		}
		indButton++;
	}
}

void BrillouinAcquisition::on_actionAbout_triggered() {
	QString clean = "Yes";
	if (Version::VerDirty) {
		clean = "No";
	}
	auto preRelease = QString{ "" };
	if (Version::PRERELEASE.length() > 0) {
		preRelease = QString::fromStdString("-" + Version::PRERELEASE);
	}

	auto debugString = QString{ "" };
	#ifdef _DEBUG
		debugString = QString{ " - Debug" };
	#endif
	QString str = QString("BrillouinAcquisition v%1.%2.%3%4%11 <br> Build from commit: <a href='%5'>%6</a><br>Clean build: %7<br>Author: <a href='mailto:%8?subject=BrillouinAcquisition'>%9</a><br>Date: %10")
		.arg(Version::MAJOR)
		.arg(Version::MINOR)
		.arg(Version::PATCH)
		.arg(preRelease)
		.arg(Version::Url.c_str())
		.arg(Version::Commit.c_str())
		.arg(clean)
		.arg(Version::AuthorEmail.c_str())
		.arg(Version::Author.c_str())
		.arg(Version::Date.c_str())
		.arg(debugString);

	QMessageBox::about(this, tr("About BrillouinAcquisition"), str);
}

void BrillouinAcquisition::on_camera_playPause_clicked() {
	if (!m_andor->m_isPreviewRunning) {
		m_andor->setSettings(m_Brillouin->settings.camera);
		QMetaObject::invokeMethod(
			m_andor,
			[&m_andor = m_andor]() {
				m_andor->startPreview();
			},
			Qt::AutoConnection
		);
	} else {
		m_andor->m_stopPreview = true;
	}
}

void BrillouinAcquisition::on_camera_singleShot_clicked() {
}

void BrillouinAcquisition::on_BrillouinStart_clicked() {
	if (m_Brillouin->getStatus() == ACQUISITION_STATUS::WAITFORSURFACEREVIEW) {
		QMetaObject::invokeMethod(
			m_Brillouin,
			[&m_Brillouin = m_Brillouin]() {
				m_Brillouin->continueAfterSurfaceReview(false);
			},
			Qt::AutoConnection
		);
		return;
	}
	if (m_Brillouin->getStatus() < ACQUISITION_STATUS::STARTED) {
		if (m_Brillouin->settings.useRoiMask && m_Brillouin->settings.roiPolygonUm.size() < 3) {
			QMessageBox::warning(
				this,
				"Invalid ROI Mask",
				"ROI masking is enabled but polygon has fewer than 3 points.\n"
				"Enable Draw ROI and click in the brightfield plot to add points, or right click to clear."
			);
			return;
		}
		if (m_Brillouin->settings.useRoiMask && isSelfIntersectingPolygon(m_Brillouin->settings.roiPolygonUm)) {
			QMessageBox::warning(
				this,
				"Invalid ROI Mask",
				"ROI polygon is self-intersecting.\nAdjust points in Draw ROI mode before starting acquisition."
			);
			return;
		}

		// set camera ROI
		// Copy the whole struct, not just top/left/width_physical/height_physical - .bottom in
		// particular is what remapProxyRoi() needs for the spectral proxy ROI's vertical origin
		// (see cameraSettingsChanged(), which is what keeps m_deviceSettings.camera.roi's copy
		// of it live/correct). A partial copy here left .bottom stale at whatever
		// m_Brillouin->settings.camera.roi last held (its CAMERA_ROI default, in practice, since
		// nothing else ever wrote it) even once everything else was updated to the real crop -
		// exactly the kind of same-looking-but-wrong desync that made a spectral ROI drawn under
		// one crop silently misalign against the signal once a measurement actually started.
		m_Brillouin->settings.camera.roi = m_deviceSettings.camera.roi;
		m_Brillouin->setSettings(m_Brillouin->settings);
		QMetaObject::invokeMethod(
			m_Brillouin,
			[&m_Brillouin = m_Brillouin]() {
				m_Brillouin->startRepetitions();
			},
			Qt::AutoConnection
		);
	} else {
		m_Brillouin->m_abort = true;
	}
}

void BrillouinAcquisition::on_fullGridButton_clicked() {
	if (m_Brillouin->getStatus() != ACQUISITION_STATUS::WAITFORSURFACEREVIEW) {
		return;
	}
	QMetaObject::invokeMethod(
		m_Brillouin,
		[&m_Brillouin = m_Brillouin]() {
			m_Brillouin->continueAfterSurfaceReview(true);
		},
		Qt::AutoConnection
	);
}

void BrillouinAcquisition::updateFilename(const std::string& filename) {
	m_storagePath.filename = filename;
	updateBrillouinSettings();
}

void BrillouinAcquisition::updateBrillouinSettings() {
	ui->acquisitionFilename->setText(QString::fromStdString(m_storagePath.filename));

	// AOI settings
	ui->startX->setValue(m_Brillouin->settings.xMin);
	ui->startY->setValue(m_Brillouin->settings.yMin);
	ui->startZ->setValue(m_Brillouin->settings.zMin);
	ui->endX->setValue(m_Brillouin->settings.xMax);
	ui->endY->setValue(m_Brillouin->settings.yMax);
	ui->endZ->setValue(m_Brillouin->settings.zMax);
	ui->stepsX->setValue(m_Brillouin->settings.xSteps);
	ui->stepsY->setValue(m_Brillouin->settings.ySteps);
	ui->stepsZ->setValue(m_Brillouin->settings.zSteps);

	// calibration settings
	ui->preCalibration->setChecked(m_Brillouin->settings.preCalibration);
	ui->postCalibration->setChecked(m_Brillouin->settings.postCalibration);
	ui->conCalibration->setChecked(m_Brillouin->settings.conCalibration);
	ui->conCalibrationInterval->setValue(m_Brillouin->settings.conCalibrationInterval);
	ui->nrCalibrationImages->setValue(m_Brillouin->settings.nrCalibrationImages);
	ui->calibrationExposureTime->setValue(m_Brillouin->settings.calibrationExposureTime);
	ui->sampleSelection->setCurrentText(QString::fromStdString(m_Brillouin->settings.sample));

	// repetition settings
	ui->repetitionCount->setValue(m_Brillouin->settings.repetitions.count);
	ui->repetitionInterval->setValue(m_Brillouin->settings.repetitions.interval);
	ui->repetitionNewFile->setChecked(m_Brillouin->settings.repetitions.filePerRepetition);

	// Shared with the background ROI - see RoiTarget's own comment.
	updateRoiMaskCheckboxStateFor(mainRoiTarget());
	updateRoiMaskCheckboxStateFor(backgroundRoiTarget());
	if (m_useSurfaceFollowCheckbox) {
		const QSignalBlocker blocker(*m_useSurfaceFollowCheckbox);
		m_useSurfaceFollowCheckbox->setChecked(m_Brillouin->settings.useSurfaceFollow);
	}
	if (m_preScanXYBinSpinBox) {
		const QSignalBlocker blocker(*m_preScanXYBinSpinBox);
		m_preScanXYBinSpinBox->setValue(std::max(1, m_Brillouin->settings.preScanXYBin));
		m_preScanXYBinSpinBox->setEnabled(m_Brillouin->settings.useSurfaceFollow);
	}
	if (m_additionalBoundaryPointsSpinBox) {
		const QSignalBlocker blocker(*m_additionalBoundaryPointsSpinBox);
		m_additionalBoundaryPointsSpinBox->setValue(std::max(0, m_Brillouin->settings.additionalBoundaryPoints));
		m_additionalBoundaryPointsSpinBox->setEnabled(m_Brillouin->settings.useSurfaceFollow);
	}
	if (m_preScanZStepSpinBox) {
		const QSignalBlocker blocker(*m_preScanZStepSpinBox);
		m_preScanZStepSpinBox->setValue(std::max(0.01, m_Brillouin->settings.preScanZStepUm));
		m_preScanZStepSpinBox->setEnabled(m_Brillouin->settings.useSurfaceFollow);
	}
	if (m_preScanZTravelSpinBox) {
		const QSignalBlocker blocker(*m_preScanZTravelSpinBox);
		m_preScanZTravelSpinBox->setValue(std::max(0.01, m_Brillouin->settings.preScanZTravelRangeUm));
		m_preScanZTravelSpinBox->setEnabled(m_Brillouin->settings.useSurfaceFollow);
	}
	if (m_surfaceDropSpinBox) {
		const QSignalBlocker blocker(*m_surfaceDropSpinBox);
		m_surfaceDropSpinBox->setValue(100.0 * m_Brillouin->settings.surfaceDropFraction);
		m_surfaceDropSpinBox->setEnabled(m_Brillouin->settings.useSurfaceFollow);
	}
	if (m_mediumReferenceFrameCountSpinBox) {
		const QSignalBlocker blocker(*m_mediumReferenceFrameCountSpinBox);
		m_mediumReferenceFrameCountSpinBox->setValue(std::max(1, m_Brillouin->settings.mediumReferenceFrameCount));
		m_mediumReferenceFrameCountSpinBox->setEnabled(m_Brillouin->settings.useSurfaceFollow);
	}
	if (m_surfaceMaxRewindSpinBox) {
		const QSignalBlocker blocker(*m_surfaceMaxRewindSpinBox);
		m_surfaceMaxRewindSpinBox->setValue(std::max(0.0, m_Brillouin->settings.surfaceMaxRewindUm));
		m_surfaceMaxRewindSpinBox->setEnabled(m_Brillouin->settings.useSurfaceFollow);
	}
	if (m_surfaceVerificationStepsSpinBox) {
		const QSignalBlocker blocker(*m_surfaceVerificationStepsSpinBox);
		m_surfaceVerificationStepsSpinBox->setValue(std::max(0, m_Brillouin->settings.surfaceVerificationSteps));
		m_surfaceVerificationStepsSpinBox->setEnabled(m_Brillouin->settings.useSurfaceFollow);
	}
	if (m_surfaceVerificationFrameAverageSpinBox) {
		const QSignalBlocker blocker(*m_surfaceVerificationFrameAverageSpinBox);
		m_surfaceVerificationFrameAverageSpinBox->setValue(std::max(1, m_Brillouin->settings.surfaceVerificationFrameAverage));
		m_surfaceVerificationFrameAverageSpinBox->setEnabled(m_Brillouin->settings.useSurfaceFollow);
	}
	if (m_surfaceVerificationToleranceSpinBox) {
		const QSignalBlocker blocker(*m_surfaceVerificationToleranceSpinBox);
		m_surfaceVerificationToleranceSpinBox->setValue(100.0 * m_Brillouin->settings.surfaceVerificationToleranceFraction);
		m_surfaceVerificationToleranceSpinBox->setEnabled(m_Brillouin->settings.useSurfaceFollow);
	}
	if (m_absoluteGridCheckbox) {
		const QSignalBlocker blocker(*m_absoluteGridCheckbox);
		m_absoluteGridCheckbox->setChecked(m_Brillouin->settings.gridCoordinatesAbsolute);
	}
	if (m_gridHysteresisCompensationCheckbox) {
		const QSignalBlocker blocker(*m_gridHysteresisCompensationCheckbox);
		m_gridHysteresisCompensationCheckbox->setChecked(m_Brillouin->settings.useGridHysteresisCompensation);
	}
	if (m_doseProtectionCheckbox) {
		const QSignalBlocker blocker(*m_doseProtectionCheckbox);
		m_doseProtectionCheckbox->setChecked(m_Brillouin->settings.useDoseProtection);
	}
	if (m_saveOverviewBrightfieldPerZCheckbox) {
		const QSignalBlocker blocker(*m_saveOverviewBrightfieldPerZCheckbox);
		m_saveOverviewBrightfieldPerZCheckbox->setChecked(m_Brillouin->settings.saveOverviewBrightfieldPerZ);
		m_saveOverviewBrightfieldPerZCheckbox->setEnabled(
			m_hasFluorescence && m_brightfieldCamera != nullptr && m_brightfieldCamera->getConnectionStatus()
		);
	}
	// Coverage-mode controls work in both absolute and relative grid mode (see
	// Brillouin::overviewImageXY()) - they only actually need per-Z overview capture to be
	// enabled at all, or they'd be settings that silently do nothing.
	{
		const auto overviewPossible = m_Brillouin->settings.saveOverviewBrightfieldPerZ;
		if (m_overviewSingleImageRadio) {
			const QSignalBlocker blocker(*m_overviewSingleImageRadio);
			m_overviewSingleImageRadio->setChecked(!m_Brillouin->settings.overviewBrightfieldFullGrid);
			m_overviewSingleImageRadio->setEnabled(overviewPossible);
		}
		if (m_overviewFullGridRadio) {
			const QSignalBlocker blocker(*m_overviewFullGridRadio);
			m_overviewFullGridRadio->setChecked(m_Brillouin->settings.overviewBrightfieldFullGrid);
			m_overviewFullGridRadio->setEnabled(overviewPossible);
		}
		if (m_overviewFullStackCheckbox) {
			const QSignalBlocker blocker(*m_overviewFullStackCheckbox);
			m_overviewFullStackCheckbox->setChecked(
				m_Brillouin->settings.overviewBrightfieldFullGrid
				? m_Brillouin->settings.overviewBrightfieldFullStackMosaic
				: m_Brillouin->settings.overviewBrightfieldFullStackSingle
			);
			m_overviewFullStackCheckbox->setEnabled(overviewPossible);
		}
		updateOverviewTileOutlines();
	}
	// Independent of the per-z overview above (capturePerPointBrightfield is its own,
	// separate setting) - only needs the brightfield camera itself connected, same
	// precondition saveOverviewBrightfieldPerZCheckbox above uses.
	{
		const auto perPointPossible = m_brightfieldCamera != nullptr && m_brightfieldCamera->getConnectionStatus();
		if (m_capturePerPointBrightfieldCheckbox) {
			const QSignalBlocker blocker(*m_capturePerPointBrightfieldCheckbox);
			m_capturePerPointBrightfieldCheckbox->setChecked(m_Brillouin->settings.capturePerPointBrightfield);
			m_capturePerPointBrightfieldCheckbox->setEnabled(perPointPossible);
		}
		if (m_perPointBrightfieldEveryNSpinBox) {
			const QSignalBlocker blocker(*m_perPointBrightfieldEveryNSpinBox);
			m_perPointBrightfieldEveryNSpinBox->setValue(m_Brillouin->settings.perPointBrightfieldEveryN);
			m_perPointBrightfieldEveryNSpinBox->setEnabled(perPointPossible && m_Brillouin->settings.capturePerPointBrightfield);
		}
		if (m_perPointBrightfieldDuringAcquisitionCheckbox) {
			const QSignalBlocker blocker(*m_perPointBrightfieldDuringAcquisitionCheckbox);
			m_perPointBrightfieldDuringAcquisitionCheckbox->setChecked(m_Brillouin->settings.perPointBrightfieldDuringAcquisition);
			m_perPointBrightfieldDuringAcquisitionCheckbox->setEnabled(perPointPossible && m_Brillouin->settings.capturePerPointBrightfield);
		}
	}
	// See the ACQUISITION_STATUS handler's identical setHome lines for why it's enabled/
	// relabeled rather than disabled in absolute mode.
	ui->setHome->setDisabled(m_enabledModes != ACQUISITION_MODE::NONE);
	ui->setHome->setText(m_Brillouin->settings.gridCoordinatesAbsolute ? "Set plane" : "Set home");
	ui->moveHome->setDisabled(m_Brillouin->settings.gridCoordinatesAbsolute || m_enabledModes != ACQUISITION_MODE::NONE);
	// See the comment on this same lock in the ACQUISITION_STATUS handler - repeated here so
	// it stays correct across every path that refreshes the grid UI (e.g. an objective switch
	// re-running updatePositions()), not just the toggle handler and the status handler. Z is
	// excluded from the absolute-mode part of the lock - see that comment for why.
	const auto gridLockedXY = m_Brillouin->settings.gridCoordinatesAbsolute || m_enabledModes != ACQUISITION_MODE::NONE;
	const auto gridLockedZ = m_enabledModes != ACQUISITION_MODE::NONE;
	ui->startX->setDisabled(gridLockedXY);
	ui->startY->setDisabled(gridLockedXY);
	ui->startZ->setDisabled(gridLockedZ);
	ui->endX->setDisabled(gridLockedXY);
	ui->endY->setDisabled(gridLockedXY);
	ui->endZ->setDisabled(gridLockedZ);
	ui->stepsX->setDisabled(gridLockedXY);
	ui->stepsY->setDisabled(gridLockedXY);
	ui->stepsZ->setDisabled(gridLockedZ);
	if (m_editSpectralProxyRoiCheckbox) {
		m_editSpectralProxyRoiCheckbox->setEnabled(m_Brillouin->settings.useSurfaceFollow);
	}

	refreshSpectralProxyRoiRects();
	ui->customplot->replot();
	updateEstimatedAcquisitionTime();
	updateBrillouinStartAvailability();
	updateAbsoluteGridStatus();
}

void BrillouinAcquisition::on_startX_valueChanged(double value) {
	m_Brillouin->setXMin(value);
	updateBrillouinSettings();
}

void BrillouinAcquisition::on_startY_valueChanged(double value) {
	m_Brillouin->setYMin(value);
	updateBrillouinSettings();
}

void BrillouinAcquisition::on_startZ_valueChanged(double value) {
	m_Brillouin->setZMin(value);
	updateBrillouinSettings();
}

void BrillouinAcquisition::on_endX_valueChanged(double value) {
	m_Brillouin->setXMax(value);
	updateBrillouinSettings();
}

void BrillouinAcquisition::on_endY_valueChanged(double value) {
	m_Brillouin->setYMax(value);
	updateBrillouinSettings();
}

void BrillouinAcquisition::on_endZ_valueChanged(double value) {
	m_Brillouin->setZMax(value);
	updateBrillouinSettings();
}

void BrillouinAcquisition::on_stepsX_valueChanged(int value) {
	m_Brillouin->setStepNumberX(value);
	updateBrillouinSettings();
}

void BrillouinAcquisition::on_stepsY_valueChanged(int value) {
	m_Brillouin->setStepNumberY(value);
	updateBrillouinSettings();
}

void BrillouinAcquisition::on_stepsZ_valueChanged(int value) {
	m_Brillouin->setStepNumberZ(value);
	updateBrillouinSettings();
}

void BrillouinAcquisition::on_showOverlay_stateChanged(int show) {
	m_showPositions = show;
	update_AOI_preview();
}

/*
 * React when the ordered positions have changed
 */
void BrillouinAcquisition::AOI_changed(const std::vector<POINT3>& orderedPositions, bool isAbsolute) {
	m_positionsMicrometerIsAbsolute = isAbsolute;
	m_positionsComputed = true;
	if (m_scanControl) {
		m_positionsMicrometer = orderedPositions;
		// isAbsolute is the mode these positions were actually computed under (travels with
		// the signal - see Brillouin::s_orderedPositionsChanged()'s own comment), NOT
		// m_Brillouin->settings.gridCoordinatesAbsolute's current, possibly-already-changed-
		// again live value - using the live value here reintroduced exactly the race this
		// parameter exists to avoid.
		m_positionsPixel = m_scanControl->getPositionsPix(m_positionsMicrometer, isAbsolute);
		std::transform(m_positionsPixel.begin(), m_positionsPixel.end(), m_positionsPixel.begin(),
			[this](POINT2 point) { return brightfieldRawToDisplay(point); }
		);
		// Refresh the cached grid offset from the exact same call chain (getPositionsPix() ->
		// convertPositionsToPix()) that just computed m_positionsPixel, for the same isAbsolute -
		// not from on_gridOffsetChanged()'s separately-queued s_gridOffsetChanged signal, which
		// only updates on ScanControl's own announcePositions()/setScaleCalibration() triggers
		// and was therefore left stale exactly when THIS function (a grid recompute) was the one
		// that actually moved the pixel positions - see currentGridOffset()'s own comment on why
		// update_AOI_preview()'s ROI-coloring branch depends on this cache being fresh.
		m_currentGridOffsetUm = m_scanControl->getPositionOffset(isAbsolute);
		m_currentGridOffsetIsAbsolute = isAbsolute;
		update_AOI_preview();
	}
	updateEstimatedAcquisitionTime();
}

/*
 * React when the set of grid points the ROI mask excludes has changed (preview-only, see
 * ScanPlannerOutput::excludedPositionsAbsolute/Relative) - drives the red "outside ROI" markers.
 */
void BrillouinAcquisition::excludedAOI_changed(const std::vector<POINT3>& excludedPositions) {
	m_excludedPositionsMicrometer = excludedPositions;
	if (m_scanControl) {
		update_AOI_preview();
	}
}

/*
 * React when the scancontrol settings have changed, e.g. the start position was adjusted
 */
void BrillouinAcquisition::on_scaleCalibrationChanged(const std::vector<POINT2>& positions) {
	m_positionsPixel = positions;
	std::transform(m_positionsPixel.begin(), m_positionsPixel.end(), m_positionsPixel.begin(),
		[this](POINT2 point) { return brightfieldRawToDisplay(point); }
	);
	// This is the path a PURE objective switch actually takes to move the on-screen grid dots
	// (ScanControl::setScaleCalibration() -> convertPositionsToPix() -> this slot), completely
	// separate from AOI_changed()/m_positionsMicrometerIsAbsolute - this function overwrites
	// m_positionsPixel directly from whatever ScanControl computed, using ScanControl's OWN
	// cached m_AOI_positionsAbsolute, not this class's mode flag at all.
	update_AOI_preview();
}

/*
 * ScanControl emits this immediately before s_scaleCalibrationChanged, from the exact same
 * getPositionOffset() call the just-emitted pixel positions were computed with - see
 * m_currentGridOffsetUm's declaration for why update_AOI_preview()/updateRoiPolygonPreview()
 * must use this cached snapshot rather than calling getPositionOffset() live.
 */
void BrillouinAcquisition::on_gridOffsetChanged(POINT2 offsetUm, bool positionIsAbsolute) {
	m_currentGridOffsetUm = offsetUm;
	m_currentGridOffsetIsAbsolute = positionIsAbsolute;
}

/*
 * Update the plot showing the measurement positions as overlay in the brightfield preview
 */
void BrillouinAcquisition::update_AOI_preview() {
	if (m_showPositions) {
		// Paused for surface-scan review: show only the actual measurement-grid points
		// that ended up with a surface z value (found or interpolated) as squares, with no
		// cross markers - reviewMode below overrides the ordinary pre-scan coarse-grid
		// preview squares and hides the crosses, but keeps their connecting line.
		const bool reviewMode = m_Brillouin->getStatus() == ACQUISITION_STATUS::WAITFORSURFACEREVIEW;
		const bool showSurfaceSquares = reviewMode || m_Brillouin->settings.useSurfaceFollow;
		const bool colorByRoi = m_scanControl
			&& m_Brillouin->settings.useRoiMask
			&& m_Brillouin->settings.roiPolygonUm.size() >= 3;
		// m_positionsPixel is already the correct, mode-aware projection of the current
		// grid (ScanControl::convertPositionsToPix() branches on absolute vs. relative
		// mode internally and both are mathematically consistent with the polygon
		// projection below).
		auto positionsPixelForRoi = m_positionsPixel;
		std::vector<POINT2> excludedPixelForRoi;
		if (colorByRoi && m_scanControl) {
			// Project every overlay (markers, excluded points) from the same cached offset
			// snapshot (see m_currentGridOffsetUm) instead of
			// ScanControl::getPositionsPix()/getPositionPix(), which each re-derive the offset
			// with their own live ScanControl::getPositionOffset() call. ScanControl lives on
			// another thread, so those live calls could each observe a different offset if
			// something there (e.g. enableMeasurementMode(false) at acquisition end) changes
			// mid-way through this function.
			//
			// m_positionsMicrometerIsAbsolute - the mode m_positionsMicrometer/
			// m_excludedPositionsMicrometer were actually computed under (see AOI_changed(),
			// which also refreshes m_currentGridOffsetUm for this same mode) - not the live
			// m_Brillouin->settings.gridCoordinatesAbsolute: this function runs synchronously
			// on the GUI thread, but those arrays were populated asynchronously by an earlier
			// queued signal, so the live mode can already have changed again by the time this
			// runs.
			const auto gridAbsolute = m_positionsMicrometerIsAbsolute;
			const auto offset = currentGridOffset(gridAbsolute);
			positionsPixelForRoi.clear();
			positionsPixelForRoi.reserve(m_positionsMicrometer.size());
			for (const auto& p : m_positionsMicrometer) {
				const auto pix = m_scanControl->microMeterToPix(POINT2{ p.x, p.y } + offset);
				positionsPixelForRoi.push_back(brightfieldRawToDisplay(pix));
			}
			excludedPixelForRoi.reserve(m_excludedPositionsMicrometer.size());
			for (const auto& p : m_excludedPositionsMicrometer) {
				const auto pix = m_scanControl->microMeterToPix(POINT2{ p.x, p.y } + offset);
				excludedPixelForRoi.push_back(brightfieldRawToDisplay(pix));
			}
		}
		QVector<double> squareX;
		QVector<double> squareY;
		const auto& squarePositionsPixel = colorByRoi ? positionsPixelForRoi : m_positionsPixel;
		if (reviewMode && !squarePositionsPixel.empty()) {
			// squarePositionsPixel is index-aligned with getOrderedIndices() - both are
			// projections/copies of the same orderedPositions the surface scan produced.
			const auto orderedIndices = m_Brillouin->getOrderedIndices();
			const auto foundXYIndices = m_Brillouin->getSurfaceFoundXYIndices();
			const auto count = std::min(orderedIndices.size(), squarePositionsPixel.size());
			squareX.reserve((int)count);
			squareY.reserve((int)count);
			for (size_t i = 0; i < count; i++) {
				const auto key = std::make_pair(orderedIndices[i].x, orderedIndices[i].y);
				if (foundXYIndices.find(key) == foundXYIndices.end()) {
					continue;
				}
				squareX.push_back(squarePositionsPixel[i].x);
				squareY.push_back(squarePositionsPixel[i].y);
			}
		} else if (showSurfaceSquares && !squarePositionsPixel.empty() && m_scanControl) {
			// Reuse the exact same µm-space point set runSurfacePreScan() will actually
			// measure (Brillouin::surfacePreScanGridXY(): the uniform coarse grid plus any
			// additional boundary points, built the same way the pre-scan itself builds
			// them), instead of independently reconstructing an
			// approximation from the dense grid's pixel-space bounding box - that
			// reconstruction disagreed with where the pre-scan really goes whenever ROI
			// masking shrank the dense pixel bounding box (or the calibration wasn't a
			// simple uniform scale), which is what made the preview squares look "a bit
			// off" from the actual measured positions.
			// surfacePreScanGridXY() already applies the authoritative µm-space ROI test
			// (the same one runSurfacePreScan() itself uses) internally when useRoiMask is
			// on, which colorByRoi implies - no need for a second, pixel-space ROI test
			// here that could disagree with it.
			const auto gridAbsolute = m_Brillouin->settings.gridCoordinatesAbsolute;
			const auto offset = currentGridOffset(gridAbsolute);
			const auto coarsePoints = m_Brillouin->surfacePreScanGridXY();
			squareX.reserve((int)coarsePoints.size());
			squareY.reserve((int)coarsePoints.size());
			for (const auto& p : coarsePoints) {
				const auto pix = brightfieldRawToDisplay(m_scanControl->microMeterToPix(POINT2{ p.x + offset.x, p.y + offset.y }));
				squareX.push_back(pix.x);
				squareY.push_back(pix.y);
			}
		}
		if (colorByRoi) {
			// positionsPixelForRoi is exactly ScanPlanner's included list and
			// excludedPixelForRoi is exactly what it excluded (see
			// ScanPlannerOutput::excludedPositionsAbsolute/Relative) - both already reflect
			// the real in/out decision the actual scan will use, so there is nothing left to
			// re-test here. Re-testing them against the ROI polygon in pixel space (a second,
			// independent classification) is what let the on-screen coloring disagree with
			// what ScanPlanner would really include, whenever the two tests' notions of the
			// current scanner/stage offset drifted apart even slightly.
			QVector<double> xInside;
			QVector<double> yInside;
			QVector<double> xOutside;
			QVector<double> yOutside;
			xInside.reserve((int)positionsPixelForRoi.size());
			yInside.reserve((int)positionsPixelForRoi.size());
			xOutside.reserve((int)excludedPixelForRoi.size());
			yOutside.reserve((int)excludedPixelForRoi.size());

			for (const auto& posPix : positionsPixelForRoi) {
				xInside.push_back(posPix.x);
				yInside.push_back(posPix.y);
			}
			for (const auto& posPix : excludedPixelForRoi) {
				xOutside.push_back(posPix.x);
				yOutside.push_back(posPix.y);
			}

			if (!m_positionsMarkerInsideRoi) {
				m_positionsMarkerInsideRoi = new QCPCurve(ui->customplot_brightfield->xAxis, ui->customplot_brightfield->yAxis);
				m_positionsMarkerInsideRoi->setLineStyle(QCPCurve::lsLine);
				QPen pen;
				pen.setColor(QColor(0, 170, 0));
				pen.setWidth(2);
				QCPScatterStyle scatterStyle;
				scatterStyle.setShape(QCPScatterStyle::ssCross);
				scatterStyle.setPen(pen);
				scatterStyle.setSize(8);
				m_positionsMarkerInsideRoi->setScatterStyle(scatterStyle);
			}
			if (showSurfaceSquares && !m_positionsMarkerSquareInsideRoi) {
				m_positionsMarkerSquareInsideRoi = new QCPCurve(ui->customplot_brightfield->xAxis, ui->customplot_brightfield->yAxis);
				m_positionsMarkerSquareInsideRoi->setLineStyle(QCPCurve::lsNone);
				QPen pen;
				pen.setColor(QColor(0, 170, 0));
				pen.setWidth(2);
				QCPScatterStyle scatterStyle;
				scatterStyle.setShape(QCPScatterStyle::ssSquare);
				scatterStyle.setPen(pen);
				scatterStyle.setBrush(Qt::NoBrush);
				scatterStyle.setSize(10);
				m_positionsMarkerSquareInsideRoi->setScatterStyle(scatterStyle);
			}
			if (!m_positionsMarkerOutsideRoi) {
				m_positionsMarkerOutsideRoi = new QCPCurve(ui->customplot_brightfield->xAxis, ui->customplot_brightfield->yAxis);
				m_positionsMarkerOutsideRoi->setLineStyle(QCPCurve::lsNone);
				QPen pen;
				pen.setColor(Qt::red);
				pen.setWidth(2);
				QCPScatterStyle scatterStyle;
				scatterStyle.setShape(QCPScatterStyle::ssCross);
				scatterStyle.setPen(pen);
				scatterStyle.setSize(8);
				m_positionsMarkerOutsideRoi->setScatterStyle(scatterStyle);
			}
			// Reviewing a finished surface scan: keep the connecting line but hide the
			// cross markers themselves, since the squares above already show the points
			// that matter now. Re-applied every call (not just at creation) so it tracks
			// reviewMode as it changes.
			{
				auto scatterStyle = m_positionsMarkerInsideRoi->scatterStyle();
				scatterStyle.setShape(reviewMode ? QCPScatterStyle::ssNone : QCPScatterStyle::ssCross);
				m_positionsMarkerInsideRoi->setScatterStyle(scatterStyle);
			}
			{
				auto scatterStyle = m_positionsMarkerOutsideRoi->scatterStyle();
				scatterStyle.setShape(reviewMode ? QCPScatterStyle::ssNone : QCPScatterStyle::ssCross);
				m_positionsMarkerOutsideRoi->setScatterStyle(scatterStyle);
			}
			m_positionsMarkerInsideRoi->setData(xInside, yInside);
			m_positionsMarkerOutsideRoi->setData(xOutside, yOutside);
			if (showSurfaceSquares && m_positionsMarkerSquareInsideRoi) {
				m_positionsMarkerSquareInsideRoi->setData(squareX, squareY);
			}

			if (m_positionsMarker && ui->customplot_brightfield->removePlottable(m_positionsMarker)) {
				m_positionsMarker = nullptr;
			}
			if (m_positionsMarkerSquare && ui->customplot_brightfield->removePlottable(m_positionsMarkerSquare)) {
				m_positionsMarkerSquare = nullptr;
			}
			if (m_positionsMarkerSquareOutsideRoi && ui->customplot_brightfield->removePlottable(m_positionsMarkerSquareOutsideRoi)) {
				m_positionsMarkerSquareOutsideRoi = nullptr;
			}
			if (!showSurfaceSquares && m_positionsMarkerSquareInsideRoi && ui->customplot_brightfield->removePlottable(m_positionsMarkerSquareInsideRoi)) {
				m_positionsMarkerSquareInsideRoi = nullptr;
			}
		} else {
			QVector<double> xPos(m_positionsPixel.size());
			QVector<double> yPos(m_positionsPixel.size());
			int index{ 0 };
			for (auto const& position : m_positionsPixel) {
				xPos[index] = position.x;
				yPos[index] = position.y;
				++index;
			}
			// Single-color legacy marker when ROI mask is not active.
			if (!m_positionsMarker) {
				m_positionsMarker = new QCPCurve(ui->customplot_brightfield->xAxis, ui->customplot_brightfield->yAxis);
				m_positionsMarker->setLineStyle(QCPCurve::lsLine);
				QPen pen;
				pen.setColor(Qt::red);
				pen.setWidth(2);
				QCPScatterStyle scatterStyle;
				scatterStyle.setShape(QCPScatterStyle::ssCross);
				scatterStyle.setPen(pen);
				scatterStyle.setSize(8);
				m_positionsMarker->setScatterStyle(scatterStyle);
			}
			if (showSurfaceSquares && !m_positionsMarkerSquare) {
				m_positionsMarkerSquare = new QCPCurve(ui->customplot_brightfield->xAxis, ui->customplot_brightfield->yAxis);
				m_positionsMarkerSquare->setLineStyle(QCPCurve::lsNone);
				QPen pen;
				pen.setColor(Qt::red);
				pen.setWidth(2);
				QCPScatterStyle scatterStyle;
				scatterStyle.setShape(QCPScatterStyle::ssSquare);
				scatterStyle.setPen(pen);
				scatterStyle.setBrush(Qt::NoBrush);
				scatterStyle.setSize(10);
				m_positionsMarkerSquare->setScatterStyle(scatterStyle);
			}
			// See the colorByRoi branch above for why this is re-applied every call.
			{
				auto scatterStyle = m_positionsMarker->scatterStyle();
				scatterStyle.setShape(reviewMode ? QCPScatterStyle::ssNone : QCPScatterStyle::ssCross);
				m_positionsMarker->setScatterStyle(scatterStyle);
			}
			m_positionsMarker->setData(xPos, yPos);
			if (showSurfaceSquares && m_positionsMarkerSquare) {
				m_positionsMarkerSquare->setData(squareX, squareY);
			}

			if (m_positionsMarkerInsideRoi && ui->customplot_brightfield->removePlottable(m_positionsMarkerInsideRoi)) {
				m_positionsMarkerInsideRoi = nullptr;
			}
			if (m_positionsMarkerOutsideRoi && ui->customplot_brightfield->removePlottable(m_positionsMarkerOutsideRoi)) {
				m_positionsMarkerOutsideRoi = nullptr;
			}
			if (m_positionsMarkerSquareInsideRoi && ui->customplot_brightfield->removePlottable(m_positionsMarkerSquareInsideRoi)) {
				m_positionsMarkerSquareInsideRoi = nullptr;
			}
			if (m_positionsMarkerSquareOutsideRoi && ui->customplot_brightfield->removePlottable(m_positionsMarkerSquareOutsideRoi)) {
				m_positionsMarkerSquareOutsideRoi = nullptr;
			}
			if (!showSurfaceSquares && m_positionsMarkerSquare && ui->customplot_brightfield->removePlottable(m_positionsMarkerSquare)) {
				m_positionsMarkerSquare = nullptr;
			}
		}
		ui->customplot_brightfield->replot();
	} else if (m_positionsMarker || m_positionsMarkerSquare || m_positionsMarkerInsideRoi || m_positionsMarkerOutsideRoi || m_positionsMarkerSquareInsideRoi || m_positionsMarkerSquareOutsideRoi) {
		if (ui->customplot_brightfield->removePlottable(m_positionsMarker)) {
			m_positionsMarker = nullptr;
		}
		if (ui->customplot_brightfield->removePlottable(m_positionsMarkerSquare)) {
			m_positionsMarkerSquare = nullptr;
		}
		if (ui->customplot_brightfield->removePlottable(m_positionsMarkerInsideRoi)) {
			m_positionsMarkerInsideRoi = nullptr;
		}
		if (ui->customplot_brightfield->removePlottable(m_positionsMarkerOutsideRoi)) {
			m_positionsMarkerOutsideRoi = nullptr;
		}
		if (ui->customplot_brightfield->removePlottable(m_positionsMarkerSquareInsideRoi)) {
			m_positionsMarkerSquareInsideRoi = nullptr;
		}
		if (ui->customplot_brightfield->removePlottable(m_positionsMarkerSquareOutsideRoi)) {
			m_positionsMarkerSquareOutsideRoi = nullptr;
		}
		ui->customplot_brightfield->replot();
	}
	updateRoiPolygonPreview();
	updateOverviewTileOutlines();
}

/*
 * Show the outline (dashed yellow) of the area the brightfield overview mosaic will
 * cover, in the live view, while "Full grid (mosaic)" is active - one outline per
 * disjoint group of active points, not one rectangle per individual tile.
 */
void BrillouinAcquisition::updateOverviewTileOutlines() {
	const bool overviewActive = m_showPositions && m_scanControl && m_Brillouin->settings.saveOverviewBrightfieldPerZ;
	const bool showTiles = overviewActive && m_Brillouin->settings.overviewBrightfieldFullGrid;
	// Shows whenever the overview image isn't the mosaic.
	const bool showPoints = overviewActive && !m_Brillouin->settings.overviewBrightfieldFullGrid;
	const bool gridAbsolute = m_Brillouin->settings.gridCoordinatesAbsolute;
	// See the comment below on offset/frame conventions - both the mosaic outlines and the
	// point markers below live in the same frame and need the same conversion.
	const auto offset = currentGridOffset(gridAbsolute);

	if (!showTiles) {
		if (!m_overviewTileRects.empty()) {
			for (auto* rect : m_overviewTileRects) {
				ui->customplot_brightfield->removeItem(rect);
			}
			m_overviewTileRects.clear();
			ui->customplot_brightfield->replot();
		}
	} else {
		const auto outlines = m_Brillouin->overviewTileOutlinesUm();

		while (m_overviewTileRects.size() > outlines.size()) {
			ui->customplot_brightfield->removeItem(m_overviewTileRects.back());
			m_overviewTileRects.pop_back();
		}
		while (m_overviewTileRects.size() < outlines.size()) {
			auto* rect = new QCPItemRect(ui->customplot_brightfield);
			QPen pen(Qt::yellow);
			pen.setStyle(Qt::DashLine);
			pen.setWidth(2);
			rect->setPen(pen);
			rect->setBrush(Qt::NoBrush);
			m_overviewTileRects.push_back(rect);
		}

		// corner.first/second come from overviewTileOutlinesUm(), which - like m_orderedPositions/
		// m_orderedPositionsRelative it's built from - is already origin-inclusive in absolute
		// mode and origin-excluded (pure offset) in relative mode; NOT the pre-origin frame
		// roiPolygonUm uses. So this must mirror ScanControl::convertPositionsToPix()'s own
		// "point + offset" formula directly rather than going through gridOffsetToImagePlaneUm()
		// (which adds absoluteGridOriginUm again - correct for roiPolygonUm, a double-count here).
		// The offset itself still comes from the cached snapshot (see m_currentGridOffsetUm)
		// rather than a live getPositionOffset() call, for the same cross-thread-race reason the
		// ROI polygon overlay was fixed for.
		for (size_t i = 0; i < outlines.size(); i++) {
			const auto& corner = outlines[i];
			const auto topLeft = brightfieldRawToDisplay(m_scanControl->microMeterToPix(POINT2{ corner.first.x + offset.x, corner.first.y + offset.y }));
			const auto bottomRight = brightfieldRawToDisplay(m_scanControl->microMeterToPix(POINT2{ corner.second.x + offset.x, corner.second.y + offset.y }));
			m_overviewTileRects[i]->topLeft->setCoords(topLeft.x, topLeft.y);
			m_overviewTileRects[i]->bottomRight->setCoords(bottomRight.x, bottomRight.y);
		}
	}

	// Single-image center marker: shows exactly where the BF overview will be captured, in
	// the same frame/offset convention as the mosaic outlines above.
	if (!showPoints) {
		if (m_overviewPointMarker && ui->customplot_brightfield->removePlottable(m_overviewPointMarker)) {
			m_overviewPointMarker = nullptr;
		}
	} else {
		std::vector<POINT2> points{ m_Brillouin->overviewGridCenterXY() };

		if (!m_overviewPointMarker) {
			m_overviewPointMarker = new QCPCurve(ui->customplot_brightfield->xAxis, ui->customplot_brightfield->yAxis);
			m_overviewPointMarker->setLineStyle(QCPCurve::lsNone);
			QPen pen;
			pen.setColor(Qt::cyan);
			pen.setWidth(2);
			QCPScatterStyle scatterStyle;
			scatterStyle.setShape(QCPScatterStyle::ssDisc);
			scatterStyle.setPen(pen);
			scatterStyle.setSize(8);
			m_overviewPointMarker->setScatterStyle(scatterStyle);
		}
		QVector<double> xPix(static_cast<int>(points.size()));
		QVector<double> yPix(static_cast<int>(points.size()));
		for (size_t i = 0; i < points.size(); i++) {
			const auto pix = brightfieldRawToDisplay(m_scanControl->microMeterToPix(POINT2{ points[i].x + offset.x, points[i].y + offset.y }));
			xPix[static_cast<int>(i)] = pix.x;
			yPix[static_cast<int>(i)] = pix.y;
		}
		m_overviewPointMarker->setData(xPix, yPix);
	}

	ui->customplot_brightfield->replot();
}

BrillouinAcquisition::RoiTarget BrillouinAcquisition::mainRoiTarget() {
	return RoiTarget{
		&m_Brillouin->settings.roiPolygonUm,
		&m_Brillouin->settings.useRoiMask,
		&m_roiMaskAutoDisabled,
		m_useRoiMaskCheckbox,
		m_editRoiCheckbox,
		m_clearRoiButton,
		&m_lastClearedRoiPolygonUm,
		&m_lastClearedRoiUseMask,
		&m_roiPolygonMarker,
		&m_draggingRoiVertex,
		&m_draggedRoiVertexIndex,
		QColor(255, 165, 0),
		"Clear ROI",
		"Reset ROI",
		"ROI"
	};
}

BrillouinAcquisition::RoiTarget BrillouinAcquisition::backgroundRoiTarget() {
	return RoiTarget{
		&m_Brillouin->settings.backgroundRoiPolygonUm,
		&m_Brillouin->settings.useBackgroundRoiMask,
		&m_backgroundRoiMaskAutoDisabled,
		m_useBackgroundRoiMaskCheckbox,
		m_editBackgroundRoiCheckbox,
		m_clearBackgroundRoiButton,
		&m_lastClearedBackgroundRoiPolygonUm,
		&m_lastClearedBackgroundRoiUseMask,
		&m_backgroundRoiPolygonMarker,
		&m_draggingBackgroundRoiVertex,
		&m_draggedBackgroundRoiVertexIndex,
		// Deliberately distinct from the main ROI's orange, so both can be shown at once
		// without the overlays being mistaken for each other.
		QColor(30, 144, 255),
		"Clear bg. ROI",
		"Reset bg. ROI",
		"background ROI"
	};
}

// Shared by mainRoiTarget()/backgroundRoiTarget() - see RoiTarget's own comment for why this
// is written once instead of once per polygon.
void BrillouinAcquisition::updateRoiPolygonPreviewFor(const RoiTarget& target) {
	if (!m_scanControl) {
		return;
	}

	const auto& polygon = *target.polygon;
	const bool drawActive = (target.editCheckbox != nullptr && target.editCheckbox->isChecked());
	const bool show = drawActive || *target.useMask;
	auto removeMarker = [&]() {
		if (*target.marker && ui->customplot_brightfield->removePlottable(*target.marker)) {
			*target.marker = nullptr;
			ui->customplot_brightfield->replot();
		}
	};
	if (!show || polygon.empty()) {
		removeMarker();
		return;
	}

	const bool selfIntersecting = isSelfIntersectingPolygon(polygon);
	if (!*target.marker) {
		*target.marker = new QCPCurve(ui->customplot_brightfield->xAxis, ui->customplot_brightfield->yAxis);
		(*target.marker)->setLineStyle(QCPCurve::lsLine);
		(*target.marker)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssDisc, 6));
	}
	if (polygon.size() >= 3) {
		auto fill = target.color;
		fill.setAlpha(45);
		(*target.marker)->setBrush(QBrush(fill));
	} else {
		(*target.marker)->setBrush(Qt::NoBrush);
	}
	if (selfIntersecting) {
		(*target.marker)->setPen(QPen(QColor(220, 20, 60), 2));
		statusBar()->showMessage(
			QString("%1 invalid: self-intersection detected. Adjust points in Draw mode.").arg(target.label), 4000);
	} else {
		(*target.marker)->setPen(QPen(target.color, 2));
	}

	std::vector<POINT2> polygonPix;
	polygonPix.reserve(polygon.size() + 1);
	for (const auto& p : polygon) {
		auto pUm = gridOffsetToImagePlaneUm(p);
		polygonPix.push_back(brightfieldRawToDisplay(m_scanControl->microMeterToPix(pUm)));
	}
	if (polygon.size() >= 3) {
		auto pUm = gridOffsetToImagePlaneUm(polygon[0]);
		polygonPix.push_back(brightfieldRawToDisplay(m_scanControl->microMeterToPix(pUm)));
	}
	QVector<double> xPos(static_cast<int>(polygonPix.size()));
	QVector<double> yPos(static_cast<int>(polygonPix.size()));
	for (gsl::index i{ 0 }; i < (gsl::index)polygonPix.size(); i++) {
		xPos[(int)i] = polygonPix[i].x;
		yPos[(int)i] = polygonPix[i].y;
	}
	(*target.marker)->setData(xPos, yPos);
	ui->customplot_brightfield->replot();
}

void BrillouinAcquisition::updateRoiPolygonPreview() {
	updateRoiPolygonPreviewFor(mainRoiTarget());
	updateRoiPolygonPreviewFor(backgroundRoiTarget());
}

// "Clear"/"Reset" toggle-button behavior: clears (backing up first) if there's a polygon,
// otherwise restores the last backup - shared between the main and background clear buttons.
void BrillouinAcquisition::clearRoiPolygonFor(const RoiTarget& target) {
	if (!target.polygon->empty()) {
		*target.lastCleared = *target.polygon;
		*target.lastClearedUseMask = *target.useMask;
		target.polygon->clear();
		*target.useMask = false;
		if (target.useMaskCheckbox) {
			const QSignalBlocker blocker(target.useMaskCheckbox);
			target.useMaskCheckbox->setChecked(false);
		}
	} else if (!target.lastCleared->empty()) {
		*target.polygon = *target.lastCleared;
		*target.useMask = *target.lastClearedUseMask;
		if (target.useMaskCheckbox) {
			const QSignalBlocker blocker(target.useMaskCheckbox);
			target.useMaskCheckbox->setChecked(*target.lastClearedUseMask);
		}
	}
	if (target.clearButton) {
		target.clearButton->setText(
			target.polygon->empty() && !target.lastCleared->empty() ? target.resetLabel : target.clearLabel);
	}
}

// Right-click-to-clear in the plot: always just clears (backing up first), never restores -
// a quicker, simpler shortcut than the "Clear"/"Reset" toggle button above.
void BrillouinAcquisition::quickClearRoiPolygonFor(const RoiTarget& target) {
	if (!target.polygon->empty()) {
		*target.lastCleared = *target.polygon;
		*target.lastClearedUseMask = *target.useMask;
	}
	target.polygon->clear();
	*target.useMask = false;
	if (target.clearButton) {
		target.clearButton->setText(!target.lastCleared->empty() ? target.resetLabel : target.clearLabel);
	}
}

void BrillouinAcquisition::addRoiPolygonPointFor(const RoiTarget& target, POINT2 positionInUm) {
	target.polygon->push_back(positionInUm);
	if (target.polygon->size() >= 3) {
		*target.useMask = true;
	}
	if (target.clearButton) {
		target.clearButton->setText(target.clearLabel);
	}
}

bool BrillouinAcquisition::tryEnableRoiMaskFor(const RoiTarget& target) {
	if (target.polygon->size() < 3) {
		QMessageBox::warning(
			this,
			QString("%1 Mask Needs Polygon").arg(target.label),
			QString("Enable Draw %1 and add at least 3 points in the brightfield plot.").arg(target.label)
		);
		return false;
	}
	if (isSelfIntersectingPolygon(*target.polygon)) {
		QMessageBox::warning(
			this,
			QString("Invalid %1 Polygon").arg(target.label),
			QString("%1 polygon edges intersect each other.\nPlease adjust points so the polygon is non-self-intersecting.").arg(target.label)
		);
		return false;
	}
	return true;
}

void BrillouinAcquisition::updateRoiMaskCheckboxStateFor(const RoiTarget& target) {
	if (!target.useMaskCheckbox) {
		return;
	}
	const bool selfIntersecting = isSelfIntersectingPolygon(*target.polygon);
	const bool maskPossible = target.polygon->size() >= 3 && !selfIntersecting;
	target.useMaskCheckbox->setEnabled(maskPossible);
	if (!maskPossible && *target.useMask) {
		*target.useMask = false;
		*target.autoDisabled = true;
	} else if (maskPossible && *target.autoDisabled && !*target.useMask) {
		// See m_roiMaskAutoDisabled's own comment - undo an auto-disable, not a genuine
		// user choice, once the polygon is valid again.
		*target.useMask = true;
	}
	if (maskPossible) {
		*target.autoDisabled = false;
	}
	const QSignalBlocker blocker(target.useMaskCheckbox);
	target.useMaskCheckbox->setChecked(*target.useMask);
	if (selfIntersecting) {
		target.useMaskCheckbox->setToolTip(
			QString("%1 invalid: polygon edges intersect. Adjust points in Draw mode.").arg(target.label));
	} else if (target.polygon->size() < 3) {
		target.useMaskCheckbox->setToolTip(QString("%1 needs at least 3 points.").arg(target.label));
	} else {
		target.useMaskCheckbox->setToolTip("");
	}
}

void BrillouinAcquisition::updateDraggedRoiVertex(const RoiTarget& target, QMouseEvent* event) {
	event->accept();
	const auto posX = m_ODTPlot.plotHandle->xAxis->pixelToCoord(event->pos().x());
	const auto posY = m_ODTPlot.plotHandle->yAxis->pixelToCoord(event->pos().y());
	auto positionInUm = m_scanControl->pixToMicroMeter(brightfieldDisplayToRaw(POINT2{ posX, posY }));
	positionInUm = imagePlaneUmToGridOffset(positionInUm);
	auto& poly = *target.polygon;
	const int idx = *target.draggedVertexIndex;
	if (idx >= 0 && idx < (int)poly.size()) {
		poly[(size_t)idx] = positionInUm;
		updateRoiPolygonPreviewFor(target);
	}
}

void BrillouinAcquisition::on_preCalibration_stateChanged(int state) {
	m_Brillouin->settings.preCalibration = (bool)state;
}

void BrillouinAcquisition::on_postCalibration_stateChanged(int state) {
	m_Brillouin->settings.postCalibration = (bool)state;
}

void BrillouinAcquisition::on_conCalibration_stateChanged(int state) {
	m_Brillouin->settings.conCalibration = (bool)state;
}


void BrillouinAcquisition::on_sampleSelection_currentIndexChanged(const QString &text) {
	m_Brillouin->settings.sample = text.toStdString();
}

void BrillouinAcquisition::on_conCalibrationInterval_valueChanged(double value) {
	m_Brillouin->settings.conCalibrationInterval = value;
}

void BrillouinAcquisition::on_nrCalibrationImages_valueChanged(int value) {
	m_Brillouin->settings.nrCalibrationImages = value;
}

void BrillouinAcquisition::on_calibrationExposureTime_valueChanged(double value) {
	m_Brillouin->settings.calibrationExposureTime = value;
}

/*
 * Functions regarding the repetition feature.
 */

void BrillouinAcquisition::on_repetitionCount_valueChanged(int count) {
	m_Brillouin->settings.repetitions.count = count;
}

void BrillouinAcquisition::on_repetitionInterval_valueChanged(double interval) {
	m_Brillouin->settings.repetitions.interval = interval;
}

void BrillouinAcquisition::on_repetitionNewFile_stateChanged(int checked) {
	m_Brillouin->settings.repetitions.filePerRepetition = (bool)checked;
}

void BrillouinAcquisition::showRepProgress(int repNumber, int timeToNext) {
	ui->repetitionProgress->setValue(100 * ((double)repNumber + 1) / m_Brillouin->settings.repetitions.count);

	QString string;
	if (timeToNext > 0) {
		string = formatSeconds(timeToNext) + " to next repetition.";
	} else if (timeToNext > -2) {
		if (repNumber < m_Brillouin->settings.repetitions.count) {
			string.sprintf("Measuring repetition %1.0d of %1.0d.", repNumber + 1, m_Brillouin->settings.repetitions.count);
		} else {
			string.sprintf("Finished %1.0d repetitions.", m_Brillouin->settings.repetitions.count);
		}
	} else {
		string.sprintf("Finished %1.0d repetitions.", repNumber);
	}
	ui->repetitionProgress->setFormat(string);
}

void BrillouinAcquisition::on_savePosition_clicked() {
	QMetaObject::invokeMethod(
		m_scanControl,
		[&m_scanControl = m_scanControl]() {
			m_scanControl->savePosition();
		},
		Qt::AutoConnection
	);
}

void BrillouinAcquisition::on_setHome_clicked() {
	// Same button, two roles - "Set home" (x/y/z) in relative mode, "Set plane" (z only) in
	// absolute mode, where Set home doesn't have a sensible x/y meaning any more (the absolute
	// origin is a fixed point, not something a button click should silently redefine) but z
	// still needs a way to re-anchor - see Brillouin::resolvedGridOriginUm()'s comment for how z
	// is anchored in each mode. Swapping roles on the one button (rather than a separate, always-
	// visible "Set plane" button) keeps the control count the same in both modes.
	if (!m_Brillouin) {
		return;
	}
	if (m_Brillouin->settings.gridCoordinatesAbsolute) {
		QMetaObject::invokeMethod(m_Brillouin, "setCurrentFocusAsZOrigin", Qt::AutoConnection);
		return;
	}
	QMetaObject::invokeMethod(
		m_scanControl,
		[&m_scanControl = m_scanControl]() {
			m_scanControl->setHome();
		},
		Qt::AutoConnection
	);
}

void BrillouinAcquisition::on_moveHome_clicked() {
	if (m_enabledModes == ACQUISITION_MODE::NONE && !m_Brillouin->settings.gridCoordinatesAbsolute) {
		QMetaObject::invokeMethod(
			m_scanControl,
			[&m_scanControl = m_scanControl]() {
				m_scanControl->moveHome();
			},
			Qt::AutoConnection
		);
	}
}

void BrillouinAcquisition::on_setPositionX_valueChanged(double positionX) {
	if (m_enabledModes == ACQUISITION_MODE::NONE) {
		QMetaObject::invokeMethod(
			m_scanControl,
			[&m_scanControl = m_scanControl, positionX]() {
				m_scanControl->setPositionRelativeX(positionX);
			},
			Qt::AutoConnection
		);
	}
}

void BrillouinAcquisition::on_setPositionY_valueChanged(double positionY) {
	if (m_enabledModes == ACQUISITION_MODE::NONE) {
		QMetaObject::invokeMethod(
			m_scanControl,
			[&m_scanControl = m_scanControl, positionY]() {
				m_scanControl->setPositionRelativeY(positionY);
			},
			Qt::AutoConnection
		);
	}
}

void BrillouinAcquisition::on_setPositionZ_valueChanged(double positionZ) {
	if (m_enabledModes == ACQUISITION_MODE::NONE) {
		QMetaObject::invokeMethod(
			m_scanControl,
			[&m_scanControl = m_scanControl, positionZ]() {
				m_scanControl->setPositionRelativeZ(positionZ);
			},
			Qt::AutoConnection
		);
	}
}

void BrillouinAcquisition::updateSavedPositions() {
	ui->tableView->setModel(tableModel);
	ui->tableView->setItemDelegateForColumn(3, &buttonDelegate);
	ui->tableView->verticalHeader()->setDefaultSectionSize(22);
	ui->tableView->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
	ui->tableView->setVerticalScrollMode(QAbstractItemView::ScrollPerPixel);
	ui->tableView->setHorizontalScrollMode(QAbstractItemView::ScrollPerPixel);
	ui->tableView->show();
}

void BrillouinAcquisition::on_scanDirAutoCheckbox_stateChanged(int automatical) {
	m_Brillouin->setScanOrderAuto((bool)automatical);
}

void BrillouinAcquisition::on_buttonGroup_buttonClicked(int button) {
	m_Brillouin->setScanOrderX(button);
}

void BrillouinAcquisition::on_buttonGroup_2_buttonClicked(int button) {
	m_Brillouin->setScanOrderY(button);
}

void BrillouinAcquisition::on_buttonGroup_3_buttonClicked(int button) {
	m_Brillouin->setScanOrderZ(button);
}

void BrillouinAcquisition::scanOrderChanged(SCAN_ORDER scanOrder) {
	if (scanOrder.x == 0) {
		ui->scanDirX0->setChecked(true);
	}
	if (scanOrder.x == 1) {
		ui->scanDirX1->setChecked(true);
	}
	if (scanOrder.x == 2) {
		ui->scanDirX2->setChecked(true);
	}
	if (scanOrder.y == 0) {
		ui->scanDirY0->setChecked(true);
	}
	if (scanOrder.y == 1) {
		ui->scanDirY1->setChecked(true);
	}
	if (scanOrder.y == 2) {
		ui->scanDirY2->setChecked(true);
	}
	if (scanOrder.z == 0) {
		ui->scanDirZ0->setChecked(true);
	}
	if (scanOrder.z == 1) {
		ui->scanDirZ1->setChecked(true);
	}
	if (scanOrder.z == 2) {
		ui->scanDirZ2->setChecked(true);
	}
	// disable radio buttons if order is determined automatically
	ui->scanDirX0->setDisabled(scanOrder.automatical);
	ui->scanDirX1->setDisabled(scanOrder.automatical);
	ui->scanDirX2->setDisabled(scanOrder.automatical);
	ui->scanDirY0->setDisabled(scanOrder.automatical);
	ui->scanDirY1->setDisabled(scanOrder.automatical);
	ui->scanDirY2->setDisabled(scanOrder.automatical);
	ui->scanDirZ0->setDisabled(scanOrder.automatical);
	ui->scanDirZ1->setDisabled(scanOrder.automatical);
	ui->scanDirZ2->setDisabled(scanOrder.automatical);
}

void BrillouinAcquisition::on_exposureTime_valueChanged(double value) {
	m_Brillouin->settings.camera.exposureTime = value;
	updateEstimatedAcquisitionTime();
}

void BrillouinAcquisition::on_frameCount_valueChanged(int value) {
	m_Brillouin->settings.camera.frameCount = value;
	updateEstimatedAcquisitionTime();
}

StoragePath BrillouinAcquisition::splitFilePath(QString fullPath) {
	QFileInfo fileInfo(fullPath);
	return {
		fileInfo.fileName().toStdString(),
		fileInfo.absolutePath().toStdString()
	};
}

QString BrillouinAcquisition::checkFilename(QString absoluteFilePath) {
	QFileInfo fileInfo(absoluteFilePath);
	// get filename without extension
	std::string rawFilename = fileInfo.baseName().toStdString();
	// remove possibly attached number separated by a hyphen
	rawFilename = rawFilename.substr(0, rawFilename.find_last_of("-"));
	int count = 0;
	std::string filename;
	std::string fullPath = absoluteFilePath.toStdString();
	while (exists(fullPath)) {
		filename = rawFilename + '-' + std::to_string(count) + '.' + fileInfo.completeSuffix().toStdString();
		fullPath = fileInfo.absolutePath().toStdString() + "/" + filename;
		count++;
	}
	return QString::fromStdString(fullPath);
}

void BrillouinAcquisition::on_actionNew_Acquisition_triggered() {

	StoragePath tmpStorage = m_storagePath;

	// A brand new acquisition gets a fresh, timestamped name by default, rather than
	// reusing whatever the last one this session was called - checkFilename() below would
	// otherwise just dedupe that against the existing file with a trailing "-0", "-1", ...,
	// which is far less informative than a timestamp and easy to mix up between runs.
	tmpStorage.filename = "Brillouin_"
		+ QDateTime::currentDateTime().toString("yyyy-MM-dd_HH-mm-ss").toStdString()
		+ ".h5";

	// Folder: whatever was last used (this session, or a previous one - m_storagePath.folder
	// is restored from settings at startup, see readSettings()), or the explicitly configured
	// default save folder (File > Set Default Save Folder...) if nothing has ever been saved,
	// or "." (the StoragePath default) if neither is set.
	if (tmpStorage.folder == "." && !m_defaultAcquisitionFolder.empty()) {
		tmpStorage.folder = m_defaultAcquisitionFolder;
	}

	QString proposedFileName = QString::fromStdString(tmpStorage.fullPath());

	proposedFileName = checkFilename(proposedFileName);

	QString fullPath = QFileDialog::getSaveFileName(this, tr("Save new Acquisition as"),
		proposedFileName, tr("Brillouin data (*.h5)"));

	if (fullPath.isEmpty()) {
		return;
	}

	m_storagePath = splitFilePath(fullPath);
	rememberAcquisitionFolder(m_storagePath.folder);

	QMetaObject::invokeMethod(
		m_acquisition,
		[&m_acquisition = m_acquisition, &m_storagePath = m_storagePath]() {
			m_acquisition->newFile(m_storagePath);
		},
		Qt::AutoConnection
	);
}

// Persists the folder a Brillouin file was just saved to or opened from, so the next
// New/Open Acquisition dialog (this session or after a restart, via m_storagePath.folder
// restored in readSettings()) proposes it automatically, with no separate "set default"
// step required. Written immediately, the same way on_actionSetDefaultAcquisitionFolder_
// triggered() persists its own folder - both are standalone File-menu actions, not
// settings-dialog fields, so there is no "Apply" click for either to wait for.
void BrillouinAcquisition::rememberAcquisitionFolder(const std::string& folder) {
	QSettings settings(QSettings::IniFormat, QSettings::UserScope, kSettingsOrg, kSettingsApp);
	settings.setValue("last-acquisition-folder", QString::fromStdString(folder));
}

void BrillouinAcquisition::on_actionSetDefaultAcquisitionFolder_triggered() {
	const auto startDir = m_defaultAcquisitionFolder.empty()
		? QString::fromStdString(m_storagePath.folder)
		: QString::fromStdString(m_defaultAcquisitionFolder);
	const auto folder = QFileDialog::getExistingDirectory(
		this, tr("Set Default Save Folder"), startDir);
	if (folder.isEmpty()) {
		return;
	}
	m_defaultAcquisitionFolder = folder.toStdString();
	// Persisted immediately (not batched into saveSettings()'s "Apply"-triggered write) -
	// this is a standalone File-menu action, not a settings-dialog field, so there is no
	// "Apply" click for it to wait for.
	QSettings settings(QSettings::IniFormat, QSettings::UserScope, kSettingsOrg, kSettingsApp);
	settings.setValue("default-acquisition-folder", QString::fromStdString(m_defaultAcquisitionFolder));
}

void BrillouinAcquisition::on_actionOpen_Acquisition_triggered() {
	QString fullPath = QFileDialog::getOpenFileName(this, tr("Save new Acquisition as"),
		QString::fromStdString(m_storagePath.folder), tr("Brillouin data (*.h5)"));

	if (fullPath.isEmpty()) {
		return;
	}

	m_storagePath = splitFilePath(fullPath);
	rememberAcquisitionFolder(m_storagePath.folder);

	QMetaObject::invokeMethod(
		m_acquisition,
		[&m_acquisition = m_acquisition, &m_storagePath = m_storagePath]() {
			m_acquisition->openFile(m_storagePath);
		},
		Qt::AutoConnection
	);
}

void BrillouinAcquisition::on_actionClose_Acquisition_triggered() {
	int ret = m_acquisition->closeFile();
	if (ret == 0) {
		m_storagePath.filename = "";
		ui->acquisitionFilename->setText(QString::fromStdString(m_storagePath.filename));
	}
}

void BrillouinAcquisition::setColormap(QCPColorGradient *gradient, const CustomGradientPreset& preset) {
	gradient->clearColorStops();
	switch (preset) {
		case CustomGradientPreset::gpViridis: {
			gradient->setColorInterpolation(QCPColorGradient::ciRGB);
			BrillouinAcquisition::applyColorMap(gradient, ColorMaps::m_viridis);
			break;
		}
		case CustomGradientPreset::gpGrayscale:
			gradient->loadPreset(QCPColorGradient::gpGrayscale);
			break;
		case CustomGradientPreset::gpInferno:
			gradient->setColorInterpolation(QCPColorGradient::ciRGB);
			BrillouinAcquisition::applyColorMap(gradient, ColorMaps::m_inferno);
			break;
		default:
			gradient->loadPreset(QCPColorGradient::gpGrayscale);
			break;
	}
}

void BrillouinAcquisition::applyColorMap(QCPColorGradient* gradient, const std::vector<std::vector<double>>& colorMap) {
	auto index{ 0 };
	auto position{ 0.0 };
	for (auto it = std::begin(colorMap); it != std::end(colorMap); ++it) {
		gradient->setColorStopAt((double)index / colorMap.size(), QColor((int)255 * (*it)[0], (int)255 * (*it)[1], (int)255 * (*it)[2]));
		++index;
	}
}

void BrillouinAcquisition::writeSettings() {
	QSettings settings(QSettings::IniFormat, QSettings::UserScope,
		kSettingsOrg, kSettingsApp);

	settings.setValue("default-acquisition-folder", QString::fromStdString(m_defaultAcquisitionFolder));
	settings.setValue("last-acquisition-folder", QString::fromStdString(m_storagePath.folder));

	auto brillouinCamera = QString{};
	switch (m_cameraBrillouinType) {
		case CAMERA_BRILLOUIN_DEVICE::ANDOR:
			brillouinCamera = "andor";
			break;
		case CAMERA_BRILLOUIN_DEVICE::PVCAM:
			brillouinCamera = "pvcam";
			break;
#ifdef _DEBUG
		case CAMERA_BRILLOUIN_DEVICE::MOCK:
			brillouinCamera = "mock";
			break;
#endif
		default:
			brillouinCamera = "andor";
			break;
	}
	auto brillouinCameraNumber = QString::number(m_cameraBrillouinNumber);

	auto brightfieldCamera = QString{};
	switch (m_cameraType) {
		case CAMERA_DEVICE::UEYE:
			brightfieldCamera = "ueye";
			break;
		case CAMERA_DEVICE::POINTGREY:
			brightfieldCamera = "pointgrey";
			break;
#ifdef _DEBUG
		case CAMERA_DEVICE::MOCK:
			brightfieldCamera = "mock";
			break;
#endif
		default:
			brightfieldCamera = "none";
			break;
	}

	auto stage = QString{};
	switch (m_scanControllerType) {
		case ScanControl::SCAN_DEVICE::ZEISSECU:
			stage = "zeiss-ecu";
			break;
		case ScanControl::SCAN_DEVICE::NIDAQ:
			stage = "nidaq";
			break;
		case ScanControl::SCAN_DEVICE::ZEISSMTB:
			stage = "zeiss-mtb";
			break;
		case ScanControl::SCAN_DEVICE::ZEISSMTBERLANGEN:
			stage = "zeiss-mtb-erlangen";
			break;
		case ScanControl::SCAN_DEVICE::ZEISSMTBERLANGEN2:
			stage = "zeiss-mtb-erlangen-2";
			break;
		default:
			stage = "zeiss-ecu";
			break;
	}

	settings.beginGroup("devices");
	settings.setValue("brillouin-camera", brillouinCamera);
	settings.setValue("brillouin-camera-number", brillouinCameraNumber);
	settings.setValue("brightfield-camera", brightfieldCamera);
	settings.setValue("stage", stage);
	settings.endGroup();
	settings.beginGroup("objective-setup");
	settings.setValue("slot-count", (int)m_objectiveSlotNames.size());
	for (size_t ii = 0; ii < m_objectiveSlotNames.size(); ii++) {
		settings.setValue(QString("slot-%1-name").arg(ii + 1), QString::fromStdString(m_objectiveSlotNames[ii]));
		auto path = ii < m_objectiveSlotCalibrationPaths.size() ? m_objectiveSlotCalibrationPaths[ii] : "";
		settings.setValue(QString("slot-%1-calibration-path").arg(ii + 1), QString::fromStdString(path));
	}
	settings.endGroup();
	settings.beginGroup("devices-settings");
	settings.setValue("stage-laser-position-x", m_positionScanner.x);
	settings.setValue("stage-laser-position-y", m_positionScanner.y);
	settings.setValue("stage-laser-position-objective-slot", m_positionScannerObjectiveSlot);
	settings.setValue("brightfield-view-rotation-degrees", (int)m_brightfieldViewRotation * 90);
	settings.setValue("brightfield-view-mirror-horizontal", m_brightfieldMirrorHorizontal);
	settings.setValue("brightfield-view-mirror-vertical", m_brightfieldMirrorVertical);
	settings.setValue("stage-x-min", m_Brillouin->settings.xMin);
	settings.setValue("stage-x-max", m_Brillouin->settings.xMax);
	settings.setValue("stage-x-steps", m_Brillouin->settings.xSteps);
	settings.setValue("stage-y-min", m_Brillouin->settings.yMin);
	settings.setValue("stage-y-max", m_Brillouin->settings.yMax);
	settings.setValue("stage-y-steps", m_Brillouin->settings.ySteps);
	settings.setValue("stage-z-min", m_Brillouin->settings.zMin);
	settings.setValue("stage-z-max", m_Brillouin->settings.zMax);
	settings.setValue("stage-z-steps", m_Brillouin->settings.zSteps);
	settings.setValue("brillouin-pre-calibrate", m_Brillouin->settings.preCalibration);
	settings.setValue("brillouin-post-calibrate", m_Brillouin->settings.postCalibration);
	settings.setValue("brillouin-con-calibrate", m_Brillouin->settings.conCalibration);
	settings.setValue("brillouin-con-calibrate-interval", m_Brillouin->settings.conCalibrationInterval);
	settings.setValue("brillouin-nr-calibration-images", m_Brillouin->settings.nrCalibrationImages);
	settings.setValue("brillouin-calibration-exposure-time", m_Brillouin->settings.calibrationExposureTime);
	settings.setValue("brillouin-use-roi-mask", m_Brillouin->settings.useRoiMask);
	settings.setValue("brillouin-roi-polygon-um", serializeRoiPolygon(m_Brillouin->settings.roiPolygonUm));
	settings.setValue("brillouin-use-background-roi-mask", m_Brillouin->settings.useBackgroundRoiMask);
	settings.setValue("brillouin-background-roi-polygon-um", serializeRoiPolygon(m_Brillouin->settings.backgroundRoiPolygonUm));
	// useSurfaceFollow is deliberately not persisted - it should always start off,
	// regardless of how the previous session ended.
	settings.setValue("brillouin-surface-z-offset-um", m_Brillouin->settings.surfaceZOffsetUm);
	settings.setValue("brillouin-surface-follow-half-range-um", m_Brillouin->settings.surfaceFollowHalfRangeUm);
	settings.setValue("brillouin-pre-scan-xy-bin", m_Brillouin->settings.preScanXYBin);
	settings.setValue("brillouin-additional-boundary-points", m_Brillouin->settings.additionalBoundaryPoints);
	settings.setValue("brillouin-pre-scan-z-step-um", m_Brillouin->settings.preScanZStepUm);
	settings.setValue("brillouin-pre-scan-z-travel-um", m_Brillouin->settings.preScanZTravelRangeUm);
	settings.setValue("brillouin-pre-scan-x-steps", m_Brillouin->settings.preScanXSteps);
	settings.setValue("brillouin-pre-scan-y-steps", m_Brillouin->settings.preScanYSteps);
	settings.setValue("brillouin-pre-scan-z-steps", m_Brillouin->settings.preScanZSteps);
	settings.setValue("brillouin-pre-scan-z-min", m_Brillouin->settings.preScanZMin);
	settings.setValue("brillouin-pre-scan-z-max", m_Brillouin->settings.preScanZMax);
	settings.setValue("brillouin-surface-metric-threshold", m_Brillouin->settings.surfaceMetricThreshold);
	settings.setValue("brillouin-surface-smooth-sigma-um", m_Brillouin->settings.surfaceSmoothSigmaUm);
	settings.setValue("brillouin-surface-drop-fraction", m_Brillouin->settings.surfaceDropFraction);
	settings.setValue("brillouin-medium-reference-value", m_Brillouin->settings.mediumReferenceValue);
	settings.setValue("brillouin-medium-reference-frame-count", m_Brillouin->settings.mediumReferenceFrameCount);
	settings.setValue("brillouin-surface-max-rewind-um", m_Brillouin->settings.surfaceMaxRewindUm);
	settings.setValue("brillouin-surface-verification-steps", m_Brillouin->settings.surfaceVerificationSteps);
	settings.setValue("brillouin-surface-verification-frame-average", m_Brillouin->settings.surfaceVerificationFrameAverage);
	settings.setValue("brillouin-surface-verification-tolerance-fraction", m_Brillouin->settings.surfaceVerificationToleranceFraction);
	// gridCoordinatesAbsolute is deliberately not persisted - it should always start off,
	// regardless of how the previous session ended. The origin itself is still saved/
	// restored below, in case the user re-enables absolute mode.
	settings.setValue("brillouin-absolute-grid-origin-x-um", m_Brillouin->settings.absoluteGridOriginUm.x);
	settings.setValue("brillouin-absolute-grid-origin-y-um", m_Brillouin->settings.absoluteGridOriginUm.y);
	settings.setValue("brillouin-absolute-grid-origin-z-um", m_Brillouin->settings.absoluteGridOriginUm.z);
	settings.setValue("brillouin-use-grid-hysteresis-compensation", m_Brillouin->settings.useGridHysteresisCompensation);
	settings.setValue("brillouin-use-dose-protection", m_Brillouin->settings.useDoseProtection);
	settings.setValue("brillouin-save-overview-brightfield-per-z", m_Brillouin->settings.saveOverviewBrightfieldPerZ);
	settings.setValue("brillouin-overview-brightfield-exposure-ms", m_Brillouin->settings.overviewBrightfieldExposureMs);
	settings.setValue("brillouin-overview-brightfield-gain", m_Brillouin->settings.overviewBrightfieldGain);
	settings.setValue("brillouin-overview-brightfield-full-grid", m_Brillouin->settings.overviewBrightfieldFullGrid);
	settings.setValue("brillouin-overview-brightfield-full-stack-single", m_Brillouin->settings.overviewBrightfieldFullStackSingle);
	settings.setValue("brillouin-overview-brightfield-full-stack-mosaic", m_Brillouin->settings.overviewBrightfieldFullStackMosaic);
	settings.setValue("brillouin-capture-per-point-brightfield", m_Brillouin->settings.capturePerPointBrightfield);
	settings.setValue("brillouin-per-point-brightfield-every-n", m_Brillouin->settings.perPointBrightfieldEveryN);
	settings.setValue("brillouin-per-point-brightfield-during-acquisition", m_Brillouin->settings.perPointBrightfieldDuringAcquisition);
	settings.setValue("brillouin-surface-proxy-roi-left", m_Brillouin->settings.surfaceProxyRoiLeft);
	settings.setValue("brillouin-surface-proxy-roi-top", m_Brillouin->settings.surfaceProxyRoiTop);
	settings.setValue("brillouin-surface-proxy-roi-width", m_Brillouin->settings.surfaceProxyRoiWidth);
	settings.setValue("brillouin-surface-proxy-roi-height", m_Brillouin->settings.surfaceProxyRoiHeight);
	settings.setValue("brillouin-surface-proxy-roi-2-left", m_Brillouin->settings.surfaceProxyRoi2Left);
	settings.setValue("brillouin-surface-proxy-roi-2-top", m_Brillouin->settings.surfaceProxyRoi2Top);
	settings.setValue("brillouin-surface-proxy-roi-2-width", m_Brillouin->settings.surfaceProxyRoi2Width);
	settings.setValue("brillouin-surface-proxy-roi-2-height", m_Brillouin->settings.surfaceProxyRoi2Height);
	settings.setValue("brillouin-surface-proxy-roi-frame-width", m_Brillouin->settings.surfaceProxyRoiFrameWidth);
	settings.setValue("brillouin-surface-proxy-roi-frame-height", m_Brillouin->settings.surfaceProxyRoiFrameHeight);
	settings.setValue("brillouin-surface-proxy-roi-frame-origin-left", (qlonglong)m_Brillouin->settings.surfaceProxyRoiFrameOriginLeft);
	settings.setValue("brillouin-surface-proxy-roi-frame-origin-bottom", (qlonglong)m_Brillouin->settings.surfaceProxyRoiFrameOriginBottom);
	settings.setValue("brillouin-surface-proxy-roi-frame-width-physical", (qlonglong)m_Brillouin->settings.surfaceProxyRoiFrameWidthPhysical);
	settings.setValue("brillouin-surface-proxy-roi-frame-height-physical", (qlonglong)m_Brillouin->settings.surfaceProxyRoiFrameHeightPhysical);
	settings.setValue("brillouin-surface-proxy-roi-2-frame-width", m_Brillouin->settings.surfaceProxyRoi2FrameWidth);
	settings.setValue("brillouin-surface-proxy-roi-2-frame-height", m_Brillouin->settings.surfaceProxyRoi2FrameHeight);
	settings.setValue("brillouin-surface-proxy-roi-2-frame-origin-left", (qlonglong)m_Brillouin->settings.surfaceProxyRoi2FrameOriginLeft);
	settings.setValue("brillouin-surface-proxy-roi-2-frame-origin-bottom", (qlonglong)m_Brillouin->settings.surfaceProxyRoi2FrameOriginBottom);
	settings.setValue("brillouin-surface-proxy-roi-2-frame-width-physical", (qlonglong)m_Brillouin->settings.surfaceProxyRoi2FrameWidthPhysical);
	settings.setValue("brillouin-surface-proxy-roi-2-frame-height-physical", (qlonglong)m_Brillouin->settings.surfaceProxyRoi2FrameHeightPhysical);
	settings.setValue("brillouin-camera-roi-left", m_deviceSettings.camera.roi.left);
	settings.setValue("brillouin-camera-roi-top", m_deviceSettings.camera.roi.top);
	settings.setValue("brillouin-camera-roi-width-physical", m_deviceSettings.camera.roi.width_physical);
	settings.setValue("brillouin-camera-roi-height-physical", m_deviceSettings.camera.roi.height_physical);
	settings.setValue("brillouin-camera-exposure-time", m_Brillouin->settings.camera.exposureTime);
	settings.setValue("brillouin-camera-frame-count", m_Brillouin->settings.camera.frameCount);
	settings.endGroup();
}

void BrillouinAcquisition::readSettings() {
	QSettings settings(QSettings::IniFormat, QSettings::UserScope,
		kSettingsOrg, kSettingsApp);

	m_defaultAcquisitionFolder = settings.value("default-acquisition-folder", "").toString().toStdString();
	m_storagePath.folder = settings.value("last-acquisition-folder", m_storagePath.folder).toString().toStdString();

	settings.beginGroup("devices");
	QVariant BrillouinCam = settings.value("brillouin-camera");
	QVariant BrillouinCamNumber = settings.value("brillouin-camera-number");
	QVariant BrightfieldCam = settings.value("brightfield-camera");
	QVariant stage = settings.value("stage");

	// Brillouin camera
	if (BrillouinCam == "andor") {
		m_cameraBrillouinType = CAMERA_BRILLOUIN_DEVICE::ANDOR;
	} else if (BrillouinCam == "pvcam") {
		m_cameraBrillouinType = CAMERA_BRILLOUIN_DEVICE::PVCAM;
	}
#ifdef _DEBUG
	else if (BrillouinCam == "mock") {
		m_cameraBrillouinType = CAMERA_BRILLOUIN_DEVICE::MOCK;
	}
#endif
	else {
		m_cameraBrillouinType = CAMERA_BRILLOUIN_DEVICE::ANDOR;
	}

	m_cameraBrillouinNumber = BrillouinCamNumber.toInt();

	// Brightfield camera
	if (BrightfieldCam == "ueye") {
		m_cameraType = CAMERA_DEVICE::UEYE;
	} else if (BrightfieldCam == "pointgrey") {
		m_cameraType = CAMERA_DEVICE::POINTGREY;
	}
#ifdef _DEBUG
	else if (BrightfieldCam == "mock") {
		m_cameraType = CAMERA_DEVICE::MOCK;
	}
#endif
	else {
		m_cameraType = CAMERA_DEVICE::NONE;
	}

	// Scanning stage
	if (stage == "zeiss-ecu") {
		m_scanControllerType = ScanControl::SCAN_DEVICE::ZEISSECU;
	} else if (stage == "nidaq") {
		m_scanControllerType = ScanControl::SCAN_DEVICE::NIDAQ;
	} else if (stage == "zeiss-mtb") {
		m_scanControllerType = ScanControl::SCAN_DEVICE::ZEISSMTB;
	} else if (stage == "zeiss-mtb-erlangen") {
		m_scanControllerType = ScanControl::SCAN_DEVICE::ZEISSMTBERLANGEN;
	} else if (stage == "zeiss-mtb-erlangen-2") {
		m_scanControllerType = ScanControl::SCAN_DEVICE::ZEISSMTBERLANGEN2;
	} else {
		m_scanControllerType = ScanControl::SCAN_DEVICE::ZEISSECU;
	}

	settings.endGroup();

	settings.beginGroup("objective-setup");
	// Read into m_objectiveSlotNames/m_objectiveSlotCalibrationPaths as-is here;
	// initScanControl() (run later at startup, once the active backend and its real
	// "Objective" element maxOptions are known) resizes both to match, preserving these values
	// for whichever slots still exist.
	auto slotCount = settings.value("slot-count", 0).toInt();
	m_objectiveSlotNames.assign(slotCount, std::string{});
	m_objectiveSlotCalibrationPaths.assign(slotCount, std::string{});
	for (int ii = 0; ii < slotCount; ii++) {
		m_objectiveSlotNames[ii] = settings.value(QString("slot-%1-name").arg(ii + 1), "").toString().toStdString();
		m_objectiveSlotCalibrationPaths[ii] = settings.value(QString("slot-%1-calibration-path").arg(ii + 1), "").toString().toStdString();
	}
	settings.endGroup();

	settings.beginGroup("devices-settings");
	auto posX = settings.value("stage-laser-position-x");
	auto posY = settings.value("stage-laser-position-y");
	m_positionScanner = POINT2{ posX.toDouble(), posY.toDouble() };
	// -1 (not found) for settings written before this field existed - setPendingRestoredMarker()
	// then never matches any real objective slot, so an old marker position is simply left
	// unapplied rather than guessed at.
	m_positionScannerObjectiveSlot = settings.value("stage-laser-position-objective-slot", -1).toInt();
	const auto brightfieldRotationDegrees = settings.value("brightfield-view-rotation-degrees", (int)m_brightfieldViewRotation * 90).toInt();
	m_brightfieldViewRotation = (BrightfieldViewRotation)std::clamp(brightfieldRotationDegrees / 90, 0, 3);
	m_brightfieldMirrorHorizontal = settings.value("brightfield-view-mirror-horizontal", m_brightfieldMirrorHorizontal).toBool();
	m_brightfieldMirrorVertical = settings.value("brightfield-view-mirror-vertical", m_brightfieldMirrorVertical).toBool();
	updateBrightfieldTransformButtons();
	m_Brillouin->settings.setXMin(settings.value("stage-x-min", m_Brillouin->settings.xMin).toInt());
	m_Brillouin->settings.setXMax(settings.value("stage-x-max", m_Brillouin->settings.xMax).toInt());
	m_Brillouin->settings.setXSteps(settings.value("stage-x-steps", m_Brillouin->settings.xSteps).toInt());
	m_Brillouin->settings.setYMin(settings.value("stage-y-min", m_Brillouin->settings.yMin).toInt());
	m_Brillouin->settings.setYMax(settings.value("stage-y-max", m_Brillouin->settings.yMax).toInt());
	m_Brillouin->settings.setYSteps(settings.value("stage-y-steps", m_Brillouin->settings.ySteps).toInt());
	m_Brillouin->settings.setZMin(settings.value("stage-z-min", m_Brillouin->settings.zMin).toInt());
	m_Brillouin->settings.setZMax(settings.value("stage-z-max", m_Brillouin->settings.zMax).toInt());
	m_Brillouin->settings.setZSteps(settings.value("stage-z-steps", m_Brillouin->settings.zSteps).toInt());
	m_Brillouin->settings.preCalibration = settings.value("brillouin-pre-calibrate", m_Brillouin->settings.preCalibration).toBool();
	m_Brillouin->settings.postCalibration = settings.value("brillouin-post-calibrate", m_Brillouin->settings.postCalibration).toBool();
	m_Brillouin->settings.conCalibration = settings.value("brillouin-con-calibrate", m_Brillouin->settings.conCalibration).toBool();
	m_Brillouin->settings.conCalibrationInterval = settings.value("brillouin-con-calibrate-interval", m_Brillouin->settings.conCalibrationInterval).toDouble();
	m_Brillouin->settings.nrCalibrationImages = settings.value("brillouin-nr-calibration-images", m_Brillouin->settings.nrCalibrationImages).toInt();
	m_Brillouin->settings.calibrationExposureTime = settings.value("brillouin-calibration-exposure-time", m_Brillouin->settings.calibrationExposureTime).toDouble();
	m_Brillouin->settings.useRoiMask = settings.value("brillouin-use-roi-mask", m_Brillouin->settings.useRoiMask).toBool();
	m_Brillouin->settings.roiPolygonUm = deserializeRoiPolygon(settings.value("brillouin-roi-polygon-um", "").toString());
	m_Brillouin->settings.useBackgroundRoiMask = settings.value("brillouin-use-background-roi-mask", m_Brillouin->settings.useBackgroundRoiMask).toBool();
	m_Brillouin->settings.backgroundRoiPolygonUm = deserializeRoiPolygon(settings.value("brillouin-background-roi-polygon-um", "").toString());
	// useSurfaceFollow is deliberately not restored - always starts off (see saveSettings()).
	m_Brillouin->settings.surfaceZOffsetUm = settings.value("brillouin-surface-z-offset-um", m_Brillouin->settings.surfaceZOffsetUm).toDouble();
	m_Brillouin->settings.surfaceFollowHalfRangeUm = settings.value("brillouin-surface-follow-half-range-um", m_Brillouin->settings.surfaceFollowHalfRangeUm).toDouble();
	m_Brillouin->settings.preScanXYBin = settings.value("brillouin-pre-scan-xy-bin", m_Brillouin->settings.preScanXYBin).toInt();
	m_Brillouin->settings.additionalBoundaryPoints = settings.value("brillouin-additional-boundary-points", m_Brillouin->settings.additionalBoundaryPoints).toInt();
	m_Brillouin->settings.preScanZStepUm = settings.value("brillouin-pre-scan-z-step-um", m_Brillouin->settings.preScanZStepUm).toDouble();
	m_Brillouin->settings.preScanZTravelRangeUm = settings.value("brillouin-pre-scan-z-travel-um", m_Brillouin->settings.preScanZTravelRangeUm).toDouble();
	m_Brillouin->settings.preScanXSteps = settings.value("brillouin-pre-scan-x-steps", m_Brillouin->settings.preScanXSteps).toInt();
	m_Brillouin->settings.preScanYSteps = settings.value("brillouin-pre-scan-y-steps", m_Brillouin->settings.preScanYSteps).toInt();
	m_Brillouin->settings.preScanZSteps = settings.value("brillouin-pre-scan-z-steps", m_Brillouin->settings.preScanZSteps).toInt();
	m_Brillouin->settings.preScanZMin = settings.value("brillouin-pre-scan-z-min", m_Brillouin->settings.preScanZMin).toDouble();
	m_Brillouin->settings.preScanZMax = settings.value("brillouin-pre-scan-z-max", m_Brillouin->settings.preScanZMax).toDouble();
	m_Brillouin->settings.surfaceMetricThreshold = settings.value("brillouin-surface-metric-threshold", m_Brillouin->settings.surfaceMetricThreshold).toDouble();
	m_Brillouin->settings.surfaceSmoothSigmaUm = settings.value("brillouin-surface-smooth-sigma-um", m_Brillouin->settings.surfaceSmoothSigmaUm).toDouble();
	m_Brillouin->settings.surfaceDropFraction = settings.value("brillouin-surface-drop-fraction", m_Brillouin->settings.surfaceDropFraction).toDouble();
	m_Brillouin->settings.mediumReferenceValue = settings.value("brillouin-medium-reference-value", m_Brillouin->settings.mediumReferenceValue).toDouble();
	m_Brillouin->settings.mediumReferenceFrameCount = settings.value("brillouin-medium-reference-frame-count", m_Brillouin->settings.mediumReferenceFrameCount).toInt();
	m_Brillouin->settings.surfaceMaxRewindUm = settings.value("brillouin-surface-max-rewind-um", m_Brillouin->settings.surfaceMaxRewindUm).toDouble();
	m_Brillouin->settings.surfaceVerificationSteps = settings.value("brillouin-surface-verification-steps", m_Brillouin->settings.surfaceVerificationSteps).toInt();
	m_Brillouin->settings.surfaceVerificationFrameAverage = settings.value("brillouin-surface-verification-frame-average", m_Brillouin->settings.surfaceVerificationFrameAverage).toInt();
	m_Brillouin->settings.surfaceVerificationToleranceFraction = settings.value("brillouin-surface-verification-tolerance-fraction", m_Brillouin->settings.surfaceVerificationToleranceFraction).toDouble();
	// gridCoordinatesAbsolute is deliberately not restored - always starts off (see saveSettings()).
	m_Brillouin->settings.absoluteGridOriginUm.x = settings.value("brillouin-absolute-grid-origin-x-um", m_Brillouin->settings.absoluteGridOriginUm.x).toDouble();
	m_Brillouin->settings.absoluteGridOriginUm.y = settings.value("brillouin-absolute-grid-origin-y-um", m_Brillouin->settings.absoluteGridOriginUm.y).toDouble();
	m_Brillouin->settings.absoluteGridOriginUm.z = settings.value("brillouin-absolute-grid-origin-z-um", m_Brillouin->settings.absoluteGridOriginUm.z).toDouble();
	m_Brillouin->settings.useGridHysteresisCompensation = settings.value("brillouin-use-grid-hysteresis-compensation", m_Brillouin->settings.useGridHysteresisCompensation).toBool();
	m_Brillouin->settings.useDoseProtection = settings.value("brillouin-use-dose-protection", m_Brillouin->settings.useDoseProtection).toBool();
	m_Brillouin->settings.saveOverviewBrightfieldPerZ = settings.value("brillouin-save-overview-brightfield-per-z", m_Brillouin->settings.saveOverviewBrightfieldPerZ).toBool();
	m_Brillouin->settings.overviewBrightfieldExposureMs = settings.value("brillouin-overview-brightfield-exposure-ms", m_Brillouin->settings.overviewBrightfieldExposureMs).toInt();
	m_Brillouin->settings.overviewBrightfieldGain = settings.value("brillouin-overview-brightfield-gain", m_Brillouin->settings.overviewBrightfieldGain).toDouble();
	m_Brillouin->settings.overviewBrightfieldFullGrid = settings.value("brillouin-overview-brightfield-full-grid", m_Brillouin->settings.overviewBrightfieldFullGrid).toBool();
	m_Brillouin->settings.overviewBrightfieldFullStackSingle = settings.value("brillouin-overview-brightfield-full-stack-single", m_Brillouin->settings.overviewBrightfieldFullStackSingle).toBool();
	m_Brillouin->settings.overviewBrightfieldFullStackMosaic = settings.value("brillouin-overview-brightfield-full-stack-mosaic", m_Brillouin->settings.overviewBrightfieldFullStackMosaic).toBool();
	m_Brillouin->settings.capturePerPointBrightfield = settings.value("brillouin-capture-per-point-brightfield", m_Brillouin->settings.capturePerPointBrightfield).toBool();
	m_Brillouin->settings.perPointBrightfieldEveryN = settings.value("brillouin-per-point-brightfield-every-n", m_Brillouin->settings.perPointBrightfieldEveryN).toInt();
	m_Brillouin->settings.perPointBrightfieldDuringAcquisition = settings.value("brillouin-per-point-brightfield-during-acquisition", m_Brillouin->settings.perPointBrightfieldDuringAcquisition).toBool();
	m_Brillouin->settings.surfaceProxyRoiLeft = settings.value("brillouin-surface-proxy-roi-left", m_Brillouin->settings.surfaceProxyRoiLeft).toInt();
	m_Brillouin->settings.surfaceProxyRoiTop = settings.value("brillouin-surface-proxy-roi-top", m_Brillouin->settings.surfaceProxyRoiTop).toInt();
	m_Brillouin->settings.surfaceProxyRoiWidth = settings.value("brillouin-surface-proxy-roi-width", m_Brillouin->settings.surfaceProxyRoiWidth).toInt();
	m_Brillouin->settings.surfaceProxyRoiHeight = settings.value("brillouin-surface-proxy-roi-height", m_Brillouin->settings.surfaceProxyRoiHeight).toInt();
	m_Brillouin->settings.surfaceProxyRoi2Left = settings.value("brillouin-surface-proxy-roi-2-left", m_Brillouin->settings.surfaceProxyRoi2Left).toInt();
	m_Brillouin->settings.surfaceProxyRoi2Top = settings.value("brillouin-surface-proxy-roi-2-top", m_Brillouin->settings.surfaceProxyRoi2Top).toInt();
	m_Brillouin->settings.surfaceProxyRoi2Width = settings.value("brillouin-surface-proxy-roi-2-width", m_Brillouin->settings.surfaceProxyRoi2Width).toInt();
	m_Brillouin->settings.surfaceProxyRoi2Height = settings.value("brillouin-surface-proxy-roi-2-height", m_Brillouin->settings.surfaceProxyRoi2Height).toInt();
	m_Brillouin->settings.surfaceProxyRoiFrameWidth = settings.value("brillouin-surface-proxy-roi-frame-width", m_Brillouin->settings.surfaceProxyRoiFrameWidth).toInt();
	m_Brillouin->settings.surfaceProxyRoiFrameHeight = settings.value("brillouin-surface-proxy-roi-frame-height", m_Brillouin->settings.surfaceProxyRoiFrameHeight).toInt();
	m_Brillouin->settings.surfaceProxyRoiFrameOriginLeft = settings.value("brillouin-surface-proxy-roi-frame-origin-left", (qlonglong)m_Brillouin->settings.surfaceProxyRoiFrameOriginLeft).toLongLong();
	m_Brillouin->settings.surfaceProxyRoiFrameOriginBottom = settings.value("brillouin-surface-proxy-roi-frame-origin-bottom", (qlonglong)m_Brillouin->settings.surfaceProxyRoiFrameOriginBottom).toLongLong();
	m_Brillouin->settings.surfaceProxyRoiFrameWidthPhysical = settings.value("brillouin-surface-proxy-roi-frame-width-physical", (qlonglong)m_Brillouin->settings.surfaceProxyRoiFrameWidthPhysical).toLongLong();
	m_Brillouin->settings.surfaceProxyRoiFrameHeightPhysical = settings.value("brillouin-surface-proxy-roi-frame-height-physical", (qlonglong)m_Brillouin->settings.surfaceProxyRoiFrameHeightPhysical).toLongLong();
	m_Brillouin->settings.surfaceProxyRoi2FrameWidth = settings.value("brillouin-surface-proxy-roi-2-frame-width", m_Brillouin->settings.surfaceProxyRoi2FrameWidth).toInt();
	m_Brillouin->settings.surfaceProxyRoi2FrameHeight = settings.value("brillouin-surface-proxy-roi-2-frame-height", m_Brillouin->settings.surfaceProxyRoi2FrameHeight).toInt();
	m_Brillouin->settings.surfaceProxyRoi2FrameOriginLeft = settings.value("brillouin-surface-proxy-roi-2-frame-origin-left", (qlonglong)m_Brillouin->settings.surfaceProxyRoi2FrameOriginLeft).toLongLong();
	m_Brillouin->settings.surfaceProxyRoi2FrameOriginBottom = settings.value("brillouin-surface-proxy-roi-2-frame-origin-bottom", (qlonglong)m_Brillouin->settings.surfaceProxyRoi2FrameOriginBottom).toLongLong();
	m_Brillouin->settings.surfaceProxyRoi2FrameWidthPhysical = settings.value("brillouin-surface-proxy-roi-2-frame-width-physical", (qlonglong)m_Brillouin->settings.surfaceProxyRoi2FrameWidthPhysical).toLongLong();
	m_Brillouin->settings.surfaceProxyRoi2FrameHeightPhysical = settings.value("brillouin-surface-proxy-roi-2-frame-height-physical", (qlonglong)m_Brillouin->settings.surfaceProxyRoi2FrameHeightPhysical).toLongLong();
	settings.endGroup();
}

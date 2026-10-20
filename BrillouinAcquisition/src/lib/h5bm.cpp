#include "stdafx.h"
#include "h5bm.h"
#include "src/helper/logger.h"
#include "filesystem"

using namespace std::filesystem;

void H5BM::reportWriteError(const QString& message) {
	qWarning(logWarning()) << "H5BM:" << message;
	emit(s_writeError(message));
}

H5BM::H5BM(QObject *parent, const std::string& filename, int flags) noexcept
	: QObject(parent) {
	// H5F_ACC_RDONLY is 0, so "flags & H5F_ACC_RDONLY" can never be true - compare directly.
	if (flags == H5F_ACC_RDONLY) {
		m_fileWritable = false;
		if (exists(filename)) {
			m_file = H5Fopen(&filename[0], flags, H5P_DEFAULT);
		}
	} else if (flags & (H5F_ACC_RDWR | H5F_ACC_TRUNC)) {
		m_fileWritable = true;
		if (!exists(filename)) {
			// create the file
			m_file = H5Fcreate(&filename[0], H5F_ACC_EXCL, H5P_DEFAULT, H5P_DEFAULT);

			setAttribute("created", getNow());
			setAttribute("date", getNow());
		} else if (flags & H5F_ACC_RDWR) {
			m_file = H5Fopen(&filename[0], flags, H5P_DEFAULT);
		} else {
			m_file = H5Fcreate(&filename[0], flags, H5P_DEFAULT, H5P_DEFAULT);

			setAttribute("created", getNow());
			setAttribute("date", getNow());
		}
		if (m_file < 0) {
			m_fileWritable = false;
			m_fileValid = false;
		} else {
			// set the version attribute
			setAttribute("version", m_versionstring);
		}
		getRootHandle(m_Brillouin, true);
		getRootHandle(m_ODT, true);
		getRootHandle(m_Fluorescence, true);
	}
}

H5BM::~H5BM() {
	H5Fflush(m_file, H5F_SCOPE_GLOBAL);
	m_Brillouin.close();
	m_ODT.close();
	m_Fluorescence.close();
	if (m_file > -1) {
		if (H5Fclose(m_file) > -1) {
			m_file = -1;
		}
	}
}

void H5BM::getRootHandle(ModeHandles &handle, bool create) {
	handle.rootHandle = H5Gopen2(m_file, handle.modename.c_str(), H5P_DEFAULT);
	if (handle.rootHandle < 0) {
		handle.rootHandle = H5Gcreate2(m_file, handle.modename.c_str(), H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
		setAttribute("last-modified", getNow());
	}
}

ModeHandles* H5BM::getModeHandle(ACQUISITION_MODE mode) {
	switch (mode) {
		case ACQUISITION_MODE::BRILLOUIN:
			return &m_Brillouin;
			break;
		case ACQUISITION_MODE::ODT:
			return &m_ODT;
			break;
		case ACQUISITION_MODE::FLUORESCENCE:
			return &m_Fluorescence;
			break;
		default:
			return nullptr;
	}
}

void H5BM::newRepetition(ACQUISITION_MODE mode) {
	auto handle = getModeHandle(mode);
	getRepetitionHandle(*handle, true);
}

void H5BM::getRepetitionHandle(ModeHandles &handle, bool create) {
	closeGroup(handle.currentRepetitionHandle);

	std::string repetitionCountString = std::to_string(handle.repetitionCount);
	handle.currentRepetitionHandle = H5Gopen2(handle.rootHandle, repetitionCountString.c_str(), H5P_DEFAULT);
	// Check for existing repetitions and do not override them
	while (handle.currentRepetitionHandle > -1) {
		closeGroup(handle.currentRepetitionHandle);

		repetitionCountString = std::to_string(++handle.repetitionCount);
		handle.currentRepetitionHandle = H5Gopen2(handle.rootHandle, repetitionCountString.c_str(), H5P_DEFAULT);
	}

	if (handle.currentRepetitionHandle < 0 && create) {
		handle.currentRepetitionHandle = H5Gcreate2(handle.rootHandle, repetitionCountString.c_str(), H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
	}
	// set the current datetime
	std::string date = getNow();
	setAttribute("date", date, handle.currentRepetitionHandle);
	setAttribute("last-modified", date);
	// create group handles
	handle.groups = std::make_unique <RepetitionHandles>(handle.mode, handle.currentRepetitionHandle, true);
}

void H5BM::writePoint(hid_t group, const std::string& subGroupName, POINT2 point) {
	auto subGroup = H5Gcreate2(group, subGroupName.c_str(), H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
	setAttribute("x", point.x, subGroup);
	setAttribute("y", point.y, subGroup);
	closeGroup(subGroup);
}

template<typename T>
void H5BM::setAttribute(const std::string& attrName, T* attrValue, hid_t parent, hid_t& type_id) {
	hsize_t dims[1] = { 1 };
	hsize_t maxdims[1] = { 1 };
	hid_t space_id = H5Screate_simple(1, dims, maxdims);
	hid_t acpl_id = H5Pcreate(H5P_ATTRIBUTE_CREATE);
	hid_t attr_id;
	attr_id = H5Aopen(parent, attrName.c_str(), H5P_DEFAULT);
	if (attr_id < 0) {
		attr_id = H5Acreate2(parent, attrName.c_str(), type_id, space_id, H5P_DEFAULT, H5P_DEFAULT);
	}
	H5Awrite(attr_id, type_id, attrValue);
	H5Aclose(attr_id);
	H5Pclose(acpl_id);
	H5Sclose(space_id);
	H5Tclose(type_id);
}

void H5BM::setAttribute(const std::string& attrName, const std::string& attr, hid_t parent) {
	auto type_id = H5Tcopy(H5T_C_S1);
	H5Tset_size(type_id, attr.length());
	setAttribute(attrName, attr.c_str(), parent, type_id);
	H5Tclose(type_id);
}

void H5BM::setAttribute(const std::string& attrName, const char* attr, hid_t parent) {
	setAttribute(attrName, std::string(attr), parent);
}

template<typename T>
void H5BM::setAttribute(const std::string& attrName, T attr, hid_t parent) {
	auto type_id = get_memtype<T>();
	setAttribute(attrName, &attr, parent, type_id);
	H5Tclose(type_id);
}

template<typename T>
void H5BM::setAttribute(const std::string& attrName, T attr) {
	setAttribute(attrName, attr, m_file);
}

template<typename T>
T H5BM::getAttribute(std::string attrName, hid_t parent) {
	// Returned as-is if the attribute doesn't exist or the read fails.
	T buf{};
	hid_t attr_id = H5Aopen(parent, attrName.c_str(), H5P_DEFAULT);
	if (attr_id >= 0) {
		hid_t attr_type = H5Aget_type(attr_id);
		if (attr_type >= 0) {
			H5Aread(attr_id, attr_type, &buf);
			H5Tclose(attr_type);
		}
		H5Aclose(attr_id);
	}
	return buf;
}

template<>
std::string H5BM::getAttribute(std::string attrName, hid_t parent) {
	std::string string = "";
	hid_t attr_id = H5Aopen(parent, attrName.c_str(), H5P_DEFAULT);
	if (attr_id >= 0) {
		hsize_t attr_size = H5Aget_storage_size(attr_id);
		hid_t attr_type = H5Aget_type(attr_id);
		if (attr_type >= 0) {
			char *buf = new char[attr_size + 1];
			H5Aread(attr_id, attr_type, buf);
			string.assign(buf, attr_size);
			delete[] buf;
			buf = nullptr;
			H5Tclose(attr_type);
		}
		H5Aclose(attr_id);
	}
	return string;
}

template<typename T>
T H5BM::getAttribute(std::string attrName) {
	return getAttribute<T>(attrName, m_file);
}

void H5BM::setDate(const std::string& date) {
	std::string attrName = "date";
	setAttribute(attrName, date);

	// write last-modified date to file
}

std::string H5BM::getDate() {
	std::string attrName = "date";
	return getAttribute<std::string>(attrName);
}

std::string H5BM::getVersion() {
	std::string attrName = "version";
	return getAttribute<std::string>(attrName);
}

void H5BM::setComment(const std::string& comment) {
	std::string attrName = "comment";
	setAttribute(attrName, comment);

	// write last-modified date to file
	setAttribute("last-modified", getNow());
}

std::string H5BM::getComment() {
	std::string attrName = "comment";
	return getAttribute<std::string>(attrName);
}

void H5BM::setResolution(std::string direction, int resolution) {
	direction = "resolution-" + direction;
	setAttribute(direction, resolution, m_Brillouin.groups->payload);

	// write last-modified date to file
	setAttribute("last-modified", getNow());
}

int H5BM::getResolution(std::string direction) {
	direction = "resolution-" + direction;
	return getAttribute<int>(direction, m_Brillouin.groups->payload);
}

void H5BM::setScaleCalibration(ACQUISITION_MODE mode, ScaleCalibrationDataExtended scaleCalibration) {
	// Create scaleCalibration group
	auto modeHandle = getModeHandle(mode);
	auto scaleCalibrationGroup = H5Gcreate2(modeHandle->groups->payload, "scaleCalibration", H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);

	writePoint(scaleCalibrationGroup, "origin", scaleCalibration.originPix);

	writePoint(scaleCalibrationGroup, "pixToMicrometerX", scaleCalibration.pixToMicrometerX);
	writePoint(scaleCalibrationGroup, "pixToMicrometerY", scaleCalibration.pixToMicrometerY);

	writePoint(scaleCalibrationGroup, "micrometerToPixX", scaleCalibration.micrometerToPixX);
	writePoint(scaleCalibrationGroup, "micrometerToPixY", scaleCalibration.micrometerToPixY);

	writePoint(scaleCalibrationGroup, "positionStage", { scaleCalibration.positionStage.x, scaleCalibration.positionStage.y });
	writePoint(scaleCalibrationGroup, "positionScanner", { scaleCalibration.positionScanner.x, scaleCalibration.positionScanner.y });

	setAttribute("objectiveSlot", scaleCalibration.objectiveSlot, scaleCalibrationGroup);
	setAttribute("objectiveName", scaleCalibration.objectiveName, scaleCalibrationGroup);
	setAttribute("magnification", scaleCalibration.magnification, scaleCalibrationGroup);
	setAttribute("referenceObjectiveName", scaleCalibration.referenceObjectiveName, scaleCalibrationGroup);
	setAttribute("hasFovOffset", scaleCalibration.hasFovOffset ? 1 : 0, scaleCalibrationGroup);
	writePoint(scaleCalibrationGroup, "fovOffset", scaleCalibration.fovOffsetUm);
	setAttribute("fovOffsetSigma", scaleCalibration.fovOffsetSigmaUm, scaleCalibrationGroup);
	setAttribute("missingOffsetAccepted", scaleCalibration.missingOffsetAccepted ? 1 : 0, scaleCalibrationGroup);

	closeGroup(scaleCalibrationGroup);
}

void H5BM::getDataset(std::vector<double>* data, hid_t parent, std::string name) {
	hid_t dset_id = H5Dopen2(parent, name.c_str(), H5P_DEFAULT);

	// get dataspace
	hid_t space_id = H5Dget_space(dset_id);

	// query dataspace length
	hsize_t nrPoints = H5Sget_simple_extent_npoints(space_id);

	// resize positions vector accordingly
	data->resize(nrPoints);

	H5Dread(dset_id, H5T_NATIVE_DOUBLE, H5S_ALL, H5S_ALL, H5P_DEFAULT, data->data());
	H5Sclose(space_id);
	closeDataset(dset_id);
}

void H5BM::setPositions(std::string direction, const std::vector<double>& positions, const int rank, const hsize_t *dims) {
	if (!m_fileWritable) {
		return;
	}
	direction = "positions-" + direction;

	hid_t dset_id = setDataset(m_Brillouin.groups->payload, positions, direction, rank, dims);
	closeDataset(dset_id);

	// write last-modified date to file
	setAttribute("last-modified", getNow());
}

std::vector<double> H5BM::getPositions(std::string direction) {
	direction = "positions-" + direction;

	std::vector<double> positions;
	try {
		getDataset(&positions, m_Brillouin.groups->payload, direction);
	} catch (int e) {
		//
	}
	return positions;
}

std::vector<double> H5BM::getData(const std::string& name, hid_t parent) {
	std::vector<double> data;
	try {
		getDataset(&data, parent, name);
	} catch (int e) {
		//
	}
	return data;
}

std::string H5BM::getDate(std::string name, hid_t parent) {
	std::string date;
	try {
		hid_t dset_id = H5Dopen2(parent, name.c_str(), H5P_DEFAULT);
		date = getAttribute<std::string>("date", dset_id);
		closeDataset(dset_id);
	}
	catch (int e) {
		//
	}
	return date;
}

std::vector<double> H5BM::getPayloadData(int indX, int indY, int indZ) {
	auto name = calculateIndex(indX, indY, indZ);
	return getData(name, m_Brillouin.groups->payload);
}

std::string H5BM::getPayloadDate(int indX, int indY, int indZ) {
	auto name = calculateIndex(indX, indY, indZ);
	return getDate(name, m_Brillouin.groups->payload);
}

std::string H5BM::calculateIndex(int indX, int indY, int indZ) {
	int resolutionX = getResolution("x");
	int resolutionY = getResolution("y");
	int resolutionZ = getResolution("z");

	int index = (indZ*(resolutionX*resolutionY) + indY * resolutionX + indX);
	return std::to_string(index);
}

std::string H5BM::getNow() {
	return QDateTime::currentDateTime().toOffsetFromUtc(QDateTime::currentDateTime().offsetFromUtc())
		.toString(Qt::ISODateWithMs).toStdString();
}

std::vector<double> H5BM::getBackgroundData() {
	return getData("1", m_Brillouin.groups->payload);
}

std::string H5BM::getBackgroundDate() {
	return getDate("1", m_Brillouin.groups->payload);
}

std::vector<double> H5BM::getCalibrationData(int index) {
	return std::vector<double>();
}

std::string H5BM::getCalibrationDate(int index) {
	return std::string();
}

std::string H5BM::getCalibrationSample(int index) {
	return std::string();
}

double H5BM::getCalibrationShift(int index) {
	return 0.0;
}

void H5BM::closeGroup(hid_t& group) {
	if (group > -1) {
		if (H5Gclose(group) > -1) {
			group = -1;
		}
	}
}

void H5BM::closeDataset(hid_t& dataset) {
	if (dataset > -1) {
		if (H5Dclose(dataset) > -1) {
			dataset = -1;
		}
	}
}
int H5BM::lastRepetition(ACQUISITION_MODE mode) {
	auto handle = getModeHandle(mode);
	if (!handle || handle->rootHandle < 0) {
		return -1;
	}
	auto last = -1;
	for (auto n = 0; H5Lexists(handle->rootHandle, std::to_string(n).c_str(), H5P_DEFAULT) > 0; n++) {
		last = n;
	}
	return last;
}

bool H5BM::openRepetition(ACQUISITION_MODE mode, int repetition) {
	auto handle = getModeHandle(mode);
	if (!handle || handle->rootHandle < 0 || repetition < 0) {
		return false;
	}
	const auto name = std::to_string(repetition);
	if (H5Lexists(handle->rootHandle, name.c_str(), H5P_DEFAULT) <= 0) {
		return false;
	}
	closeGroup(handle->currentRepetitionHandle);
	handle->currentRepetitionHandle = H5Gopen2(handle->rootHandle, name.c_str(), H5P_DEFAULT);
	if (handle->currentRepetitionHandle < 0) {
		return false;
	}
	handle->repetitionCount = repetition;
	handle->groups = std::make_unique<RepetitionHandles>(handle->mode, handle->currentRepetitionHandle, false);
	return true;
}

void H5BM::setRepetitionState(ACQUISITION_MODE mode, const std::string& state) {
	if (!m_fileWritable) {
		return;
	}
	auto handle = getModeHandle(mode);
	if (!handle || handle->currentRepetitionHandle < 0) {
		return;
	}
	setAttribute("acquisition_state", state, handle->currentRepetitionHandle);
	setAttribute("acquisition_state_time", getNow(), handle->currentRepetitionHandle);
	setAttribute("last-modified", getNow());
	H5Fflush(m_file, H5F_SCOPE_GLOBAL);
}

std::string H5BM::getRepetitionState(ACQUISITION_MODE mode, int repetition) {
	auto handle = getModeHandle(mode);
	if (!handle || handle->rootHandle < 0 || repetition < 0) {
		return "";
	}
	const auto name = std::to_string(repetition);
	if (H5Lexists(handle->rootHandle, name.c_str(), H5P_DEFAULT) <= 0) {
		return "";
	}
	auto group = H5Gopen2(handle->rootHandle, name.c_str(), H5P_DEFAULT);
	if (group < 0) {
		return "";
	}
	std::string state;
	if (H5Aexists(group, "acquisition_state") > 0) {
		state = getAttribute<std::string>("acquisition_state", group);
	}
	H5Gclose(group);
	return state;
}

void H5BM::setResumeNumbers(const std::string& name, const std::vector<double>& values) {
	if (!m_fileWritable || !m_Brillouin.groups) {
		return;
	}
	// stored with its length in front: a vector can be empty, a dataset of no elements is not written or read reliably
	std::vector<double> stored;
	stored.reserve(values.size() + 1);
	stored.push_back((double)values.size());
	stored.insert(stored.end(), values.begin(), values.end());
	const hsize_t dims[1] = { (hsize_t)stored.size() };
	setPositions("resume-" + name, stored, 1, dims);
}

bool H5BM::getResumeNumbers(const std::string& name, std::vector<double>* values) {
	if (!m_Brillouin.groups || m_Brillouin.groups->payload < 0) {
		return false;
	}
	const auto dataset = "positions-resume-" + name;
	if (H5Lexists(m_Brillouin.groups->payload, dataset.c_str(), H5P_DEFAULT) <= 0) {
		return false;
	}
	std::vector<double> stored;
	try {
		getDataset(&stored, m_Brillouin.groups->payload, dataset);
	} catch (int) {
		return false;
	}
	if (stored.empty() || (size_t)std::llround(stored.front()) + 1 != stored.size()) {
		return false;
	}
	values->assign(stored.begin() + 1, stored.end());
	return true;
}

void H5BM::setResumeText(const std::string& name, const std::string& value) {
	if (!m_fileWritable || !m_Brillouin.groups) {
		return;
	}
	// never empty (an empty HDF5 string attribute is not possible), the prefix is removed again when it is read
	setAttribute("resume_" + name, "s:" + value, m_Brillouin.groups->payload);
}

bool H5BM::getResumeText(const std::string& name, std::string* value) {
	if (!m_Brillouin.groups || m_Brillouin.groups->payload < 0) {
		return false;
	}
	const auto attribute = "resume_" + name;
	if (H5Aexists(m_Brillouin.groups->payload, attribute.c_str()) <= 0) {
		return false;
	}
	const auto text = getAttribute<std::string>(attribute, m_Brillouin.groups->payload);
	if (text.size() < 2 || text.compare(0, 2, "s:") != 0) {
		return false;
	}
	*value = text.substr(2);
	return true;
}

namespace {

	std::vector<std::string> groupMembers(hid_t group) {
		std::vector<std::string> names;
		if (group < 0) {
			return names;
		}
		H5G_info_t info;
		if (H5Gget_info(group, &info) < 0) {
			return names;
		}
		for (hsize_t i = 0; i < info.nlinks; i++) {
			const auto length = H5Lget_name_by_idx(group, ".", H5_INDEX_NAME, H5_ITER_NATIVE, i, nullptr, 0, H5P_DEFAULT);
			if (length < 0) {
				continue;
			}
			std::string name((size_t)length + 1, '\0');
			H5Lget_name_by_idx(group, ".", H5_INDEX_NAME, H5_ITER_NATIVE, i, &name[0], (size_t)length + 1, H5P_DEFAULT);
			name.resize((size_t)length);
			names.push_back(name);
		}
		return names;
	}

}

void H5BM::listPayloadPoints(std::vector<int>* complete, std::vector<int>* incomplete) {
	complete->clear();
	incomplete->clear();
	if (!m_Brillouin.groups || m_Brillouin.groups->payloadData < 0) {
		return;
	}
	for (const auto& name : groupMembers(m_Brillouin.groups->payloadData)) {
		if (name.empty() || name.find_first_not_of("0123456789") != std::string::npos) {
			continue;
		}
		auto dataset = H5Dopen2(m_Brillouin.groups->payloadData, name.c_str(), H5P_DEFAULT);
		if (dataset < 0) {
			incomplete->push_back(std::stoi(name));
			continue;
		}
		// position_z_um is among the last attributes a point gets (see setData()): without it the write was cut short
		const auto isComplete = H5Aexists(dataset, "date") > 0 && H5Aexists(dataset, "position_z_um") > 0;
		H5Dclose(dataset);
		(isComplete ? complete : incomplete)->push_back(std::stoi(name));
	}
}

bool H5BM::deletePayloadPoint(int index) {
	if (!m_fileWritable || !m_Brillouin.groups || m_Brillouin.groups->payloadData < 0) {
		return false;
	}
	return H5Ldelete(m_Brillouin.groups->payloadData, std::to_string(index).c_str(), H5P_DEFAULT) >= 0;
}

std::string H5BM::getPayloadDateByIndex(int index) {
	if (!m_Brillouin.groups || m_Brillouin.groups->payloadData < 0) {
		return "";
	}
	return getDate(std::to_string(index), m_Brillouin.groups->payloadData);
}

int H5BM::countCalibrationData() {
	if (!m_Brillouin.groups) {
		return 0;
	}
	return (int)groupMembers(m_Brillouin.groups->calibrationData).size();
}

int H5BM::countBackgroundData() {
	if (!m_Brillouin.groups) {
		return 0;
	}
	return (int)groupMembers(m_Brillouin.groups->backgroundData).size();
}

int H5BM::payloadIndex(int indX, int indY, int indZ) {
	const auto resolutionX = getResolution("x");
	const auto resolutionY = getResolution("y");
	return indZ * (resolutionX * resolutionY) + indY * resolutionX + indX;
}

bool H5BM::readPositions(const std::string& name, std::vector<double>* values) {
	if (!m_Brillouin.groups || m_Brillouin.groups->payload < 0) {
		return false;
	}
	const auto dataset = "positions-" + name;
	if (H5Lexists(m_Brillouin.groups->payload, dataset.c_str(), H5P_DEFAULT) <= 0) {
		return false;
	}
	values->clear();
	getDataset(values, m_Brillouin.groups->payload, dataset);
	return true;
}

bool H5BM::readPositionsOfRepetition(ACQUISITION_MODE mode, int repetition, const std::string& name, std::vector<double>* values) {
	auto handle = getModeHandle(mode);
	if (!handle || handle->rootHandle < 0 || repetition < 0) {
		return false;
	}
	const auto repetitionName = std::to_string(repetition);
	if (H5Lexists(handle->rootHandle, repetitionName.c_str(), H5P_DEFAULT) <= 0) {
		return false;
	}
	auto group = H5Gopen2(handle->rootHandle, repetitionName.c_str(), H5P_DEFAULT);
	if (group < 0) {
		return false;
	}
	auto found = false;
	if (H5Lexists(group, "payload", H5P_DEFAULT) > 0) {
		auto payload = H5Gopen2(group, "payload", H5P_DEFAULT);
		const auto dataset = "positions-" + name;
		if (payload >= 0) {
			if (H5Lexists(payload, dataset.c_str(), H5P_DEFAULT) > 0) {
				values->clear();
				getDataset(values, payload, dataset);
				found = true;
			}
			H5Gclose(payload);
		}
	}
	H5Gclose(group);
	return found;
}

void H5BM::flushFile() {
	if (m_fileWritable && m_file > -1) {
		H5Fflush(m_file, H5F_SCOPE_GLOBAL);
	}
}

#ifndef BRILLOUINRESUME_H
#define BRILLOUINRESUME_H

#include <map>
#include <string>
#include <vector>

#include "Brillouin.h"
#include "ResumeRecord.h"

/*
 * Continue of an interrupted measurement (crash, cancel, pause): everything a measurement needs to be picked up again is written to its file,
 * here the settings in full. The record is a flat list of named numbers and texts; it is written to the file as one dataset (numbers) or
 * attribute (text) per name and read back the same way.
 */
namespace resume {

	// Every setting of a measurement (grid, surface scan, overviews, drift, calibration, repetitions, camera, ...)
	Record encodeSettings(const BRILLOUIN_SETTINGS& settings);
	// Sets every setting that is in the record; the others keep their value. Returns how many entries were applied.
	int decodeSettings(const Record& record, BRILLOUIN_SETTINGS* settings);
	// All names that encodeSettings() produces (what a reader has to ask the file for)
	std::vector<std::string> settingNames(bool numbers);

	std::string toUtf8(const std::wstring& text);
	std::wstring fromUtf8(const std::string& text);

}

#endif // BRILLOUINRESUME_H

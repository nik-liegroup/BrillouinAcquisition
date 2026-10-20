#ifndef RESUMERECORD_H
#define RESUMERECORD_H

#include <map>
#include <string>
#include <vector>

namespace resume {

	// A flat list of named numbers and texts: what is written to the file as one dataset (numbers) or attribute (text) per name
	struct Record {
		std::map<std::string, std::vector<double>> numbers;
		std::map<std::string, std::string> strings;
	};

}

#endif // RESUMERECORD_H

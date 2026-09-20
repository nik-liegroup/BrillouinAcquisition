#ifndef COM_H
#define COM_H

#include <sstream>
#include <iomanip>
#include <regex>
#include <optional>

#include <QSerialPort>
#include <QtCore>
#include <gsl/gsl>

class com : public QSerialPort {
public:
	com() {};
	explicit com(const std::string& terminator) : m_terminator(terminator) {};

	virtual qint64 writeToDevice(const char* data);

	std::string receive(std::string request);
	void send(std::string message);

protected:
	std::string m_terminator{ "\r" };
	bool waitForReady(int timeout);
};

class helper {
public:
	static std::string dec2hex(int dec, int digits);
	// nullopt if the string is too short to hold a valid hex position reply (e.g. a serial
	// timeout returned an empty/truncated response).
	static std::optional<int> hex2dec(std::string);
	static std::string parse(std::string answer, const std::string& prefix);
};

#endif //COM_H
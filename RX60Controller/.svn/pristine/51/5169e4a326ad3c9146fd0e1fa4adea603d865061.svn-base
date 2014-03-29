//
// C++ Implementation: RX60TCPSocket
//
// Description: 
//
//
// Author: kraft,,, <kraft@tek-4714>, (C) 2008
//
//
//

#include "RX60TCPSocket.hpp"

#include <iostream>
#include <fstream>
#include <boost/asio.hpp>
#include <boost/array.hpp>

using boost::asio::ip::tcp;

RX60TCPSocket::RX60TCPSocket() {

}

RX60TCPSocket::~RX60TCPSocket() {

}

bool RX60TCPSocket::open(const std::string serverAddress,
		const std::string serverPort) {

	std::istringstream isstream(serverPort);

	int port;

	isstream >> port;

	try {
		boost::asio::io_service io_service;

		boost::asio::ip::tcp::endpoint endpoint(
				boost::asio::ip::address::from_string(serverAddress), port);

		_stream.connect(endpoint);

		return true;

	} catch (const boost::system::system_error& error) {
		std::cout << "Could not open connection to " << serverAddress << ":"
				<< serverPort << std::endl;
		return false;
	}

}

bool RX60TCPSocket::close() {

	if (!_stream)
		return false;

	bool ret;

	try {
		_stream.close();
		ret = true;

	} catch (const boost::system::system_error& error) {
		std::cout << "Could not close stream:" << std::endl << error.what()
				<< std::endl;
		ret = false;
		abort();
	}

	return ret;

}

bool RX60TCPSocket::write(const std::string buf) {

	if (!isConnected()) {
		std::cout << "Please establish a connection first!" << std::endl;
		return false;
	}

	try {
		_stream << buf;
	} catch (const std::exception& error) {
		std::cout << error.what() << std::endl;
		return false;
	}

	return true;

}

int RX60TCPSocket::read(std::string & buf) {

	if (!isConnected()) {
		std::cout << "Please establish a connection first!" << std::endl;
		return false;
	}

	try {
		std::getline(_stream, buf);
	} catch (const std::exception& error) {
		std::cout << error.what() << std::endl;
		return -1;
	}

	buf = filter(buf);

	return buf.size();
}

std::string RX60TCPSocket::filter(std::string & buf) {

	std::ostringstream obuf;

	for (unsigned int i = 0; i < buf.size(); i++) {
		if (buf[i] != 0) {
			obuf << buf[i];
		}
	}

	return obuf.str();
}

bool RX60TCPSocket::isConnected() {
	if (_stream && _stream.rdbuf() && _stream.rdbuf()->is_open()) {
		boost::system::error_code error;
		_stream.rdbuf()->remote_endpoint(error);
		if (!error) {
			return true;
		}
	}
	return false;
}

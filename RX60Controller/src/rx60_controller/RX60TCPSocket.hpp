//
// C++ Interface: RX60TCPSocket
//
// Description: 
//
//
// Author: kraft,,, <kraft@tek-4714>, (C) 2008
//
//
//

#ifndef RX60TCPSOCKET_HPP_
#define RX60TCPSOCKET_HPP_

#include <iostream>
#include <boost/asio.hpp>

class RX60TCPSocket {

// Variable definition
public:

protected:

private:
	boost::asio::ip::tcp::iostream _stream;

// Function definition
public:
	RX60TCPSocket();

	~RX60TCPSocket();

	bool open(const std::string serverAddress = "172.16.1.1",
			const std::string serverPort = "22222");
	bool close();

	bool write(const std::string buf);
	int read(std::string & buf);

	bool isConnected();

protected:

private:
	std::string filter(std::string & buf);
};

#endif

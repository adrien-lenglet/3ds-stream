#include <cstdio>
#include <cstring>
#include <boost/asio.hpp>

int main(int argc, char **argv)
{
	argc--;
	argv++;
	if (argc != 1) {
		std::printf("Expected one arg");
		return 1;
	}
	auto addr = argv[0];
	char lastbuf[8];
	try {
		boost::asio::ip::tcp::endpoint ep(boost::asio::ip::address::from_string(addr), 69);
		boost::asio::io_service ios;
		boost::asio::ip::tcp::socket sock(ios, ep.protocol());
		sock.connect(ep);
		std::printf("Connected to 3DS %s!\n", addr);
		while (true) {
			char buf[8];
			if (boost::asio::read(sock, boost::asio::buffer(buf)) != sizeof(buf)) {
				std::printf("Disconnected.\n");
				break;
			}
			if (std::memcmp(buf, lastbuf, sizeof(buf))) {
				for (size_t i = 0; i < 8; i++)
					std::printf("%d ", buf[i]);
				std::printf("\n");
			}
			std::memcpy(lastbuf, buf, sizeof(buf));
		}
	} catch (boost::system::system_error &e) {
		std::printf("ERR: %s\n", e.what());
		return 1;
	}
	return 0;
}
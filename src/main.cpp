#include <cstdio>
#include <cstring>
#include <map>
#include <boost/asio.hpp>
#include <windows.h>

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
	uint32_t last_in = 0;
	try {
		boost::asio::ip::tcp::endpoint ep(boost::asio::ip::address::from_string(addr), 69);
		boost::asio::io_service ios;
		boost::asio::ip::tcp::socket sock(ios, ep.protocol());
		sock.connect(ep);
		std::printf("Connected to 3DS %s!\n", addr);
		const std::map<size_t, size_t> bindings = {
			{0, 0x31},	// N
			{1, 0x30},	// B
			{9, 0x17},	// I
			{10, 0x18},	// O
			{6, 0x16},	// U
			{7, 0x24},	// J
			{5, 0x23},	// H
			{4, 0x25}	// K
		};
		while (true) {
			char buf[8];
			if (boost::asio::read(sock, boost::asio::buffer(buf)) != sizeof(buf)) {
				std::printf("Disconnected.\n");
				break;
			}
			auto in = reinterpret_cast<uint32_t&>(buf[0]);
			size_t map_size = bindings.size();
			INPUT inputs[map_size];
			size_t i = 0;
			for (auto &p : bindings) {
				uint32_t b = 1 << p.first;
				if ((in & b) != (last_in & b)) {
					INPUT c {};
					c.type = INPUT_KEYBOARD;
					c.ki.wScan = p.second;
					c.ki.dwFlags = KEYEVENTF_SCANCODE | ((in & b) ? 0 : KEYEVENTF_KEYUP);
					inputs[i++] = c;
				}
			}
			if (i > 0)
				SendInput(i, inputs, sizeof(INPUT));
			/*if (std::memcmp(buf, lastbuf, sizeof(buf))) {
				for (size_t i = 0; i < 8; i++)
					std::printf("%d ", buf[i]);
				std::printf("\n");
			}*/
			std::memcpy(lastbuf, buf, sizeof(buf));
			last_in = in;
		}
	} catch (boost::system::system_error &e) {
		std::printf("ERR: %s\n", e.what());
		return 1;
	}
	return 0;
}
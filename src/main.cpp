
#include <cstdio>
#include <cstring>
#include <map>
#include <boost/asio.hpp>
#include <Xinput.h>
#include "ViGEm/Client.h"
#include <wingdi.h>
#include <cstdio>
#include <chrono>
#include <thread>
#include <mutex>

#define BIT(n) (1U<<(n))

namespace m3ds {

enum
{
	KEY_A       = BIT(0),       ///< A
	KEY_B       = BIT(1),       ///< B
	KEY_SELECT  = BIT(2),       ///< Select
	KEY_START   = BIT(3),       ///< Start
	KEY_DRIGHT  = BIT(4),       ///< D-Pad Right
	KEY_DLEFT   = BIT(5),       ///< D-Pad Left
	KEY_DUP     = BIT(6),       ///< D-Pad Up
	KEY_DDOWN   = BIT(7),       ///< D-Pad Down
	KEY_R       = BIT(8),       ///< R
	KEY_L       = BIT(9),       ///< L
	KEY_X       = BIT(10),      ///< X
	KEY_Y       = BIT(11),      ///< Y
	KEY_ZL      = BIT(14),      ///< ZL (New 3DS only)
	KEY_ZR      = BIT(15),      ///< ZR (New 3DS only)
	KEY_TOUCH   = BIT(20),      ///< Touch (Not actually provided by HID)
	KEY_CSTICK_RIGHT = BIT(24), ///< C-Stick Right (New 3DS only)
	KEY_CSTICK_LEFT  = BIT(25), ///< C-Stick Left (New 3DS only)
	KEY_CSTICK_UP    = BIT(26), ///< C-Stick Up (New 3DS only)
	KEY_CSTICK_DOWN  = BIT(27), ///< C-Stick Down (New 3DS only)
	KEY_CPAD_RIGHT = BIT(28),   ///< Circle Pad Right
	KEY_CPAD_LEFT  = BIT(29),   ///< Circle Pad Left
	KEY_CPAD_UP    = BIT(30),   ///< Circle Pad Up
	KEY_CPAD_DOWN  = BIT(31),   ///< Circle Pad Down

	// Generic catch-all directions
	KEY_UP    = KEY_DUP    | KEY_CPAD_UP,    ///< D-Pad Up or Circle Pad Up
	KEY_DOWN  = KEY_DDOWN  | KEY_CPAD_DOWN,  ///< D-Pad Down or Circle Pad Down
	KEY_LEFT  = KEY_DLEFT  | KEY_CPAD_LEFT,  ///< D-Pad Left or Circle Pad Left
	KEY_RIGHT = KEY_DRIGHT | KEY_CPAD_RIGHT, ///< D-Pad Right or Circle Pad Right
};

}

static void vAssert(VIGEM_ERROR e)
{
	if (e != VIGEM_ERROR_NONE) {
		std::printf("vigem failed: %x\n", e);
		exit(1);
	}
}

#include "Img.hpp"

int main(int argc, char **argv)
{
	argc--;
	argv++;
	if (argc != 1) {
		std::printf("Expected one arg");
		return 1;
	}

	Img src("./sample.bmp");
	auto dst = Img::create();
	src.sample(dst);
	static constexpr auto e = Img::de;
	std::printf("IMG SIZE: %zu\n", dst.cmp_size(e));
	/*dst.cmp(e, cmp);
	dst.dcmp(e, cmp);
	dst.out("./sample_out.bmp");*/

	HDC hScreenDC = GetDC(nullptr); // CreateDC("DISPLAY",nullptr,nullptr,nullptr);
	HDC hMemoryDC = CreateCompatibleDC(hScreenDC);
	int width = GetDeviceCaps(hScreenDC,HORZRES);
	int height = GetDeviceCaps(hScreenDC,VERTRES);
	HBITMAP hBitmap = CreateCompatibleBitmap(hScreenDC,width,height);

	auto frame_bmp_buf = Img(width + 16, height);
	auto frame_bmp = Img(width, height);

	auto &frame = dst;

	auto vc = vigem_alloc();
	vAssert(vigem_connect(vc));
	const auto pad = vigem_target_x360_alloc();
	vAssert(vigem_target_add(vc, pad));

	auto addr = argv[0];
	char lastbuf[12];
	//uint32_t last_in = 0;

	size_t frame_ndx = 0;
	//auto bef = std::chrono::high_resolution_clock::now();

	std::mutex conn_mtx;
	std::mutex cap_mtx;
	std::mutex enc_mtx;
	std::vector<std::jthread> threads;
	bool stop = false;
	std::mutex stop_mtx;
	static constexpr size_t pp_depth = 3;
	uint8_t *cmps[pp_depth];

	boost::asio::ip::tcp::endpoint ep(boost::asio::ip::address::from_string(addr), 69);
	boost::asio::io_service ios;
	boost::asio::ip::tcp::socket sock(ios, ep.protocol());
	sock.connect(ep);
	std::printf("Connected to 3DS %s!\n", addr);
	for (size_t i = 0; i < pp_depth; i++) {
		auto cmp = dst.alloc_cmp(e);
		cmps[i] = cmp;
		threads.emplace_back([&, i, cmp]() {
			try {
				while (true) {
					{
						std::lock_guard l(conn_mtx);
						char buf[12];
						if (boost::asio::read(sock, boost::asio::buffer(buf)) != sizeof(buf)) {
							std::printf("Disconnected.\n");
							break;
						}
						auto in = reinterpret_cast<uint32_t&>(buf[0]);
						auto x = reinterpret_cast<int16_t&>(buf[4]);
						auto y = reinterpret_cast<int16_t&>(buf[6]);
						size_t cframe_ndx = reinterpret_cast<uint32_t&>(buf[8]);

						XINPUT_STATE state{};
						int32_t scale = 192;
						state.Gamepad.sThumbLX = x * scale;
						state.Gamepad.sThumbLY = y * scale;
						state.Gamepad.wButtons =
							(in & m3ds::KEY_A ? XINPUT_GAMEPAD_B : 0) |
							(in & m3ds::KEY_B ? XINPUT_GAMEPAD_A : 0) |
							(in & m3ds::KEY_X ? XINPUT_GAMEPAD_Y : 0) |
							(in & m3ds::KEY_Y ? XINPUT_GAMEPAD_X : 0) |

							(in & m3ds::KEY_L ? XINPUT_GAMEPAD_LEFT_SHOULDER : 0) |
							(in & m3ds::KEY_R ? XINPUT_GAMEPAD_RIGHT_SHOULDER : 0) |

							(in & m3ds::KEY_START ? XINPUT_GAMEPAD_START : 0) |
							(in & m3ds::KEY_SELECT ? XINPUT_GAMEPAD_LEFT_THUMB : 0) |

							(in & m3ds::KEY_DLEFT ? XINPUT_GAMEPAD_DPAD_LEFT : 0) |
							(in & m3ds::KEY_DRIGHT ? XINPUT_GAMEPAD_DPAD_RIGHT : 0) |
							(in & m3ds::KEY_DUP ? XINPUT_GAMEPAD_DPAD_UP : 0) |
							(in & m3ds::KEY_DDOWN ? XINPUT_GAMEPAD_DPAD_DOWN : 0)
							;

						if (std::memcmp(buf, lastbuf, sizeof(buf))) {
							vigem_target_x360_update(vc, pad, *reinterpret_cast<XUSB_REPORT*>(&state.Gamepad));
							/*for (size_t i = 0; i < 8; i++)
								std::printf("%d ", buf[i]);
							std::printf("\n");*/
						}
						std::memcpy(lastbuf, buf, sizeof(buf));
						//last_in = in;
						if (cframe_ndx + 1 >= frame_ndx) {
							boost::asio::write(sock, boost::asio::buffer(cmp, frame.cmp_size(e)));
							frame_ndx++;
							if (frame_ndx % 15 == 0)
								std::printf("frame: %zu\n", frame_ndx);

							/*auto now = std::chrono::high_resolution_clock::now();
							auto delta = static_cast<std::chrono::duration<double>>(now - bef).count();
							bef = now;
							double ts = 1.0 / 50.0 - delta;
							if (ts > 0)
								std::this_thread::sleep_for(std::chrono::nanoseconds(static_cast<size_t>(ts) * 1000000000));*/
						}
					}
					{
						std::lock_guard l(cap_mtx);
						HBITMAP hOldBitmap = static_cast<HBITMAP>(SelectObject(hMemoryDC,hBitmap));
						BitBlt(hMemoryDC,0,0,width,height,hScreenDC,0,0,SRCCOPY);
						hBitmap = static_cast<HBITMAP>(SelectObject(hMemoryDC,hOldBitmap));
						BITMAPINFO bi{
							{
								sizeof(BITMAPINFOHEADER),
								width,
								height,
								1,	// biPlanes
								24,	// biBitCount
								BI_RGB,	// biCompression
								static_cast<uint32_t>(Img::computeStride(width) * height),	// biSizeImage
								1,	// biXPelsPerMeter
								1,	// biYPelsPerMeter
								0,	// biClrUsed
								0	// biClrImportant
							},
							{}
						};
						assert(GetDIBits(hMemoryDC, hBitmap, 0, height, frame_bmp_buf.get_data(), &bi, DIB_RGB_COLORS));
					}
					{
						std::lock_guard l(enc_mtx);
						frame_bmp.load(frame_bmp_buf.get_data());
						frame_bmp.sample(dst);
						dst.cmp<e>(cmp);
					}
				}
			} catch (boost::system::system_error &e) {
				std::printf("ERR: %s\n", e.what());
				{
					std::lock_guard l(stop_mtx);
					stop = true;
				}
			}
		});
	}
	//cleanup();
	while (true) {
		{
			std::lock_guard l(stop_mtx);
			if (stop)
				break;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
	DeleteDC(hMemoryDC);
	DeleteDC(hScreenDC);
	for (size_t i = 0; i < pp_depth; i++)
		delete[] cmps[i];
	vigem_target_remove(vc, pad);
	vigem_target_free(pad);
	vigem_disconnect(vc);
	vigem_free(vc);
	return 0;
}
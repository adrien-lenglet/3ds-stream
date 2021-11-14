
#include <cstdio>
#include <cstring>
#include <map>
#include <boost/asio.hpp>
#include <Xinput.h>
#include "ViGEm/Client.h"
#include <wingdi.h>
#include <cstdio>

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

static char sbuf[4000000];

static size_t align_up(size_t v, size_t al)
{
	size_t mod = v % al;
	if (mod == 0)
		return v;
	else
		return v - mod + al;
}

class Img
{
	size_t m_w;
	size_t m_h;
	uint8_t *m_data;

	static size_t computeStride(size_t w)
	{
		return align_up(w * 3, 4);
	}

public:
	Img(const char *filename)
	{
		FILE *f = fopen(filename, "rb");
		if (f == nullptr)
			throw std::runtime_error(filename);
		fseek(f, 0x0A, SEEK_SET);
		uint32_t off;
		assert(fread(&off, sizeof(off), 1, f) == 1);
		fseek(f, 0x0E, SEEK_SET);
		BITMAPINFOHEADER h;
		assert(fread(&h, sizeof(h), 1, f) == 1);
		m_w = h.biWidth;
		m_h = h.biHeight;
		m_data = new uint8_t[m_w * m_h * 3];
		if (h.biBitCount != 24)
			throw std::runtime_error("BPP must be 24");

		size_t s = computeStride(m_w);
		size_t ds = m_w * 3;
		for (size_t i = 0; i < m_h; i++) {
			fseek(f, off + s * i, SEEK_SET);
			assert(fread(m_data + ds * i, ds, 1, f) == 1);
		}
		fclose(f);
		//std::printf("%zu, %zu\n", m_w, m_h);
	}
	Img(size_t w, size_t h) :
		m_w(w),
		m_h(h),
		m_data(new uint8_t[m_w * m_h * 3])
	{
	}
	~Img(void)
	{
		delete[] m_data;
	}

	void out(const char *filename)
	{
		FILE *f = fopen(filename, "wb");
		if (f == nullptr)
			throw std::runtime_error(filename);

		auto s = computeStride(m_w);
		assert(fwrite("BM", 2, 1, f) == 1);
		{
			uint32_t b = 0x0E + sizeof(BITMAPINFOHEADER) + s * m_h;
			assert(fwrite(&b, sizeof(uint32_t), 1, f) == 1);
			b = 0;
			assert(fwrite(&b, sizeof(uint16_t), 1, f) == 1);
			assert(fwrite(&b, sizeof(uint16_t), 1, f) == 1);
			b = 0x0E + sizeof(BITMAPINFOHEADER);
			assert(fwrite(&b, sizeof(uint32_t), 1, f) == 1);
		}

		BITMAPINFOHEADER hi{
			sizeof(hi),	// biSize
			static_cast<int32_t>(m_w),	// biWidth
			static_cast<int32_t>(m_h),	// biWidth
			1,	// biPlanes
			24,	// biBitCount
			BI_RGB,	// biCompression
			static_cast<uint32_t>(s * m_h),	// biSizeImage
			1,	// biXPelsPerMeter
			1,	// biYPelsPerMeter
			0,	// biClrUsed
			0	// biClrImportant
		};
		assert(fwrite(&hi, sizeof(hi), 1, f) == 1);

		size_t ds = m_w * 3;
		uint8_t row[s] {};
		for (size_t i = 0; i < m_h; i++) {
			std::memcpy(row, m_data + ds * i, ds);
			assert(fwrite(row, s, 1, f) == 1);
		}
		fclose(f);
	}

	void sample(Img &dst)
	{
		static auto nr = [](size_t i, size_t s) {
			return static_cast<double>(i) / static_cast<double>(s - 1);
		};
		static auto dnr = [](double v, size_t s) {
			return static_cast<size_t>(v * static_cast<double>(s - 1));
		};
		static auto cv = [](size_t i, size_t ss, size_t ds) {
			return dnr(nr(i, ss), ds);
		};
		class px { uint8_t data[3]; };
		for (size_t i = 0; i < dst.m_h; i++)
			for (size_t j = 0; j < dst.m_w; j++)
				reinterpret_cast<px*>(dst.m_data)[i * dst.m_w + j] =
					reinterpret_cast<px*>(m_data)[cv(i, dst.m_h, m_h) * m_w + cv(j, dst.m_w, m_w)];
	}
};

int main(int argc, char **argv)
{
	argc--;
	argv++;
	if (argc != 1) {
		std::printf("Expected one arg");
		return 1;
	}

	Img src("./sample.bmp");
	Img dst(400, 240);
	src.sample(dst);
	dst.out("./sample_out.bmp");

	auto vc = vigem_alloc();
	vAssert(vigem_connect(vc));
	const auto pad = vigem_target_x360_alloc();
	vAssert(vigem_target_add(vc, pad));

	auto cleanup = [&]() {
		vigem_target_remove(vc, pad);
		vigem_target_free(pad);
		vigem_disconnect(vc);
		vigem_free(vc);
	};

	auto addr = argv[0];
	char lastbuf[8];
	//uint32_t last_in = 0;

	try {
		boost::asio::ip::tcp::endpoint ep(boost::asio::ip::address::from_string(addr), 69);
		boost::asio::io_service ios;
		boost::asio::ip::tcp::socket sock(ios, ep.protocol());
		sock.connect(ep);
		std::printf("Connected to 3DS %s!\n", addr);
		while (true) {
			char buf[8];
			/*if (boost::asio::read(sock, boost::asio::buffer(buf)) != sizeof(buf)) {
				std::printf("Disconnected.\n");
				break;
			}*/
			auto in = reinterpret_cast<uint32_t&>(buf[0]);
			auto x = reinterpret_cast<int16_t&>(buf[4]);
			auto y = reinterpret_cast<int16_t&>(buf[6]);

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

			sock.write_some(boost::asio::buffer(sbuf));
		}
	} catch (boost::system::system_error &e) {
		std::printf("ERR: %s\n", e.what());
		cleanup();
		return 1;
	}
	cleanup();
	return 0;
}
/*
	Circle Pad example made by Aurelio Mannara for libctru
	Please refer to https://github.com/devkitPro/libctru/blob/master/libctru/include/3ds/services/hid.h for more information
	This code was modified for the last time on: 12/13/2014 2:20 UTC+1

	This wouldn't be possible without the amazing work done by:
	-Smealum
	-fincs
	-WinterMute
	-yellows8
	-plutoo
	-mtheall
	-Many others who worked on 3DS and I'm surely forgetting about
*/

#include <3ds.h>
#include <stdio.h>
#include <stdlib.h>
#include <malloc.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <stdarg.h>

#include <fcntl.h>

#include <sys/types.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <chrono>
#include <mutex>
#include <thread>
#include <vector>
#include "Img.hpp"

#define SOC_ALIGN       0x1000
#define SOC_BUFFERSIZE  0x100000

static u32 *SOC_buffer = nullptr;
static s32 sock = -1, csock = -1;

//---------------------------------------------------------------------------------
void failExit(const char *fmt, ...) {
//---------------------------------------------------------------------------------

	if(sock>0) close(sock);
	if(csock>0) close(csock);

	va_list ap;

	printf(CONSOLE_RED);
	va_start(ap, fmt);
	vprintf(fmt, ap);
	va_end(ap);
	printf(CONSOLE_RESET);
	printf("\nPress B to exit\n");

	while (aptMainLoop()) {
		gspWaitForVBlank();
		hidScanInput();

		u32 kDown = hidKeysDown();
		if (kDown & KEY_B) exit(0);
	}
}

//---------------------------------------------------------------------------------
void socShutdown() {
//---------------------------------------------------------------------------------
	printf("waiting for socExit...\n");
	socExit();

}

static void svcAssert(Result r, const char *code)
{
	if (r != 0) {
		printf("Func '%s' failed: %li\n", code, r);
		while (true) {
			hidScanInput();
			auto kDown = hidKeysHeld();
			if (kDown & KEY_START)
				break;
			gfxFlushBuffers();
			gfxSwapBuffers();
			gspWaitForVBlank();
		}
		exit(1);
	}
}

#define sAssert(v) svcAssert(v, #v)

template <typename Fn>
Handle launchThread(void *stack_top, s32 thread_priority, s32 processor_id, Fn &&fn)
{
	Handle res;
	svcAssert(svcCreateThread(&res, [](void *ptr) {
		(*reinterpret_cast<std::remove_reference_t<Fn>*>(ptr))();
	}, reinterpret_cast<u32>(static_cast<void*>(&fn)), reinterpret_cast<u32*>(stack_top), thread_priority, processor_id), "create thread");
	return res;
}

//static char rbuf[4000000];

static constexpr size_t pp_depth = 1;
static constexpr size_t stack_size = 1024 * 1024;

class mutex
{
	Handle m_handle;

public:
	mutex(void)
	{
		sAssert(svcCreateMutex(&m_handle, false));
	}
	~mutex(void)
	{
		sAssert(svcCloseHandle(m_handle));
	}

	void lock(void)
	{
		sAssert(svcWaitSynchronization(m_handle, U64_MAX));
	}

	void unlock(void)
	{
		sAssert(svcReleaseMutex(m_handle));
	}
};

int main(int argc, char **argv)
{
	gfxInitDefault();
	consoleInit(GFX_BOTTOM, NULL);
	gfxSetDoubleBuffering(GFX_TOP, false);
	u16 fbw, fbh;
	static u8* fb = gfxGetFramebuffer(GFX_TOP, GFX_LEFT, &fbw, &fbh);
	printf("FB: %u, %u\n", fbw, fbh);

	SOC_buffer = (u32*)memalign(SOC_ALIGN, SOC_BUFFERSIZE);

	if(SOC_buffer == NULL)
		failExit("memalign: failed to allocate\n");

	Result ret;
	if ((ret = socInit(SOC_buffer, SOC_BUFFERSIZE)) != 0)
		failExit("socInit: 0x%08X\n", (unsigned int)ret);

	atexit(socShutdown);

	struct sockaddr_in client, server;
	u32 clientlen = sizeof(client);

	sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);

	if (sock < 0)
		failExit("socket: %d %s\n", errno, strerror(errno));

	memset(&server, 0, sizeof (server));
	memset(&client, 0, sizeof (client));

	server.sin_family = AF_INET;
	server.sin_port = htons(69);
	server.sin_addr.s_addr = gethostid();

	printf("3DS joystick listening at %s/\n", inet_ntoa(server.sin_addr));
		
	if ( (ret = bind (sock, (struct sockaddr *) &server, sizeof (server))) ) {
		close(sock);
		failExit("bind: %d %s\n", errno, strerror(errno));
	}

	fcntl(sock, F_SETFL, fcntl(sock, F_GETFL, 0));

	if ( (ret = listen( sock, 1)) ) {
		failExit("listen: %d %s\n", errno, strerror(errno));
	}

	static bool isdisc = true;
	static bool done = false;
	static auto frame = Img::create();
	static constexpr auto e = Img::de;
	//size_t cmp_size = frame.cmp_size(e);
	static uint8_t *cmp[2];
	for (size_t i = 0; i < 2; i++)
		cmp[i] = frame.alloc_cmp(e);
	static auto buf = BlksBuf();
	static auto blk_0 = frame.alloc_blk(e);
	static auto blk_1 = frame.alloc_blk(e);

	static auto base = svcGetSystemTick();

	std::vector<Handle> threads;
	static uint32_t frame_ndx = 0;

	static mutex input_mtx;
	static mutex printf_mtx;
	static mutex finish_mtx;
	static size_t finish_count = 0;
	static mutex disp_mtx[2];

	static float fps = 0.0;

	uint8_t *stacks = reinterpret_cast<uint8_t*>(memalign(8, pp_depth * stack_size + 16));

	/*{	// unaligned read test, should print 255 in both cases
		uint32_t v[2] {
			0xFF00,
			0
		};
		printf("v: %lu\n", *reinterpret_cast<uint32_t*>(reinterpret_cast<uint8_t*>(&v) + 1));
		printf("v2: %u\n", *reinterpret_cast<uint16_t*>(reinterpret_cast<uint8_t*>(&v) + 1));
	}*/

	for (size_t i = 0; i < pp_depth; i++) {
		threads.emplace_back(launchThread(stacks + (i + 1) * stack_size, 60, -2, [&]() {
			size_t cd = 0;
			size_t ckcd = 0;
			bool disc = true;
			while (true) {
				if (ckcd++ >= 60) {
					input_mtx.lock();
					if (done) {
						input_mtx.unlock();
						break;
					}
					disc = isdisc;
					input_mtx.unlock();
					ckcd = 0;
				}
				if (!disc) {
					disp_mtx[cd].lock();

					Img::dcmp<e>(cmp[cd], blk_0, blk_1, fb, buf);
					auto t = blk_0;
					blk_0 = blk_1;
					blk_1 = t;

					gfxFlushBuffers();
					gfxSwapBuffers();

					disp_mtx[cd].unlock();
					cd = (cd + 1) % 2;
				} else
					svcSleepThread(50000000);
			}
			finish_mtx.lock();
			finish_count++;
			finish_mtx.unlock();
			svcExitThread();
		}));
	}

	while (true) {
		printf_mtx.lock();
		printf("Waiting for client.. You won't be able to exit the app until someone connects (blocking socket).\n");
		printf_mtx.unlock();
		while (true) {
			csock = accept(sock, (struct sockaddr *) &client, &clientlen);
			if (csock >= 0) {
				printf("Client connected!\n");
				break;
			}
		}

		consoleClear();

		frame_ndx = 0;
		input_mtx.lock();
		isdisc = false;
		done = false;
		input_mtx.unlock();

		base = svcGetSystemTick();

		size_t cd = 0;
		while (true) {
			{
				disp_mtx[cd].lock();
				if (!aptMainLoop()) {
					input_mtx.lock();
					done = true;
					input_mtx.unlock();
					disp_mtx[cd].unlock();
					goto done;
				}
				hidScanInput();

				auto kDown = hidKeysHeld();
				if ((kDown & KEY_START) && (kDown & KEY_SELECT)) {
					input_mtx.lock();
					done = true;
					input_mtx.unlock();
					disp_mtx[cd].unlock();
					goto done;
				}
				circlePosition pos;
				hidCircleRead(&pos);
				char buf[8 + sizeof(frame_ndx)];
				reinterpret_cast<decltype(kDown)&>(buf[0]) = kDown;
				reinterpret_cast<decltype(pos.dx)&>(buf[4]) = pos.dx;
				reinterpret_cast<decltype(pos.dy)&>(buf[6]) = pos.dy;
				reinterpret_cast<decltype(frame_ndx)&>(buf[8]) = frame_ndx;
				if (write(csock, buf, sizeof(buf)) != sizeof(buf)) {
					input_mtx.lock();
					isdisc = true;
					input_mtx.unlock();
					disp_mtx[cd].unlock();
					goto cdisc;
				}

				{
					uint16_t s;
					{
						uint16_t sf = 0;
						while (true) {
							auto g = read(csock, reinterpret_cast<uint8_t*>(&s) + sf, sizeof(s));
							if (g < 0) {
								input_mtx.lock();
								isdisc = true;
								input_mtx.unlock();
								disp_mtx[cd].unlock();
								goto cdisc;
							}
							sf += static_cast<uint16_t>(g);
							if (sf >= sizeof(s))
								break;
						}
					}
					{
						uint16_t sf = 0;
						while (true) {
							auto g = read(csock, cmp[cd] + sf, s - sf);
							if (g < 0) {
								input_mtx.lock();
								isdisc = true;
								input_mtx.unlock();
								disp_mtx[cd].unlock();
								goto cdisc;
							}
							sf += static_cast<uint16_t>(g);
							if (sf >= s)
								break;
						}
					}
					frame_ndx++;
					static constexpr size_t max_frames = 60;
					if (frame_ndx % max_frames == 0) {
						auto now = svcGetSystemTick();
						auto delta = static_cast<float>(now - base) / (CPU_TICKS_PER_MSEC * 1000.0);
						base = now;
						fps = static_cast<float>(max_frames) / delta;
						printf("%g FPS\n", fps);
					}
				}

				disp_mtx[cd].unlock();
				cd = (cd + 1) % 2;
			}
		}

		cdisc:
		printf("Client disconnected.\n");
		close(csock);
		csock = -1;
	}
done:
	while (true) {
		finish_mtx.lock();
		printf("Cleaning up (done: %d).. %zu threads have joined.\n", done, finish_count);
		if (finish_count >= pp_depth) {
			finish_mtx.unlock();
			break;
		}
		finish_mtx.unlock();
		svcSleepThread(50000000);
	}
	gfxExit();
	return 0;
}

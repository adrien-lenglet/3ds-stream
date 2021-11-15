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
#include "../../win-driver/src/Img.hpp"

#define SOC_ALIGN       0x1000
#define SOC_BUFFERSIZE  0x100000

static u32 *SOC_buffer = nullptr;
s32 sock = -1, csock = -1;

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

static constexpr size_t pp_depth = 2;
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
	u8* fb = gfxGetFramebuffer(GFX_TOP, GFX_LEFT, &fbw, &fbh);
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

	bool isdisc = false;
	bool done = false;
	auto frame = Img::create();
	auto e = Img::Enc::create();
	size_t cmp_size = frame.cmp_size(e);
	auto cmp = frame.alloc_cmp(e);
	//size_t bcount = frame.blk_count(e);
	//size_t bpx_count = e.blk_px_count;

	auto base = svcGetSystemTick();

	std::vector<Handle> threads;
	uint32_t frame_ndx = 0;

	mutex input_mtx;
	mutex dec_mtx;
	mutex printf_mtx;
	mutex finish_mtx;
	size_t finish_count = 0;

	input_mtx.lock();
	float fps = 0.0;

	uint8_t *stacks = reinterpret_cast<uint8_t*>(memalign(8, (pp_depth + 1) * stack_size + 16));

	//svcWaitSynchronization

	for (size_t i = 0; i < pp_depth; i++) {
		threads.emplace_back(launchThread(stacks + (i + 1) * stack_size, 0x18, -2, [&]() {
			while (true) {
				{
					input_mtx.lock();
					if (done) {
						input_mtx.unlock();
						break;
					}
					hidScanInput();

					auto kDown = hidKeysHeld();
					if ((kDown & KEY_START) && (kDown & KEY_SELECT)) {
						done = true;
						input_mtx.unlock();
						break;
					}
					circlePosition pos;
					hidCircleRead(&pos);
					char buf[8 + sizeof(frame_ndx)];
					reinterpret_cast<decltype(kDown)&>(buf[0]) = kDown;
					reinterpret_cast<decltype(pos.dx)&>(buf[4]) = pos.dx;
					reinterpret_cast<decltype(pos.dy)&>(buf[6]) = pos.dy;
					reinterpret_cast<decltype(frame_ndx)&>(buf[8]) = frame_ndx;
					if (isdisc)
						goto input_end;
					if (write(csock, buf, sizeof(buf)) != sizeof(buf)) {
						//printf("Client disconnected.\n");
						isdisc = true;
						goto input_end;
					}

					{
						size_t sf = 0;
						while (true) {
							auto g = read(csock, cmp + sf, cmp_size - sf);
							if (g < 0) {
								//printf("Client disconnected (%d).\n", g);
								isdisc = true;
								goto input_end;
							}
							sf += static_cast<size_t>(g);
							if (sf >= cmp_size)
								break;
						}
						frame_ndx++;
						static constexpr size_t max_frames = 15;
						if (frame_ndx % max_frames == 0) {
							auto now = svcGetSystemTick();
							auto delta = static_cast<float>(now - base) / (CPU_TICKS_PER_MSEC * 1000.0);
							base = now;
							fps = static_cast<float>(max_frames) / delta;
						}
					}

				input_end:
					input_mtx.unlock();
				}
				{
					dec_mtx.lock();

					frame.dcmp(e, cmp);
					frame.flip(fb);
					//std::memcpy(fb, frame.get_data(), 400 * 240 * 3);

					gfxFlushBuffers();
					gfxSwapBuffers();
					//svcSleepThread(1000000);

					dec_mtx.unlock();
				}
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
		isdisc = false;
		done = false;

		base = svcGetSystemTick();
		input_mtx.unlock();

		size_t print_lim = 0;
		while (true) {
			input_mtx.lock();
			if (done) {
				input_mtx.unlock();
				goto done;
			}
			if (isdisc)
				goto cdisc;
			if (!aptMainLoop()) {
				done = true;
				input_mtx.unlock();
				goto done;
			}
			input_mtx.unlock();
			if (print_lim++ >= 60) {
				printf("%g FPS\n", fps);
				print_lim = 0;
			}
			svcSleepThread(50000000);
		}

		cdisc:
		printf("Client disconnected.\n");
		close(csock);
		csock = -1;
	}
done:
	while (true) {
		finish_mtx.lock();
		if (finish_count >= pp_depth) {
			finish_mtx.unlock();
			break;
		}
		finish_mtx.unlock();
		svcSleepThread(50000000);
	}
	close(csock);
	gfxExit();
	return 0;
}

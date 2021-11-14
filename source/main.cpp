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

//static char rbuf[4000000];

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

	sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);

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

	auto frame = Img::create();
	auto e = Img::Enc::create();
	size_t cmp_size = frame.cmp_size(e);
	auto cmp = frame.alloc_cmp(e);
	//size_t bcount = frame.blk_count(e);
	//size_t bpx_count = e.blk_px_count;

	while (true) {
		printf("Waiting for client.. You won't be able to exit the app until someone connects (blocking socket).\n");
		while (true) {
			hidScanInput();
			if (hidKeysDown() & KEY_START)
				goto done;

			csock = accept(sock, (struct sockaddr *) &client, &clientlen);
			if (csock >= 0) {
				printf("Client connected!\n");
				break;
			}
			/*gfxFlushBuffers();
			gfxSwapBuffers();
			gspWaitForVBlank();*/
		}

		consoleClear();

		/*auto bef = svcGetSystemTick();
		size_t rec = 0;
		size_t it = 0;
		float clock = 0.0;*/

		auto base = svcGetSystemTick();

		uint32_t frame_ndx = 0;
		while (aptMainLoop()) {
			hidScanInput();

			auto kDown = hidKeysHeld();
			if ((kDown & KEY_START) && (kDown & KEY_SELECT))
				goto done;
			circlePosition pos;
			hidCircleRead(&pos);
			char buf[8 + sizeof(frame_ndx)];
			reinterpret_cast<decltype(kDown)&>(buf[0]) = kDown;
			reinterpret_cast<decltype(pos.dx)&>(buf[4]) = pos.dx;
			reinterpret_cast<decltype(pos.dy)&>(buf[6]) = pos.dy;
			reinterpret_cast<decltype(frame_ndx)&>(buf[8]) = frame_ndx;
			if (write(csock, buf, sizeof(buf)) != sizeof(buf)) {
				printf("Client disconnected.\n");
				break;
			}

			size_t sf = 0;
			while (true) {
				auto g = read(csock, cmp + sf, cmp_size - sf);
				if (g < 0) {
					printf("Client disconnected (%d).\n", g);
					goto cdisc;
				}
				sf += static_cast<size_t>(g);
				if (sf >= cmp_size)
					break;
			}
			frame_ndx++;
			if (frame_ndx % 15 == 0) {
				auto now = svcGetSystemTick();
				auto delta = static_cast<float>(now - base) / (CPU_TICKS_PER_MSEC * 1000.0);
				printf("frame: %lu, %g FPS\n", frame_ndx, static_cast<double>(frame_ndx + 1) / delta);
			}
			frame.dcmp(e, cmp);
			frame.flip(fb);
			//std::memcpy(fb, frame.get_data(), 400 * 240 * 3);

			/*rec += read(csock, rbuf, sizeof(rbuf));
			if (it++ == 256) {

				auto now = svcGetSystemTick();
				auto delta = static_cast<float>(now - bef) / (CPU_TICKS_PER_MSEC * 1000.0);
				bef = now;
				clock += delta;
				//printf("rate: %g (%u), time: %g\n", static_cast<float>(rec) / delta, rec, clock);
				it = 0;
				rec = 0;
			}*/

			gfxFlushBuffers();
			gfxSwapBuffers();
			/*gspWaitForVBlank();*/
			//svcSleepThread(1000000);
		}
		cdisc:
		close(csock);
		csock = -1;
	}
done:
	gfxExit();
	return 0;
}

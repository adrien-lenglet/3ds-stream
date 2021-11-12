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

int main(int argc, char **argv)
{
	gfxInitDefault();
	consoleInit(GFX_TOP, NULL);

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

		while (aptMainLoop()) {
			hidScanInput();

			auto kDown = hidKeysDown();
			if (kDown & KEY_START)
				goto done;
			circlePosition pos;
			hidCircleRead(&pos);
			char buf[8];
			reinterpret_cast<decltype(kDown)&>(buf[0]) = kDown;
			reinterpret_cast<decltype(pos.dx)&>(buf[4]) = pos.dx;
			reinterpret_cast<decltype(pos.dy)&>(buf[6]) = pos.dy;
			if (write(csock, buf, sizeof(buf)) != sizeof(buf)) {
				printf("Client disconnected.\n");
				break;
			}

			/*gfxFlushBuffers();
			gfxSwapBuffers();
			gspWaitForVBlank();*/
			svcSleepThread(1000000);
		}
		close(csock);
		csock = -1;
	}
done:
	gfxExit();
	return 0;
}

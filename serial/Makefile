#command:
#		@echo	makefile checksum
GCC=~/workspace/S32V234_toolchain/gcc-linaro-aarch64-linux-gnu-4.9-2014.09_linux/bin/aarch64-linux-gnu-gcc
all:
	# gcc  serial_test.c -Werror -g -o serial -lm -lpthread
	arm-linux-gnueabihf-gcc  serial_test.c -Werror -g -o serial -lm -lpthread
	#$(GCC) radar_UDP_server.c -Werror -g -o s32v_radar_UDP_server -lm
clean:
	rm -rf *.o

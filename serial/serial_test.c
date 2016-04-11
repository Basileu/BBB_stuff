#include<arpa/inet.h>
#include<sys/socket.h>
#include <errno.h>
#include <string.h>
#include <pthread.h>
#include<math.h>
#include <signal.h>
#include <strings.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <sys/stat.h>
#include <fcntl.h>


static const char *PORT_NAME = "/dev/ttyUSB0";

unsigned char arr [20] = {0xFA,0xDD,0xF1,0x4b,0x51,0x8,0x30,0x0,0x55,0x8,
			  0x2d,0x0, 0x59, 0x08, 0x23, 0x0, 0x54, 0x08,  0x12, 0};

/* Defines */
#define NUM_POINTS 5
#define NUM_COMMANDS 4

/* Global vars */
pthread_t thread;
static pthread_mutex_t mtx = PTHREAD_MUTEX_INITIALIZER;
int fd_robo_LIDAR;
struct termios SerialPortSettings;
char buffer[2048];
// char buffer_data[1048576];
uint16_t buffer_data[65536];
char response[4096];
uint16_t Distance[360];
uint16_t GoodReadings = 0, BadReadings = 0;
uint16_t AnglesCovered = 0;
FILE *fp;
FILE * temp;
FILE * gnuplotPipe;
uint16_t deg;
float radians, displayDistance;
const float RAD = (6.28f / 360.0f);
// uint16_t x[360], y[360];
float x[360], y[360];


// char * commandsForGnuplot[] = {"set title \"TITLEEEEE\"", "plot 'data.temp'"};
char * commandsForGnuplot[] = {"set title \"TITLEEEEE\"", "set xrange [-2:2]", "set yrange [-2:2]", "plot 'data.temp'"};
#define SERVER "192.168.0.102"
#define BUFLEN 512  //Max length of buffer
#define PORT 1235   //The port on which to send data
struct sockaddr_in si_other;
int sock, i, slen;
// char message[BUFLEN]= {'0', '1', '2','3', '4', '5', '6', '7', '8', '9', '\n'};
char message[BUFLEN];


/* Functions prototypes */
void *do_smth_periodically(void *data);
uint16_t calc_checksum(uint8_t *ptr);
int serial_port_read(char *read_buffer, size_t max_chars_to_read);
void DrawDistanceMap(void);

void DrawDistanceMap(void)
{
	// UP TO 3500 i distance = 3.5m
	int i;
	int valid_data = 0;
	int ret_bytes;
	int var, s;
	radians = 0;
	static int nr_saves = 0;
	static int cnt = 0;
	// setvbuf(temp, buf, _IOFBF, sizeof(uint16_t)* 1024);
	// printf("DrawDistanceMap called...\n");
	// if(nr_saves < 500) {
	s = pthread_mutex_lock(&mtx);
	if (s != 0)
		printf("cannot pthread_mutex_lock... \n");

		s = fseek(temp,  0,  SEEK_SET);
		// printf("lseek retuned %d %s \n", s, strerror(errno));

		for (deg = 0; deg < 360; deg++) {
			if (Distance[deg] < 30000 && Distance[deg] > 0) {
			//if (Distance[deg] > 0) {
				valid_data = 1;


				displayDistance = (float)Distance[deg] / 1000.0f /*  / 25.0f */;
				// x[deg] = (cos(M_PI_2-radians) * displayDistance) + 120;
				// y[deg] = (sin(M_PI_2-radians) * displayDistance) + 120;
				x[deg] = (float)displayDistance * (float)sin((float)deg * (3.14f/180.0f));
				y[deg] = (float)displayDistance* (float)cos((float)deg * (3.14f/180.0f));

				// w[deg] = (cos(M_PI_2-radians) * displayDistance) ;
				// z[deg] = (sin(M_PI_2-radians) * displayDistance) ;

				// x[deg] = cos(displayDistance) ;
				// y[deg] = sin(displayDistance);
				//fprintf(temp, "deg = %f Distance[deg]  = %f\t x[deg] = %f \t y[deg] = %f \n " \
				//,(float)deg , (float)Distance[deg] , (float)x[deg], (float)y[deg]);

				// fprintf(temp, "%f \t %f \n " , (float)deg, (float)Distance[deg]);

				fprintf(temp, "%f \t %f   \n " , (float)x[deg], (float)y[deg]);

				// fprintf(temp, "%f %f %f\n", (float)6.6, (float)7.7, (float) Distance[deg]); //Write the data to a temporary file
				// fprintf(temp, "%d \t %d\n", deg,  Distance[deg]); //Write the data to a temporary file
				//printf("%d \t %d\n", deg,  Distance[deg]); //Write the data to a temporary file

				//var = (nr_saves * 360) + deg;
				// printf("var = %d\n", var);
				// cnt+=2;
				// if(cnt < sizeof(buffer_data) - 2) {
					// // buffer_data[cnt] = deg;
					// buffer_data[cnt+1] = Distance[deg];
					// buffer_data[cnt] = deg;
				// }


				// fflush(temp);
			}

			radians += RAD;

		}
		s = pthread_mutex_unlock(&mtx);
		if (s != 0)
			printf("cannot pthread_mutex_unlock... \n");
	// }
	// else
		// printf("Stop the flood\n");

	// fflush(stdout);

	if (valid_data) {
		nr_saves++;
	}
		// ret_bytes = fwrite(Distance, sizeof(uint16_t), 360,temp);
		// if(ret_bytes != 360)
			// printf("wrong nr of bytes written...\n");
	// }
	// if (valid_data) {
		// for (i=0; i < 360; i++) {
			// fprintf(temp, "%f %f\n", (float)x[i], (float)y[i]); //Write the data to a temporary file
		// }
		// fprintf(gnuplotPipe, "%s \n", commandsForGnuplot[1]); //Send commands to gnuplot one by one.
	// }


}
uint16_t calc_checksum(uint8_t *ptr)
{
	unsigned char i;
	unsigned  int data[10];
	unsigned  int checksum;
	unsigned int chk32;
	uint8_t * packagePointer = ptr;
	// group the data by word, little-endian
	for (i = 0; i < 10; i++) {
		data[i] = packagePointer[2*i] | (((unsigned  int)packagePointer[2*i+1]) << 8);
		// printf("\ndata[%d] = 0x%08x", i, data[i]);
	}

	// compute the checksum on 32 bits
	chk32 = 0;
	for (i = 0; i < 10; i++) {
			chk32 = (chk32 << 1) + data[i];
	}
	// printf("\nchk32 = 0x%x", chk32);

	 // return a value wrapped around on 15bits, and truncated to still fit into 15 bits
	 checksum = (chk32 & 0x7FFF) + ( chk32 >> 15 ); // wrap around to fit into 15 bits
	 checksum = checksum & 0x7FFF; // truncate to 15 bits
	// printf("\n checksum = %x\n", (int)checksum);
	return checksum;
}
int ParsePackage(uint8_t * packagePointer)
{
	int ret = 0;
	uint16_t i;
	uint16_t Index;
	uint8_t Speed;
	uint8_t InvalidFlag[4];
	uint8_t WarningFlag[4];
	uint16_t Checksum, ChecksumCalculated;

	// for (i=0 ; i< 22 ; i++) {
		// printf("packagePointer[%i] = 0x%x\n", i, packagePointer[i]);
	// }
	Checksum = ((uint16_t)packagePointer[21] << 8) | packagePointer[20];
	ChecksumCalculated = calc_checksum(packagePointer);
	if (packagePointer[0] != 0xFA) {
		// BadReadings += 4;
		// printf("Bad start byte... ignore packet \n");
		return -1;
	}
	if (Checksum != ChecksumCalculated) {
		// BadReadings += 4;
		// printf("Bad checksum... ignore packet \n");
		return -1;
	}


	Index = (packagePointer[1] - 0xA0) * 4;
	// printf("Index = %d \n", Index);
	Speed = ((uint16_t)packagePointer[3] << 8) | packagePointer[2];
	InvalidFlag[0] = (packagePointer[5] & 0x80) >> 7;
	InvalidFlag[1] = (packagePointer[9] & 0x80) >> 7;
	InvalidFlag[2] = (packagePointer[13] & 0x80) >> 7;
	InvalidFlag[3] = (packagePointer[17] & 0x80) >> 7;
	WarningFlag[0] = (packagePointer[5] & 0x40) >> 6;
	WarningFlag[1] = (packagePointer[9] & 0x40) >> 6;
	WarningFlag[2] = (packagePointer[13] & 0x40) >> 6;
	WarningFlag[3] = (packagePointer[17] & 0x40) >> 6;

	if (Index == 0) {
		// printf("Zero intdex \n");
		AnglesCovered = 0;
		for (i = 0; i < 360; i++) {
			if (Distance[i] > 0) AnglesCovered++;
		}

		GoodReadings = 0;
		BadReadings = 0;
	}

	for (i = 0; i < 4; i++) {
		if (!InvalidFlag[i])
		{
			// printf("Valid data \n");
			Distance[Index+i] = packagePointer[4+(i*4)] | ((uint16_t)(packagePointer[5+(i*4)] & 0x3F) << 8);
			// fprintf(fp, "Distance[%d] = %02x \n", i, Distance[i] );
			if( Distance[Index+i]  > 0)
				//printf("Distance[%d] = %08x \n", i, Distance[Index+i] );
			GoodReadings++;
			//fflush(stdout);
		} else {
			// printf("Invalid flag ... \n");
			Distance[Index+i] = 0;
			BadReadings++;
		}
	}
	if(Index == 356) {
		DrawDistanceMap();
	}

}
int serial_port_read(char *read_buffer, size_t max_chars_to_read)
{
	int chars_read = read(fd_robo_LIDAR, read_buffer, max_chars_to_read);

	return chars_read;
}
void intHandler(int dummy) {
    int i;
    printf("Catched CTRL-C\n");
    pthread_cancel(thread);
		close(sock);
	// for (i=0 ; i < 65536/2 ; i++)
		// // // printf("response[%d] = 0x%x\n", i, response[i]);
		// // fprintf(temp, "Distance[%d] = %x\n", i, buffer_data[i]);
		// fprintf(temp, "Deg = %d \t Distance= %d\n", buffer_data[i],  buffer_data[i+1]);
	// fflush(stdout);
}

void *do_smth_periodically(void *data)
{
  int interval = *(int *)data;
  int s;
  int i;

  for (;;) {
	// printf("thread func do_smth_periodically called ...\n ");
	s = pthread_mutex_lock(&mtx);
	if (s != 0)
		printf("cannot pthread_mutex_lock... \n");
	// for(i=1 ; i< 4 ; i++)
		// fprintf(gnuplotPipe, "%s \n", commandsForGnuplot[i]); //Send commands to gnuplot one by one.
		printf("Waiting for message...\n");
		fgets( message, sizeof(message), stdin );   // safe
	 //send the message
	 if (sendto(sock, message, strlen(message) , 0 , (struct sockaddr *) &si_other, slen)==-1)
	 {
			 printf("sendto() error ... \n");
	 }

	s = pthread_mutex_unlock(&mtx);
	if (s != 0)
		printf("cannot pthread_mutex_unlock... \n");
	usleep(interval);
  }
}


void main (void )
{

	int nr_bytes_read 		= 0;
	int nr_bytes_read_temp 	= 0;
	int start_saving 	= 0;
	int i;

	int interval = 5000;

	char buff_single;
	char buff_packet[22];
	signal(SIGINT, intHandler);
	slen=sizeof(si_other);

	if ( (sock=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
	{
			printf("socket faikled\n");
	}

	memset((char *) &si_other, 0, sizeof(si_other));
	si_other.sin_family = AF_INET;
	si_other.sin_port = htons(PORT);

	if (inet_aton(SERVER , &si_other.sin_addr) == 0)
	{
			fprintf(stderr, "inet_aton() failed\n");
			exit(1);
	}

	// memcpy(message, "0123456789", 10);
	temp = fopen("data.temp", "w+");
	// gnuplotPipe = popen ("gnuplot -persistent", "w");

	fd_robo_LIDAR = open(PORT_NAME, O_RDWR);

	if (fd_robo_LIDAR < 0)
		printf("Cannot open Neato serial interface\n");
	else
		printf("Interface opened \n ");

	fflush(stdout);

	// fprintf(gnuplotPipe, "%s \n", commandsForGnuplot[0]); //Send commands to gnuplot one by one
	fflush(gnuplotPipe);

	fp = fopen("dump.txt", "w+");
	bzero(&SerialPortSettings, sizeof(SerialPortSettings)); /* clear struct for new port settings */

	SerialPortSettings.c_cflag &= ~PARENB;   // No Parity
	SerialPortSettings.c_cflag &= ~CSIZE; /* Clears the Mask       */
	SerialPortSettings.c_cflag |=  CS8;   /* Set the data bits = 8 */
	SerialPortSettings.c_iflag = IGNPAR;

	SerialPortSettings.c_cflag |= CREAD | CLOCAL; // turn on receiver
	SerialPortSettings.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	cfsetispeed(&SerialPortSettings,B115200);
	cfsetospeed(&SerialPortSettings,B115200);

	/* now clean the LIDAR and activate the settings for the port */
	tcflush(fd_robo_LIDAR, TCIFLUSH);

	tcsetattr(fd_robo_LIDAR,TCSANOW,&SerialPortSettings);

	// pthread_create(&thread, NULL, do_smth_periodically, &interval);
	// fprintf(gnuplotPipe, "%s \n", commandsForGnuplot[0]);


/*
	while (1)
	{
		printf("Waiting for message...\n");
		fgets( message, sizeof(message), stdin );   // safe
	 //send the message
	 if (sendto(sock, message, strlen(message) , 0 , (struct sockaddr *) &si_other, slen)==-1)
	 {
			 printf("sendto() error ... \n");
	 } else {
		 printf("Cica s-a trimis ... ceva...\n" );
	 }

 }*/
	while(1)
	{
		// printf("looping...\n");
		// if (serial_port_read(buffer, (size_t)22) < 0) {
		nr_bytes_read = serial_port_read(buffer, (size_t)1);

		// if (serial_port_read(buffer, (size_t)22) < 0) {
		if (nr_bytes_read <= 0) {
			//printf("Cannot read from serial port\n");
			continue;
		} else {
			printf("nr_bytes_read = %d\n", nr_bytes_read);
			for (i=0 ; i<nr_bytes_read ; i++) {
				// printf("Byte[%d] = %x \n", i, buffer[i] & 0xFF);
				fprintf(fp, "Byte[%d] = %02x \n", i, buffer[i] & 0xFF);
				// fprintf(fp, "Byte[%d] = %02x \n", i, buff_single & 0xFF);
				//sprintf( &response[i], "%c", buffer[i] & 0xFF );
				fflush(fp);
			}
			// sync on start byte
			if ((buffer[0] & 0xFF) == 0xFA)
				start_saving = 1;

			if (start_saving) {
				buff_packet[nr_bytes_read_temp] = buffer[0] & 0xFF;
				nr_bytes_read_temp++;
				if(nr_bytes_read_temp == 22) {
					start_saving = 0;
					nr_bytes_read_temp = 0;
					ParsePackage(buff_packet);
				}
			}

		}
		// DrawDistanceMap();

	}
}

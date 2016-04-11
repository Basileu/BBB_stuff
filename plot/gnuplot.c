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
#include <math.h>

#define NR_POINTS 8
float arr_deg [8] =
{0,14,26,36,46,52,62,72};
float arr_dist [8] =
// {2,2.1,2.2,2.5,2.8,3.2,2.9,2.7};
{2.8,3.1,2.9,2.7,2.5,2.4,2.2,2};
float x[8], y[8];
char * commandsForGnuplot[] =
{"set title \"TITLEEEEE\"", "set xrange [-4:4]", "set yrange [-4:4]", "plot 'points.temp'"};

void main (void)
{
  int i;
  FILE * gnuplotPipe;
  FILE * temp;

  gnuplotPipe = popen ("gnuplot -persistent", "w");

  temp = fopen("points.temp", "w");
  for (i=0 ; i<NR_POINTS ; i++)
  {
    x[i] = (float)arr_dist[i] * (float)sin((float)arr_deg[i] * (3.14f/180.0f));
    y[i] = (float)arr_dist[i] * (float)cos((float)arr_deg[i] * (3.14f/180.0f));
    fprintf(temp, "%f \t %f\n", x[i], y[i]);
  }


  for(i=1 ; i< 4 ; i++)
	 fprintf(gnuplotPipe, "%s \n", commandsForGnuplot[i]); //Send commands to gnuplot one by one.


}

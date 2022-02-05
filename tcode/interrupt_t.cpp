#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>

#include<sys/ipc.h>
#include<sys/shm.h>

#include "motor_drive.h"
#include "localize.h"
#include <poll.h>
#include <pthread.h>

#define BUF_SIZE 1024
#define SHM_KEY 0x1234

struct shmseg {
   int cnt;
   int complete;
   char buf[BUF_SIZE];
};

int fill_buffer(char * bufptr, int size) {
   static char ch = 'A';
   int filled_count;
   
   //printf("size is %d\n", size);
   memset(bufptr, ch, size - 1);
   bufptr[size-1] = '\0';
   if (ch > 122)
   ch = 65;
   if ( (ch >= 65) && (ch <= 122) ) {
      if ( (ch >= 91) && (ch <= 96) ) {
         ch = 65;
      }
   }
   filled_count = strlen(bufptr);
   
   ch++;
   return filled_count;
}


void* counter(void*){
	char str[256];
	int fd;
	struct pollfd pfd;
	char buf[8];
	int count = 0;

	int shmid;
	struct shmseg *shmp;
	char *bufptr;
	int spaceavailable;
	shmid = shmget(SHM_KEY, sizeof(struct shmseg), 0644|IPC_CREAT);
	
	if (shmid == -1) {
	  perror("Shared memory");
	  return 1;
	}

	// Attach to the segment to get a pointer to it.
	shmp = shmat(shmid, NULL, 0);
	if (shmp == (void *) -1) {
	  perror("Shared memory attach");
	  return 1;
	}


	sprintf(str, "/sys/class/gpio/gpio%d/value", LENCODER);
	
	if ((fd = open(str, O_RDONLY)) < 0){
		fprintf(stderr, "Failed to open gpio value for monitor \n");
		printf("ERROR: %d \n", errno);
		printf("/sys/class/gpio/gpio%d/value \n", LENCODER);
		
	}

	while(1){



		pfd.fd = fd;
		pfd.events = POLLPRI;

		lseek(fd, 0, SEEK_SET);
		read(fd, buf, sizeof buf);
		poll(&pfd, 1, -1);


		lseek(fd, 0, SEEK_SET);
		read(fd, buf, sizeof buf);
		count ++;
		printf("COUNT: %d \n", count);

		bufptr = shmp->buf;
   		spaceavailable = BUF_SIZE;
		shmp->cnt = fill_buffer(bufptr, spaceavailable);
		shmp->complete = 0;
		printf("Writing Process: Shared Memory Write: Wrote %d bytes\n", shmp->cnt);
		bufptr = shmp->buf;
		spaceavailable = BUF_SIZE;
		shmp->complete = 1;

		if (shmdt(shmp) == -1) {
			perror("shmdt");
			return 1;
		}

		if (shmctl(shmid, IPC_RMID, 0) == -1) {
			perror("shmctl");
			return 1;
		}


		if(GPIORead(STOP)) break;

	}
	return NULL;

}


int main(){
	pthread_t tid;

	



	if (-1 == GPIOExport(STOP))
		return 1;

	if (-1 == GPIODirection(STOP, IN))
	return 2;


	if (-1 == GPIOExport(LENCODER))
		return 1;


	if (-1 == GPIODirection(LENCODER, IN))
		return 2;

	if (-1 == GPIOEdge(LENCODER, RISING))
		return 2;


	pthread_create(&tid, NULL, counter, NULL);

	while(1){
		if(GPIORead(STOP)) break;
	}


	pthread_join(tid, NULL);

	if (-1 == GPIOUnexport(LENCODER))
		return 1;
	if (-1 == GPIOUnexport(STOP))
		return 1;

	printf("No problems today \n");
	return 0;

}
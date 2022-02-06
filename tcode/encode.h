#ifndef ENCODE
#define ENCODE

#define LENCODER 23
#define RENCODER 24

#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>

#include <poll.h>
#include <string.h>


void* poll_l_encoder(void*);
void* poll_r_encoder(void*);



#endif
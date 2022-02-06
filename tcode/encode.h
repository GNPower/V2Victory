#ifndef ENCODE
#define ENCODE

#define LENCODER 23
#define RENCODER 24

volatile int l_encoder;
volatile int r_encoder;

void* poll_l_encoder(void*);
void* poll_r_encoder(void*);



#endif
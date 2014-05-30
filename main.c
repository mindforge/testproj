/*
 * Main.c
 * 
 * Version 0.0.1
 * 
 * 05.24.2014
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 * 
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 *   copyright notice, this list of conditions and the following disclaimer
 *   in the documentation and/or other materials provided with the
 *   distribution.
 * * Neither the name of the  nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */

#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <stddef.h>
#include <string.h>

/* Macros */
#define ERROR_CHECKING
#define ERROR_CONDITION printf("Error: line %d",__LINE__);

/* Global Structs */
struct termios serialConfig;

/* Global Pointers */

FILE *pSerialInterface = NULL;		/* pointer for fopen usage */
FILE *pDataTargetFile = NULL;		/* pointer to target file for data */
char *pBufTX = NULL;			/* Outbound message pointer */
char *pBufRX = NULL;			/* Inbound message pointer */		

/* Global Variables */
int serialDescriptor;			/* file descripter for open */

/* Global Buffers */
char bufTX[128];			/* TX message buffer */
char bufRX[128];			/* RX message buffer */	

int main(int argc, char *argv[]){

printf("\n");

pBufTX=&bufTX[0];
	
serialDescriptor = open(argv[1], O_RDWR | O_NOCTTY | O_NDELAY);

#ifdef ERROR_CHECKING

if (serialDescriptor == -1){
	ERROR_CONDITION
	return -1;
	}
	
printf("%s successfully opened\n\n", argv[1]); 

if(!isatty(serialDescriptor)){
	ERROR_CONDITION
	return -1;
	}

if(tcgetattr(serialDescriptor, &serialConfig)<0){
	ERROR_CONDITION
	return -1;
	}
#endif

serialConfig.c_iflag &= ~(IGNBRK | BRKINT |
			ICRNL |INLCR | PARMRK |
			INPCK | ISTRIP | IXON);
serialConfig.c_oflag = 0;
serialConfig.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
serialConfig.c_cflag &= ~(CSIZE | PARENB);
serialConfig.c_cflag |= CS8;
serialConfig.c_cc[VMIN]  = 1;
serialConfig.c_cc[VTIME] = 0;

if(cfsetispeed(&serialConfig,B115200)<0||cfsetospeed(&serialConfig,B115200)<0){
	ERROR_CONDITION
	return -1;}
	
if(tcsetattr(serialDescriptor,TCSAFLUSH,&serialConfig)<0){
	ERROR_CONDITION
	return -1;}

strcpy(bufTX, "This is a test string\0");

write(serialDescriptor, pBufTX, strlen(bufTX));

return 0;
}


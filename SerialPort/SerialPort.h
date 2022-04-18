#ifndef _SERIAL_PORT_H
#define _SERIAL_PORT_H

int OpenPort(char *port);
int SetPort(int fd, int nSpeed, int nBits, char nEvent, int nStop);

#endif

#ifndef _SERIAL_PORT_H
#define _SERIAL_PORT_H

struct SerialPortRx{
  char start_flag;
  float yaw;
  float pitch;
  float roll;
  char end_flag;
}__attribute__ ((packed))SerialPortRx;

struct SerialPortTx{
    char start_flag;
    short  x_offset;
    short  y_offset;
    char  shooting;
    char end_flag;
}__attribute__ ((packed))SerialPortTx;

int OpenPort(char *port);
int SetPort(int fd, int nSpeed, int nBits, char nEvent, int nStop);
void SerialPortRxHandle(int srial_port);
void SerialPortTxHandle(int srial_port);

#endif

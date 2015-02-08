#ifndef _LIB_SERIAL_H
#define _LIB_SERIAL_H

#ifdef _cplusplus
extern "C"
{
#endif

	struct serial * serial_create();
	int serial_free(struct serial * );
	int serial_open(struct serial *);
	int serial_close(struct serial *);
    int serial_blocking_read(struct serial *ser , char * data  , int len , int timeouts);
	int serial_blocking_write(struct serial * , const char *data , int len);
	int serial_nonblocking_read();
	int serial_nonblocking_write();

#ifdef _cplusplus
}
#endif

#endif

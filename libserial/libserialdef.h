#ifndef _LIB_SERIAL_DEF_H
#define _LIB_SERIAL_DEF_H

#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#endif
#include <stdio.h>
#include <stdlib.h>


#ifndef _WIN32
#define INVALID_HANDLE_VALUE -1
#endif

typedef enum _databits databits;
typedef enum _parity parity;
typedef enum _stopbits stopbits;
typedef struct _serialConf serialConf;

struct serial
{
	char			port[16];
#ifdef _WIN32
	HANDLE			fd;
	COMMTIMEOUTS	timeouts;
	OVERLAPPED		write_ovl;
	OVERLAPPED		read_ovl;
	OVERLAPPED		wait_ovl;
	DWORD			events;
#else
	int				fd;
#endif
	serialConf		*sconfig;
};

enum _databits
{
	DATABITS_SIX=6,
	DATABITS_SEVEN=7,
	DATABITS_EIGHT=8
};

enum _parity
{
	_PARITY_NONE='N',
	_PARITY_EVEN='E',
	_PARITY_ODD='O'
};

enum _stopbits
{
	STOPBITS_ONE=0,
	STOPBITS_ONEHALF=1,
	STOPBITS_TWO=2
};

struct _serialConf
{
	int			    _baud;
	databits		_bytes;
	parity			_parity;
	stopbits		_stopbits;
};

#endif

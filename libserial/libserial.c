#include "libserial.h"
#include "libserialdef.h"

static int setSerialState(int fd,int bandrate,int databits,int stopbits,int parity);

#define INIT_OVERLAPPED(ovl) do { \
	memset(&ovl, 0, sizeof(ovl)); \
	ovl.hEvent = INVALID_HANDLE_VALUE; \
	if ((ovl.hEvent = CreateEvent(NULL, TRUE, TRUE, NULL)) \
	== INVALID_HANDLE_VALUE) { \\
	} \
} while (0)

struct serial * serial_create()
{
	struct serial * ser = NULL;

	ser = (struct serial *)malloc(sizeof(struct serial));
	if(!ser)
		return NULL;
	memset(ser , 0 ,sizeof(struct serial));
#ifdef _WIN32
    if(NULL == (ser->read_ovl.hEvent = CreateEvent(NULL , TRUE , TRUE , NULL)))
		return NULL;
	if(NULL == (ser->write_ovl.hEvent = CreateEvent(NULL , TRUE , TRUE , NULL)))
		return NULL;
	if(NULL == (ser->wait_ovl.hEvent == CreateEvent(NULL , TRUE , TRUE ,NULL)));
		return NULL;
#endif
	ser->sconfig = (serialConf *)malloc(sizeof(serialConf));
	memset(ser->sconfig , 0 ,sizeof(serialConf));
	//default 9600 8 N 1
	ser->sconfig->_baud = 9600;
	ser->sconfig->_bytes = DATABITS_EIGHT;
	ser->sconfig->_parity = _PARITY_NONE;
	ser->sconfig->_stopbits = STOPBITS_ONE;
	ser->fd = INVALID_HANDLE_VALUE;
}

int serial_free(struct serial *ser)
{
    if(!ser)
        return 0;
    if(ser->fd)
        close(ser->fd);
    free(ser);
    return 1;
}


int serial_open(struct serial *ser)
{
#ifdef _WIN32
	 DCB	dcb;
	 int s_baud , s_bytes , s_stopbit;
	 char s_parity;
	 char port[16]={0} , dcbstr[32]={0};
	 sprintf(port , "\\\\.\\%s",ser->port);
	 ser->fd = CreateFile( (LPCWSTR)port,
							GENERIC_READ | GENERIC_WRITE,	// read/write types
							 0,								// comm devices must be opened with exclusive access
							NULL,							// no security attributes
							OPEN_EXISTING,					// comm devices must use OPEN_EXISTING
							FILE_FLAG_OVERLAPPED,
							NULL);
	 if(ser->fd == INVALID_HANDLE_VALUE)
		 return -1;
	 if(!GetCommState(ser->fd , &dcb))
	 {
		 CloseHandle(ser->fd);
		 return -1;
	 }
	 sprintf(dcbstr,"baud=%d parity=%c data=%d stop=%s",
				 ser->sconfig->_baud,
				ser->sconfig->_parity,
				ser->sconfig->_bytes,
				ser->sconfig->_stopbits);
	 BuildCommDCB(&dcbstr , &dcb);
	 if( !SetCommState(ser->fd , &dcb) )
	 {
		 CloseHandle(ser->fd);
		 return -1;
	 }
	 memset(&ser->timeouts , 0 ,sizeof(COMMTIMEOUTS));
	 ser->timeouts.ReadIntervalTimeout=1000;
	 ser->timeouts.ReadTotalTimeoutConstant=1000;
	 ser->timeouts.ReadTotalTimeoutMultiplier=1000;
	 ser->timeouts.WriteTotalTimeoutConstant=1000;
	 ser->timeouts.WriteTotalTimeoutMultiplier=1000;
	 SetCommTimeouts(ser->fd , &ser->timeouts);

	 //set com mask
	 SetCommMask(ser->fd , EV_RXCHAR|EV_ERR);
	 PurgeComm(ser->fd, PURGE_RXCLEAR | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_TXABORT);
#else
	ser->fd = open(ser->port,O_RDWR|O_NOCTTY|O_NDELAY);
	if(ser->fd == INVALID_HANDLE_VALUE )
		return -1;
	if(0 == isatty(STDIN_FILENO) == 0)
	{
		return -1;
	}
	setSerialState(ser->fd , ser->sconfig->_baud , ser->sconfig->_bytes , ser->sconfig->_stopbits , ser->sconfig->_parity);
#endif
	return 0;
}

int serial_close(struct serial *ser)
{
    return 1;
}

int serial_blocking_read(struct serial *ser , char * data  , int len , int timeouts)
{
#ifdef _WIN32
	DWORD	bytes_read;
	ser->timeouts.ReadIntervalTimeout = 0;
	ser->timeouts.ReadTotalTimeoutConstant = timeouts;
	if(0 == SetCommTimeouts(ser->fd , &ser->timeouts))
		return -1;
	if(0 == ReadFile(ser->fd , data , len , NULL , &ser->read_ovl ))
	{
		if(GetLastError() == ERROR_IO_PENDING)
		{
			GetOverlappedResult(ser->fd , &ser->read_ovl , &bytes_read , TRUE);
		}
		else
			return -1;
	}
	else
		bytes_read = len;
	return bytes_read;
#else
	fd_set fds;
	struct timeval start , now , end , delay;
	size_t byte_read = 0 , result;
	if(timeouts)
	{
		gettimeoftoday(&start , NULL);
		delay.tv_sec = timeouts/1000;
		delay.tv_usec = 1000*(timeouts%1000);
		timeradd(&start, &delay, &end);
	}
	while(byte_read < len)
	{
		int ret;
		FD_ZERO(&fds);
		FD_SET(ser->fd , &fds);
		if (timeouts) {
			gettimeofday(&now, NULL);
			if (timercmp(&now, &end, '>timercmp'))
				/* Timeout has expired. */
					return (byte_read);
			timersub(&end, &now, &delay);
		}
		ret =select(ser->fd+1 , &fds , NULL ,NULL , &delay);
		if(ret < 0)
		{
			if(errno == EINTR)
				continue;
			else
				return -1;
		}
		else if(ret == 0)
			return byte_read;

		result = read(ser->fd , data+byte_read , len - byte_read);
		if(result < 0)
		{
			if(errno == EAGAIN)
				continue;
			else
				return byte_read;
		}
		byte_read += result;
		if(byte_read == len)
			return byte_read;
	}
	return byte_read;
#endif
}

int serial_blocking_write(struct serial * , const char *data , int len);
int serial_nonblocking_read();
int serial_nonblocking_write();

static int setSerialState(int fd,int bandrate,int databits,int stopbits,int parity)
{
#ifdef _WIN32
	return 0;
#else
	struct termios options;
	if  ( tcgetattr( fd,&options)  !=  0) {
		perror("SetupSerial 1");
		return(0);
	}
	options.c_cflag &= ~CSIZE;
	options.c_lflag  &= ~(ICANON | ECHO | ECHOE | ISIG);  /*Input*/
	options.c_oflag  &= ~OPOST;   /*Output*/

	switch(bandrate)
	{
	case 2400:
		cfsetispeed(&options, B2400);
		cfsetospeed(&options, B2400);
		break;
	case 4800:
		cfsetispeed(&options, B4800);
		cfsetospeed(&options, B4800);
		break;
	case 9600:
		cfsetispeed(&options, B9600);
		cfsetospeed(&options, B9600);
		break;
	case 115200:
		cfsetispeed(&options, B115200);
		cfsetospeed(&options, B115200);
		break;
	default:
		cfsetispeed(&options, B9600);
		cfsetospeed(&options, B9600);
		break;

	}
	switch (databits) /*设置数据位数*/
	{
	case 7:
		options.c_cflag |= CS7;
		break;
	case 8:
		options.c_cflag |= CS8;
		break;
	default:
		fprintf(stderr,"Unsupported data size/n"); return (0);
	}
	switch (parity)
	{
	case 'n':
	case 'N':
		options.c_cflag &= ~PARENB;   /* Clear parity enable */
		options.c_iflag &= ~INPCK;     /* Enable parity checking */
		break;
	case 'o':
	case 'O':
		options.c_cflag |= (PARODD | PARENB); /* 设置为奇效验*/
		options.c_iflag |= INPCK;             /* Disnable parity checking */
		break;
	case 'e':
	case 'E':
		options.c_cflag |= PARENB;     /* Enable parity */
		options.c_cflag &= ~PARODD;   /* 转换为偶效验*/
		options.c_iflag |= INPCK;       /* Disnable parity checking */
		break;
	case 'S':
	case 's':  /*as no parity*/
		options.c_cflag &= ~PARENB;
		options.c_cflag &= ~CSTOPB;break;
	default:
		fprintf(stderr,"Unsupported parity/n");
		return (0);
	}
	/* 设置停止位*/
	switch (stopbits)
	{
	case 1:
		options.c_cflag &= ~CSTOPB;
		break;
	case 2:
		options.c_cflag |= CSTOPB;
		break;
	default:
		fprintf(stderr,"Unsupported stop bits/n");
		return (0);
	}
	/* Set input parity option */
	if (parity != 'n')
		options.c_iflag |= INPCK;
	tcflush(fd,TCIFLUSH);
	options.c_cc[VTIME] = 0; /* 设置超时15 seconds*/
	options.c_cc[VMIN] = 13; /* define the minimum bytes data to be readed*/
	if (tcsetattr(fd,TCSANOW,&options) != 0)
	{
		perror("SetupSerial 3");
		return (0);
	}
#endif
	return (1);
}

#include <iostream>

//#include <pthread.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <string>
#include <thread>

#include "uart.h"

//#define _DEBUG_ENABLE_
#define CANONICAL_MODE

UartDriverLite::UartDriverLite() : _open_success(false) { }

UartDriverLite::~UartDriverLite(void) {
    Close();
}

void UartDriverLite::Close() {
    if (_baud_rate > 0) {
        /* restore the old port settings */
        tcsetattr(uart_fd,TCSANOW,&oldtio);
    }
    close(uart_fd);
}

void UartDriverLite::Open(const char *device) {
    _device = device;
    _baud_rate = -1;    // this is a file-mode. (not device mode);

    // Open file.
    uart_fd = open(_device, O_RDWR | O_NOCTTY );
    if (uart_fd <0) {
        return;
    }

    _open_success = true;
}

void UartDriverLite::Open(const char *device, int baud_rate) {
    //int fd,c, res;
    char buf[255];

    _device = device;
    _baud_rate = baud_rate;

    //uart_fd = open(_device, O_RDWR | O_NOCTTY | O_NONBLOCK );
    uart_fd = open(_device, O_RDWR | O_NOCTTY );
    if (uart_fd <0) {
        //perror(device_file);
        //UartMessageDisplayCallback("[ERROR] Can't open the device file.\n");
        std::cerr << "[ERROR] UART terminal is not opened!" << std::endl;
        return;
    }

    tcgetattr(uart_fd, &oldtio); /* save current serial port settings */
    bzero(&newtio, sizeof(newtio)); /* clear struct for new port settings */

    /*
    BAUDRATE: . cfsetispeed()  cfsetospeed() 
    CRTSCTS : 
    CS8     : 8N1 (8bit, no parity, 1 stopbit)
    CLOCAL  : Local connection. 
    CREAD   : 
    */
    //newtio.c_cflag = BAUDRATE | CRTSCTS | CS8 | CLOCAL | CREAD;
    if (baud_rate == 115200) {
        newtio.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
    }
    else if (baud_rate == 38400) {
        newtio.c_cflag = B38400 | CS8 | CLOCAL | CREAD;
    }
    else {
        newtio.c_cflag = B9600 | CS8 | CLOCAL | CREAD;
    }

    /*
    IGNPAR   : Parity 
    ICRNL    : CR , NL 
               CR 
    otherwise make device raw (no other input processing)
    */
    newtio.c_iflag = IGNPAR | ICRNL;

    /*
    Raw output.
    */
    newtio.c_oflag = 0;

    /*
    ICANON   : canonical 
    disable all echo functionality, and don't send signals to calling program
    */
#if defined(CANONICAL_MODE)
    newtio.c_lflag = ICANON;
#else
    newtio.c_lflag &= ~(ICANON);
    newtio.c_lflag &= ~(ECHO | ECHOE);
    newtio.c_cc[VMIN] = 1;
    newtio.c_cc[VTIME] = 10;
#endif

    /*
     <termios.h> 
    */
    newtio.c_cc[VINTR]    = 0;     /* Ctrl-c */
    newtio.c_cc[VQUIT]    = 0;     /* Ctrl-\ */
    newtio.c_cc[VERASE]   = 0;     /* del */
    newtio.c_cc[VKILL]    = 0;     /* @ */
    newtio.c_cc[VEOF]     = 4;     /* Ctrl-d */
    newtio.c_cc[VTIME]    = 0;     /* inter-character timer unused */
    newtio.c_cc[VMIN]     = 1;     /* blocking read until 1 character arrives */
    newtio.c_cc[VSWTC]    = 0;     /* '\0' */
    newtio.c_cc[VSTART]   = 0;     /* Ctrl-q */
    newtio.c_cc[VSTOP]    = 0;     /* Ctrl-s */
    newtio.c_cc[VSUSP]    = 0;     /* Ctrl-z */
    newtio.c_cc[VEOL]     = 0;     /* '\0' */
    newtio.c_cc[VREPRINT] = 0;     /* Ctrl-r */
    newtio.c_cc[VDISCARD] = 0;     /* Ctrl-u */
    newtio.c_cc[VWERASE]  = 0;     /* Ctrl-w */
    newtio.c_cc[VLNEXT]   = 0;     /* Ctrl-v */
    newtio.c_cc[VEOL2]    = 0;     /* '\0' */

    /*
    이제 modem 라인을 초기화하고 포트 세팅을 마친다.
    */
    tcflush(uart_fd, TCIFLUSH);
    tcsetattr(uart_fd,TCSANOW,&newtio);

    /*
     * Create thread for rx/tx transmission.
     */
    int thr_id;
    //int status;

    //UartMessageDisplayCallback("[INFO] UART open is success.\n");

#if 0   // TODO: tx thread is blocked. [temp]
#if 0
    thr_id = pthread_create(&p_thread[0], NULL, tx_thread, (void*) &fd);
    if(thr_id < 0) {
        perror("thread create error: tx_thread\n");
        return -1;
    }
#else
    thr_id = pthread_create(&p_thread[0], NULL, tank_control_thread, (void*) &uart_fd);
    if(thr_id < 0) {
        perror("thread create error: tank_control_thread\n");
        return -1;
    }
#endif
#endif

    //thr_id = pthread_create(&p_thread[1], NULL, rx_thread, (void*) &uart_fd, UartMessageDisplayCallback);
    //if(thr_id < 0) {
    //    UartMessageDisplayCallback("thread create error: rx_thread\n");
    //    return -1;
    //}

    //p_thread[1] = std::thread(rx_thread_wrapper, this, UartMessageDisplayCallback);

    //pthread_join(p_thread[0], (void**) &status);
    //pthread_join(p_thread[1], (void**) &status);

    //printf("Program End.. Bye~\n");

    //
    ///* restore the old port settings */
    //tcsetattr(uart_fd,TCSANOW,&oldtio);

    //SendMessageUart("reset");

    _open_success = true;

    return;
}

void UartDriverLite::SendMessageUart(std::string message) {
    if (!_open_success) {
        std::cerr << "[ERROR] UART terminal is not opened!" << std::endl;
        return;
    }
    
    write(uart_fd, message.c_str(), message.size()+1);
}

int UartDriverLite::ReceiveMessageUart(std::string &message) {
    char buf[1024];

    if (!_open_success) {
        //std::cerr << "[ERROR] UART terminal is not opened!" << std::endl;
        return 0;
    }
    
    int res = read(uart_fd, buf, 1024);

    if (res >= 0) {
        buf[res] = 0;
        message = std::string(buf);
    }

    return res;
}

void UartDriverLite::SendByte(const char *data) {
    if (!_open_success) {
        std::cerr << "[ERROR] UART terminal is not opened!" << std::endl;
        return;
    }
    
    write(uart_fd, data, sizeof(char));
}

int  UartDriverLite::ReceiveByte(char *data, int size) {
    if (!_open_success) {
        //std::cerr << "[ERROR] UART terminal is not opened!" << std::endl;
        return 0;
    }

    int res = read(uart_fd, data, sizeof(char)*size);

#if defined(_DEBUG_ENABLE_)
    std::ios state(NULL);
    state.copyfmt(std::cout);
    std::cout << "RX Byte[0x" << std::hex << (*data & 0xFF) << "]\n";
    std::cout.copyfmt(state);
#endif

    return res;
}

int  UartDriverLite::ReceiveByte(char *data) {
    if (!_open_success) {
        //std::cerr << "[ERROR] UART terminal is not opened!" << std::endl;
        return 0;
    }

    int res = read(uart_fd, data, sizeof(char));

#if defined(_DEBUG_ENABLE_)
    std::ios state(NULL);
    state.copyfmt(std::cout);
    std::cout << "RX Byte[0x" << std::hex << (*data & 0xFF) << "]\n";
    std::cout.copyfmt(state);
#endif

    return 0;
}

bool UartDriverLite::isOpened(void) {
    return _open_success;
}

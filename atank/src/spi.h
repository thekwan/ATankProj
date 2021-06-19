#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

class SpiDriverLite {
public:
    SpiDriverLite();
    ~SpiDriverLite();

    void Open(const char *device, int speed_hz);
    //void Open(const char *device);
    void Close();

    //void SendMessageSpi(std::string message);
    //int  ReceiveMessageSpi(std::string &message);
    int  SendBytes(const char *tx_data, int n);
    int  ReceiveBytes(char *rx_data, int n);
    int  SendAndReceiveBytes(const char *tx_data, int tx_n, char *rx_data, int rx_n);
    bool isOpened(void);

private:
    typedef struct _spi_tool_t {
        int fd;
        uint8_t mode;
        uint8_t bits;
        uint32_t hz;
    } spi_tool_t;

    void spi_deinit();
    int spi_read(const char *tx, int tx_len, char *rx, int rx_len);
    int spi_write(const char *tx, int tx_len);

    spi_tool_t *_spi_tool;

    const char *_device;
    int _speed_hz;
    bool _open_success;
    int uart_fd;
    //std::thread  p_thread[2];
};

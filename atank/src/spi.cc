#include "spi.h"

SpiDriverLite::SpiDriverLite() {
    _open_success = false;
    _spi_tool = NULL;
}

SpiDriverLite::~SpiDriverLite() {
}

void SpiDriverLite::spi_deinit() {
    if (_spi_tool) {
        if (_spi_tool->fd > 0) {
            close(_spi_tool->fd);
        }
        free(_spi_tool);
    }
}

void SpiDriverLite::Open(const char *device, int speed_hz) {
    _device = device;
    _speed_hz = speed_hz;

    _spi_tool = (spi_tool_t*) malloc(sizeof(spi_tool_t));

    if (_spi_tool == NULL) {
        return;
    }

    _spi_tool->hz = _speed_hz;

    _spi_tool->xfer[0].tx_buf = 0;
    _spi_tool->xfer[0].rx_buf = 0;
    _spi_tool->xfer[0].len = 0; // Length of  command to write
    _spi_tool->xfer[0].cs_change = 0;
    _spi_tool->xfer[0].delay_usecs = 0,
    _spi_tool->xfer[0].speed_hz = _speed_hz,
    _spi_tool->xfer[0].bits_per_word = 8,

    _spi_tool->xfer[1].rx_buf = 0;
    _spi_tool->xfer[1].tx_buf = 0;
    _spi_tool->xfer[1].len = 0; // Length of Data to read
    _spi_tool->xfer[1].cs_change = 0;
    _spi_tool->xfer[1].delay_usecs = 0;
    _spi_tool->xfer[1].speed_hz = _speed_hz;
    _spi_tool->xfer[1].bits_per_word = 8;


    if ((_spi_tool->fd = open(_device, O_RDWR)) < 0) {
        spi_deinit();
        return;
    }

    if (ioctl(_spi_tool->fd, SPI_IOC_RD_MODE, &_spi_tool->mode) < 0) {
        spi_deinit();
        return;
    }

    /*
     * 가능한 mode. spidev.h에 선언되어 있음. 
     * #define SPI_CPHA        0x01
     * #define SPI_CPOL        0x02
     * 
     * CPOL : Clock Polarity  // 0: high active 1: low active 
     * CPHA : Clock Phase     // 0: non active -> active로 갈 때 샘플링
     *                        // 1: active -> non active로 갈 때 샘플링 
     * #define SPI_MODE_0      (0|0) 
     * #define SPI_MODE_1      (0|SPI_CPHA)
     * #define SPI_MODE_2      (SPI_CPOL|0)
     * #define SPI_MODE_3      (SPI_CPOL|SPI_CPHA)
     *
     * #define SPI_CS_HIGH     0x04
     * #define SPI_LSB_FIRST   0x08
     * #define SPI_3WIRE       0x10
     * #define SPI_LOOP        0x20
     * #define SPI_NO_CS       0x40
     * #define SPI_READY       0x80
     */

    _spi_tool->mode = SPI_MODE_0;

    if (ioctl(_spi_tool->fd, SPI_IOC_WR_MODE, &_spi_tool->mode) < 0) {
        spi_deinit();
        return;
    }

    if (ioctl(_spi_tool->fd, SPI_IOC_RD_BITS_PER_WORD, &_spi_tool->bits) < 0) {
        spi_deinit();
        return;
    }

    if (_spi_tool->bits != 8) {
        _spi_tool->bits = 8;
        if (ioctl(_spi_tool->fd, SPI_IOC_WR_BITS_PER_WORD, &_spi_tool->bits) < 0) {
            spi_deinit();
            return;
        }
    }

    if (ioctl(_spi_tool->fd, SPI_IOC_WR_MAX_SPEED_HZ, &_spi_tool->hz)<0) {
        spi_deinit();
        return;
    }

    _open_success = true;

    return;
}

void SpiDriverLite::Close() {
    spi_deinit();
    _open_success = false;
}

void SpiDriverLite::SendBytes(const char *tx_data, int n) {
    spi_write(tx_data, n);
}

int  SpiDriverLite::ReceiveBytes(char *rx_data, int n) {
    spi_read(NULL, 0, rx_data, n);
}

void SpiDriverLite::SendAndReceiveBytes(const char *tx_data, int tx_n, char *rx_data, int rx_n) {
    spi_read(tx_data, tx_n, rx_data, rx_n);
}

bool SpiDriverLite::isOpened(void) {
    return _open_success;
}

int SpiDriverLite::spi_read(const char *tx, int tx_len, char *rx, int rx_len) {
    int status = 0;
    if (!_spi_tool || !tx || !rx || _spi_tool->fd <= 0) {
        return 0;
    }

    _spi_tool->xfer[0].tx_buf = (unsigned long)tx;
    _spi_tool->xfer[0].len = tx_len;

    _spi_tool->xfer[1].rx_buf = (unsigned long)rx;
    _spi_tool->xfer[1].len = rx_len;

    status = ioctl(_spi_tool->fd, SPI_IOC_MESSAGE(2), _spi_tool->xfer);

    if (status < 0) {
        return 0;
    } 

    return rx_len;
}

int SpiDriverLite::spi_write(const char *tx, int tx_len) {
    int status = 0;
    if (!_spi_tool || !tx || _spi_tool->fd <= 0) {
        return 0;
    }

    _spi_tool->xfer[0].tx_buf = (unsigned long)tx;
    _spi_tool->xfer[0].len = tx_len;

    _spi_tool->xfer[1].rx_buf = 0;
    _spi_tool->xfer[1].len = 0;

    status = ioctl(_spi_tool->fd, SPI_IOC_MESSAGE(1), &_spi_tool->xfer[0]);

    if (status < 0) {
        return 0;
    }

    return tx_len;
}

#if 0
int main(int argc, char *argv[])
{
      FILE *lidar_ofp = fopen("lidar_capture_0.dat", "wb");

      _spi_tool_t *spi_tool = NULL;
      char tx[1025] = { 0x7, };
      char rx[1025] = { 0x4, };

      _spi_tool = spi_init("/dev/spidev0.0", 100000); // 0.1 Mhz
      if (!_spi_tool)
      {
            return 0;
      }

      int try_num = 10240;
      int try;
      for(try = 0; try < try_num; try++) {
	      // example : read chip id
	      tx[0] = 0xD; // chip id register address
	      tx[1] = 2; // chip id length. 2 bytes
	      spi_read(_spi_tool, tx, 1, rx, 2);

	      // example : write to the register. reg address 0x03
	      //memset(tx, 0, sizeof(char) * 16);
	      //tx[0] = 0x03 | 0x80; // must | 0x80
	      //tx[1] = 2; // data length
	      //tx[2] = 0x1;
	      //tx[3] = 0x2;
	      //spi_write(_spi_tool, tx, 4);

	      int data_size_l = (int) rx[0];
	      int data_size_h = (int) rx[1];
              int data_size = (data_size_h << 8) | data_size_l;
	      printf("Received data(data length) = %d [0x%02x 0x%02x] - %d\n", data_size, rx[1], rx[0], try);

	      if (data_size > 1024) {
                  data_size = 1024;
              }

	      spi_read(_spi_tool, tx, 0, rx, data_size);
	      fwrite(rx, sizeof(char), data_size, lidar_ofp);
#if 0
	      int i;
	      for(i = 0; i < data_size; i++) {
		  printf("%02x ", rx[i]);
	      }
              printf("\n");
#endif
      }

      spi_deinit(_spi_tool);

      fclose(lidar_ofp);
     
      return 0;
} 
#endif

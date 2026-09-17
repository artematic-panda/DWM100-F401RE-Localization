#include "main.h"
#include "helper.h"
#pragma once

#define FRAME_TYPE          0x1U
#define PAN_ID_COMPRESS     0x1U
#define DEST_ADDR_MODE      0x2U
#define SOURCE_ADDR_MODE    0x2U
#define FRAME_VERSION       0x1U

#define FRAME_CONTROL       FRAME_TYPE | (PAN_ID_COMPRESS << 6) | (DEST_ADDR_MODE << 10) | (FRAME_VERSION << 12) | (SOURCE_ADDR_MODE << 14)

/* DWM_write():
FUNCTION:
    Write buffered data to any register + subaddress on the DWM1000 chip through SPI
DEFS:
    hspi:      SPI peripheral address connected to DWM1000
    reg:       base register ID in DWM1000 
                    - bits 7 and 6 should be 0s
    subadress: subaddress of register (offset within register); byte level offset.
                    - subadress of 0 implies no subaddress (i.e. no 2nd or 3rd octet in header)
                    - bit 15 should be 0
    data:      pointer to user data buffer to write from
    data_size: number of bytes to write from the data buffer
*/ 
void DWM_write(
    SPI_HandleTypeDef* hspi,
    uint8_t  reg,
    uint16_t subaddress, 
    uint8_t* data,
    uint16_t data_size
);

/* DWM_read():
FUNCTION:
    Read data from any register + subaddress from the DWM1000 chip to a buffer through SPI
DEFS:
    hspi:      SPI peripheral address connected to DWM1000
    reg:       base register ID in DWM1000
                    - bits 6 and 7 should be 0s
    subadress: subaddress of register (offset within register); byte level offset
                    - subadress of 0 implies no subaddress (i.e. no 2nd or 3rd octet in header)
                    - bit 15 should be 0
    data:      pointer to user data buffer to write to
    data_size: number of bytes to write to the data buffer
*/ 
void DWM_read(
    SPI_HandleTypeDef* hspi,
    uint8_t  reg, 
    uint16_t subaddress, 
    uint8_t* data,
    uint16_t data_size
);

/* _DWM_SPI_FAULT_():
FUNCTION:
    If the SPI communication between the DWM and STM is broken, this function permanently
    called and blinks an LED. Since the LED is shared with the SPI_CLK, the SPI peripheral
    is deinitialized.
DEFS:
    hspi:      SPI peripheral address connected to DWM1000
*/
void _DWM_SPI_FAULT_(SPI_HandleTypeDef* hspi);

#if (CONFIG == TAG)
/* DWM_writeGPIO2():
FUNCTION:
    write HIGH or LOW to GPIO2 (LED on Qorvo board)
DEFS:
    hspi:       SPI peripheral address connected to DWM1000
    value:      1 or 0
*/
void DWM_writeGPIO2(SPI_HandleTypeDef* hspi, uint8_t value);
#endif

/* DWM_init():
FUNCTION:
    Initialize register and functions for DWM transceiver
DEFS:
    hspi:       SPI peripheral address connected to DWM1000
*/
void DWM_init(SPI_HandleTypeDef* hspi);

/* DWM_initTransmitBuffer():
FUNCTION:
    Writes data to the 1024 byte transmit buffer in the DWM1000 at the set offset.
DEFS:
    hspi:       SPI peripheral address connected to DWM1000
    data:       pointer to user data buffer to read from
    data_size:  length of data (in bytes) to be read
    offset:     offset from byte 0 into the 1024 byte transmit buffer
*/
void DWM_initTransmitBuffer(
    SPI_HandleTypeDef* hspi,
    uint8_t* data,
    uint16_t data_size,
    uint16_t offset
);

/* DWM_initTransmitSize():
FUNCTION:
    Writes the frame length register for the next transmission. This will be the amount of bytes 
    transmitted over RF. Maximum length is 1021 bytes if EXTENDED_FRAMES is defined, otherwise 125.
    2 bytes are omitted implicitly for CRC bytes.
DEFS:
    hspi:       SPI peripheral address connected to DWM1000
    frame_size: length of frame to be transmitted
*/
void DWM_initTransmitSize(
    SPI_HandleTypeDef* hspi,
    uint16_t frame_size
);

/* DWM_offsetTransmit():
FUNCTION:
    Writes the transmit offset register. This is the byte transmission will start reading from
    for frame_length bytes.
DEFS:
    hspi:       SPI peripheral address connected to DWM1000
    offset:     offset into 1024 byte Transmit Buffer (maximum offset is 1023 bytes)
*/
void DWM_offsetTransmit(
    SPI_HandleTypeDef* hspi,
    uint16_t offset
);

/* DWM_transmit():
FUNCTION:
    Begin transmission of frame over RF.
DEFS:
    hspi:       SPI peripheral address connected to DWM1000
    delay:      Set to TX_DELAY if delayed transmission is required, otherwise set to TX_NODELAY
    DX_TIME:    5 byte numer for delayed transmission timestamp, set to DX_TIMENULL if no delay
*/
void DWM_transmit(SPI_HandleTypeDef* hspi);

/* DWM_TRXOFF():
FUNCTION:       Turns off the DWM1000 transceiver 
DEFS: 
    hspi:       SPI peripheral address connected to DWM1000
*/
void DWM_TRXOFF(SPI_HandleTypeDef* hspi);

/* DWM_readReceive():
FUNCTION:
    Read the Receive Buffer contents into the given byte array pointer for data_size bytes.
DEFS:
    hspi:       SPI peripheral address connected to DWM1000
    data:       pointer to user data buffer to write to
    data_size:  length of data (in bytes) to be read
    offset:     starting byte to read from receive buffer
*/
void DWM_readReceive(
    SPI_HandleTypeDef* hspi,
    uint8_t* data,
    uint16_t data_size,
    uint16_t offset
);

/* DWM_checkFrameStatus():
FUNCTION:
    Checks the condition of the recceived frame with a mask of status bits, and returns HAL_OK if frame is
    okay. This may be modified based on preference, tolerance acceptance, or other.
    current masked bits: LDEDONE, RXPHE, RXFCG, RXFCE, RXRFSL, LDEERR
DEFS:
    hspi:       SPI peripheral address connected to DWM1000
*/
HAL_StatusTypeDef DWM_checkFrameStatus(SPI_HandleTypeDef* hspi);

/* DWM_getReceiveSize():
FUNCTION:
    Fetches the size of the receieved frame. This can be used to initialize array sizes before
    calling DWM_readReceive().
DEFS:
    hspi:       SPI peripheral address connected to DWM1000
*/
uint16_t DWM_getReceiveSize(SPI_HandleTypeDef* hspi);

/* DWM_getRXTimestamp():
FUNCTION:
    Fetches the adjusted receive timestamp from the DWM
DEFS:
    hspi:       SPI peripheral address connected to DWM1000
*/
TIME DWM_getRXTimestamp(SPI_HandleTypeDef* hspi);

/* DWM_getTXTimestamp():
FUNCTION:
    Fetches the adjusted transmit timestamp from the DWM
DEFS:
    hspi:       SPI peripheral address connected to DWM1000
*/
TIME DWM_getTXTimestamp(SPI_HandleTypeDef* hspi);

/* DWM_getSYSTIME():
FUNCTION:
    Fetches the system time from the DWM
DEFS:
    hspi:       SPI peripheral address connected to DWM1000
*/
TIME DWM_getSYSTIME(SPI_HandleTypeDef* hspi);

/* DWM_clearIRQ():
FUNCTION:
    clears both the TXPHS/TXFRS/RXDFR flags
DEFS:
    hspi:       SPI peripheral address connected to DWM1000
*/
void DWM_clearIRQ(SPI_HandleTypeDef* hspi);

/* DWM_setTXPHS_or_TXFRS_IRQ():
FUNCTION:
    Sets either TXPHS or TXFRS for the IRQ flag. TXPHS will be the min. time to
    read TX timestamp. TXFRS will be the min. time the radio can be turned off (via DWM_TRXOFF()).
DEFS:
    hspi:       SPI peripheral address connected to DWM1000
    PHS:        True if PHS mask. False if FRS mask.
*/
void DWM_setTXPHS_or_TXFRS_IRQ(SPI_HandleTypeDef* hspi, uint8_t PHS);

/* DWM_enableRX():
FUNCTION:
    turns on the receiver (RX) of the DWM1000
DEFS:
    hspi:       SPI peripheral address connected to DWM1000
*/
void DWM_enableRX(SPI_HandleTypeDef* hspi);
#include "DWM_driver.h"

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
){
    // SAFETY: reset top bits
    reg        &= 0x3F;
    subaddress &= 0x7FFF;
    // octet embeddings
    uint8_t flag_octet2 = subaddress > 0x0U;
    uint8_t flag_octet3 = subaddress > 0x7F;
    // first_octet: Write bit; extended header bit; reg address
    uint8_t first_octet  = (0x1U << 0x7U) | (flag_octet2 << 0x6U) | reg;
    // second_octet: extended header bit; subaddress lower 7 bits
    uint8_t second_octet = (flag_octet3 << 0x7U) | subaddress;
    // third_octet: upper 8 bits of subaddress
    uint8_t third_octet  = subaddress >> 0x7U;
    // packet header
    uint8_t header[3] = {first_octet, second_octet, third_octet};
    // If second or third octet are needed, add to header size
    uint8_t header_size = flag_octet2 + flag_octet3 + 0x1U;

    // Transaction
    HAL_GPIO_WritePin(DWM_CS_GPIO_Port, DWM_CS_Pin, GPIO_PIN_RESET); // Chip select
    // transmit header
    for (uint16_t iter = 0; iter < header_size; iter++) {
        LL_SPI_TransmitData8(hspi->Instance, header[iter]);
        while(!LL_SPI_IsActiveFlag_TXE(hspi->Instance)); // wait until TX buffer is empty
    }
    while(!LL_SPI_IsActiveFlag_TXE(hspi->Instance)); // wait until TX buffer is empty
    // transmit data
    for (uint16_t iter = 0; iter < data_size; iter++) {
        LL_SPI_TransmitData8(hspi->Instance, data[iter]);
        while(!LL_SPI_IsActiveFlag_TXE(hspi->Instance)); // wait until TX buffer is empty
    }
    while(LL_SPI_IsActiveFlag_BSY(hspi->Instance)); // wait until SPI peripheral is ready
    HAL_GPIO_WritePin(DWM_CS_GPIO_Port, DWM_CS_Pin, GPIO_PIN_SET); // Chip select
}

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
){
    // SAFETY: reset top bits
    reg        &= 0x3F;
    subaddress &= 0x7FFF;
    // octet embeddings
    char flag_octet2 = subaddress > 0x0U;
    char flag_octet3 = subaddress > 0x7F;
    // first_octet: read bit (implicit 0); extended header bit; reg address
    uint8_t first_octet  = (flag_octet2 << 0x6U) | reg;
    // second_octet: extended header bit; subaddress lower 7 bits
    uint8_t second_octet = (flag_octet3 << 0x7U) | subaddress;
    // third_octet: upper 8 bits of subaddress
    uint8_t third_octet  = subaddress >> 0x7U;
    // packet header
    uint8_t header[3] = {first_octet, second_octet, third_octet};
    // If second or third octet are needed, add to header size
    uint8_t header_size = flag_octet2 + flag_octet3 + 0x1U;

    // Transaction
    HAL_GPIO_WritePin(DWM_CS_GPIO_Port, DWM_CS_Pin, GPIO_PIN_RESET); // Chip select
    // transmit header
    for (uint16_t iter = 0; iter < header_size; iter++) {
        LL_SPI_TransmitData8(hspi->Instance, header[iter]);
        while(!LL_SPI_IsActiveFlag_TXE(hspi->Instance)); // wait until TX buffer is empty
    }
    while(LL_SPI_IsActiveFlag_BSY(hspi->Instance)); // wait until SPI peripheral is available
    (void)LL_SPI_ReceiveData8(hspi->Instance);      // dump current RX data
    // transmit data
    for (uint16_t iter = 0; iter < data_size; iter++) {
        LL_SPI_TransmitData8(hspi->Instance, 0x00); // dumby TX for clock gen
        while(!LL_SPI_IsActiveFlag_RXNE(hspi->Instance)); // wait until RX buffer has data
        data[iter] = LL_SPI_ReceiveData8(hspi->Instance);
    }
    while(LL_SPI_IsActiveFlag_BSY(hspi->Instance)); // wait until SPI peripheral is available
    HAL_GPIO_WritePin(DWM_CS_GPIO_Port, DWM_CS_Pin, GPIO_PIN_SET); // Chip select
}

/* _DWM_SPI_FAULT_():
FUNCTION:
    If the SPI communication between the DWM and STM is broken, this function permanently
    called and blinks an LED. Since the LED is shared with the SPI_CLK, the SPI peripheral
    is deinitialized.
DEFS:
    hspi:      SPI peripheral address connected to DWM1000
*/
void _DWM_SPI_FAULT_(SPI_HandleTypeDef* hspi) {
    LL_SPI_Disable(hspi->Instance);
    
    GPIO_InitTypeDef GPIO_BlinkLED;
    GPIO_BlinkLED.Pin   = GPIO_PIN_5;
    GPIO_BlinkLED.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_BlinkLED.Pull  = GPIO_NOPULL;
    GPIO_BlinkLED.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_BlinkLED);

    while(1) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
        HAL_Delay(500U);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
        HAL_Delay(500U);
    }
}

#if (CONFIG == TAG)
/* DWM_writeGPIO2():
FUNCTION:
    write HIGH or LOW to GPIO2 (LED on Qorvo board)
DEFS:
    hspi:       SPI peripheral address connected to DWM1000
    value:      1 or 0
*/
void DWM_writeGPIO2(SPI_HandleTypeDef* hspi, uint8_t value) {
    uint8_t write = ((value & 1U) << 2U) | (1U << 6U);
    DWM_write(hspi, 0x26U, 0xCU, &write, 1U); // write value with GOM2
}
#endif

/* DWM_init():
FUNCTION:
    Initialize register and functions for DWM transceiver
DEFS:
    hspi:       SPI peripheral address connected to DWM1000
*/
void DWM_init(SPI_HandleTypeDef* hspi) {
    // REF:     DWM_Write(&spi, reg, subaddress, &data_buff, data_size)
    uint8_t data[4];

    // DWM_RESET is initialized to open-drain ON (pulling reset LOW).
    HAL_Delay(100U); // Wait 100ms to stabilize reset.
    HAL_GPIO_WritePin(DWM_RESET_GPIO_Port, DWM_RESET_Pin, 1); // float RESET pin (DWM1000 internally pulls-up)
    HAL_Delay(100U); // Wait 100 ms to complete boot-up.
    
    // Verify communication with Device ID
    DWM_read(hspi, 0U, 0U, data, 4U);
    if (buff2num_LE(data) != DWM_DEVICE_ID) _DWM_SPI_FAULT_(hspi);       // Faulty DEVICE_ID read
    
    // Register inits    
    DWM_read (hspi, 0x08U,    0x2U,                          data,  1U); // TXPRF read
    // data[0] = (data[0] & 0xC0) | (2U) | (9U << 2);                       // TXPRF (64 MHz); TXPSR (d256 length)
    data[0] = (data[0] & 0xC0) | (2U) | (1U << 2);                       // TXPRF (64 MHz); TXPSR (d64 length)
    DWM_write(hspi, 0x08U,    0x2U,                          data,  1U); // TXPRF & TXPSR write
    
    DWM_read (hspi, 0x1F,     0x2U,                          data,  2U); // TX/RX PCODE & RXPRF read
    data[0] = (data[0] & 0x33) | (0x2U << 6) | (0x2U << 2);              // Set TX_PCODE (d10) bottom 2 bits(b10) & RXPRF (b10) 
    data[1] = (data[1] & 0x00) | (2U) | (0xAU << 3U);                    // Set TX_PCODE (d10)    top 2 bits & RX_PCODE (d10)
    DWM_write(hspi, 0x1F,     0x2U,                          data,  2U); // TX/RX PCODE write

    DWM_write(hspi, 0x23U,    0x4U, num2buff_LE(    0x889BU, data), 2U); // AGC_TUNE1  PRF 64 MHz
    DWM_write(hspi, 0x23U,    0xCU, num2buff_LE(0x2502A907U, data), 4U); // AGC_TUNE2
    DWM_write(hspi, 0x23U,   0x12U, num2buff_LE(    0x0035U, data), 2U); // AGC_TUNE3
    // DWM_write(hspi, 0x27U,    0x8U, num2buff_LE(0x333B00BEU, data), 4U); // DRX_TUNE2  PRF 64 MHz & d256 length (PAC d16)
    DWM_write(hspi, 0x27U,    0x8U, num2buff_LE(0x313B006BU, data), 4U); // DRX_TUNE2  PRF 64 MHz & d64 length (PAC d8)
    DWM_write(hspi, 0x1EU,    0x0U, num2buff_LE(0x25466788U, data), 4U); // TX_POWER   Channel 5 PRF 64 MHz
    DWM_write(hspi, 0x28U,    0xCU, num2buff_LE(  0x1E3FE3U, data), 3U); // RF_TXCTRL  Channel 5
    DWM_write(hspi, 0x2AU,    0xBU, num2buff_LE(      0xB5U, data), 1U); // TC_PGDELAY Channel 5
    DWM_write(hspi, 0x2BU,    0xBU, num2buff_LE(      0xBEU, data), 1U); // FS_PLLTUNE Channel 5
    DWM_write(hspi, 0x27U,    0x4U, num2buff_LE(    0x008DU, data), 2U); // DRX_TUNE1a PRF 64 MHz
    DWM_write(hspi, 0x2EU, 0x1806U, num2buff_LE(    0x0607U, data), 2U); // LDE_CFG2   PRF 64 MHz
    DWM_write(hspi, 0x2EU, 0x2804U, num2buff_LE(    0x3332U, data), 2U); // LDE_REPC   RX_PCODE d10
    uint32_t IRQ = (1U << 13U) | (1U << 6U);
    DWM_write(hspi, 0x0EU,    0x0U, num2buff_LE(        IRQ, data), 2U); // Interrupt mask for RXDFR and TXPHS

    // Frame filtering
    DWM_write(hspi, 0x04U,   0x00U, num2buff_LE(         9U, data), 1U); // Frame filtering with Data frame only
    uint32_t PANADR = ((uint32_t)PAN_ID << 16) | (uint32_t)SHORT_ADDR;
    DWM_write(hspi, 0x03U,   0x00U, num2buff_LE(     PANADR, data), 4U); // Network address and unique device address

    // Antenna Delay TX and RX (assumed same; emperically set)
    uint16_t delay = 0x3E73; // ANT_DELAY;
    DWM_write(hspi, 0x18U,   0x00U, num2buff_LE(      delay, data), 2U); // TX delay
    DWM_write(hspi, 0x2EU, 0x1804U, num2buff_LE(      delay, data), 2U); // RX delay

    #if (CONFIG == TAG)
    // LED blink setup
    DWM_write(hspi, 0x26U,    0x8U, num2buff_LE(   1U << 6U, data), 1U); // GDM enable for gpio 2
    DWM_write(hspi, 0x26U,    0x8U, num2buff_LE(         0U, data), 1U); // output mode for gpio 2; disable GDM 2
    #endif

    // Extended frame support
    #ifdef EXTENDED_FRAMES
        uint8_t buffer[2];
        // Read byte including PHR_MODE bits 17,16
        DWM_read(hspi, 0x04U, 0x02U, buffer, 1U);
        *buffer &= 0xFC;                            // preserve bits 23-18
        *buffer |= 0x3;                             // Set Frame Extension in PHR_MODE
        DWM_write(hspi, 0x04U, 0x02U, buffer, 1U);
    #endif

    // LDELOAD init start
    DWM_write(hspi, 0x36U, 0x00U, num2buff_LE(0x0301U,  data), 2U);       // PMSC_CTRL0
    DWM_write(hspi, 0x2DU, 0x06U, num2buff_LE(0x8000U,  data), 2U);       // OTP_CTRL
    HAL_Delay(2); // minimum 150uS
    DWM_write(hspi, 0x36U, 0x00U, num2buff_LE(0x0200U,  data), 2U);       // PMSC_CTRL0 (default)
    // LDELOAD init end
    DWM_write(hspi, 0x24U,  0x0U, num2buff_LE(1U << 2U, data), 1U);       // SET PLLDT
    DWM_write(hspi, 0x0FU,  0x3U, num2buff_LE(1U << 1U, data), 1U);       // CLEAR CLKPLL_LL

    // LDELOAD forces the SYSCLK to be XTI 19.2 MHz. SPI_CLK was <= 3 MHz for proper communication.
    // Set SPI to 20 MHz for forthcoming communications. 80 MHz P_CLK source.
    LL_SPI_Disable(hspi->Instance);
    hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4; // was 32 on boot
    if (HAL_SPI_Init(hspi) != HAL_OK)
        Error_Handler();
    LL_SPI_Enable(hspi->Instance);

    // DWM_initTransmitSize(hspi, BASE_FRAME_SIZE); // Preliminary minimum frame size set for communication (9 base for MAC)
    uint8_t MAC_FRAME[BASE_FRAME_SIZE];
    *(uint16_t*)MAC_FRAME                   = FRAME_CONTROL;
    *(MAC_FRAME + 2)                        = 0x00U;
    *(uint16_t*)(MAC_FRAME + 3)             = PAN_ID;
    *(uint16_t*)(MAC_FRAME + D_ADDR_OFFSET) = TAG_ADDR;     // dest (default; not required here)
    *(uint16_t*)(MAC_FRAME + S_ADDR_OFFSET) = SHORT_ADDR;   // source (should never change)
    DWM_initTransmitBuffer(hspi, MAC_FRAME, BASE_FRAME_SIZE, 0U);

    DWM_write(hspi, 0x04U, 0x03U, num2buff_LE(1U << 5, data), 1U); // RXAUTR
    #if CONFIG != TAG // listen for broadcasts from TAG
        DWM_enableRX(hspi);
    #endif
}

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
){
    // check for bad frame size + offset size
    if ((offset + data_size) > 0x3FFU) return;
    // configure transmit buffer data
    DWM_write(hspi, 0x09U, offset, data, data_size);
}

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
){  
    frame_size += 2; // need + 2 for CRC
    if (frame_size < 3) return; // per IEEE 802.15.4 Standard, minimum 3 byte frame (1 + 2 CRC)
    // check for bad frame request
    #ifdef EXTENDED_FRAMES
        if (frame_size > 0x3FFU) return;
    #else
        if (frame_size > 0x7FU) return;
    #endif
    uint8_t buffer[2];
    // configure frame length
    DWM_read(hspi, 0x08U, 0x00U, buffer, 2U);
    *(uint16_t*)buffer &= 0xFC00;                   // preserve bits 15-10
    *(uint16_t*)buffer |= frame_size;               // set frame length
    DWM_write(hspi, 0x08U, 0x00U, buffer, 2U);
}

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
){
    // check for bad offset request
    if (offset > 0x3FFU) return;
    // configure transmit buffer offset
    uint8_t buffer[2];
    DWM_read(hspi, 0x08U, 0x02U, buffer, 2U);
    *(uint16_t*)buffer &= 0x003F;                   // preserve bits 21-16
    *(uint16_t*)buffer |= offset;                   // set offset
    DWM_write(hspi, 0x08U, 0x02U, buffer, 2U);
}

/* DWM_transmit():
FUNCTION:
    Begin transmission of frame over RF.
DEFS:
    hspi:       SPI peripheral address connected to DWM1000
*/
void DWM_transmit(SPI_HandleTypeDef* hspi) {  
    // NOTE: assumes all other bits in byte are 0 by their functionality
    uint8_t buffer = (1 << 1) | (1 << 7);          // Set TXSTRT and WAIT4RESP
    DWM_write(hspi, 0x0DU, 0x00U, &buffer, 1U);    // Start transmission
}

/* DWM_TRXOFF():
FUNCTION:       Turns off the DWM1000 transceiver 
DEFS: 
    hspi:       SPI peripheral address connected to DWM1000
*/
void DWM_TRXOFF(SPI_HandleTypeDef* hspi) {
    // NOTE: assumes all other bits in byte are 0 by their functionality
    uint8_t buffer = 0x40;                           // set transceiver to idle
    DWM_write(hspi, 0x0DU, 0x00U, &buffer, 1U);      // Write to turn off transceiver
}

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
){
    // bad frame length request
    if ((data_size + offset) > 0x3FFU) return;
    // Read the frame
    DWM_read(hspi, 0x11U, offset, data, data_size);
}

/* DWM_checkFrameStatus():
FUNCTION:
    Checks the condition of the recceived frame with a mask of status bits, and returns HAL_OK if frame is
    okay. This may be modified based on preference, tolerance acceptance, or other.
    current masked bits: LDEDONE, RXPHE, RXFCG, RXFCE, RXRFSL, LDEERR
DEFS:
    hspi:       SPI peripheral address connected to DWM1000
*/
HAL_StatusTypeDef DWM_checkFrameStatus(SPI_HandleTypeDef* hspi) {
    // Verify frame has no errors
    uint16_t buffer;
    uint16_t status_mask = 0b0000010111010100;
    uint16_t expected    = 0b0000000001000100;
    DWM_read(hspi, 0x0FU, 0x01U, (uint8_t*)&buffer, 2U);
    // RXFCG HIGH and RXFCE LOW represent a good frame; all other bits are to monitor for possible RX issues.
    HAL_StatusTypeDef status = ((buffer & status_mask) != expected);
    return status;
}

/* DWM_getReceiveSize():
FUNCTION:
    Fetches the size of the receieved frame. This can be used to initialize array sizes before
    calling DWM_readReceive().
DEFS:
    hspi:       SPI peripheral address connected to DWM1000
*/
uint16_t DWM_getReceiveSize(SPI_HandleTypeDef* hspi){  
    uint8_t buffer[2];
    DWM_read(hspi, 0x10U, 0x00U, buffer, 2U);
    return *(uint16_t*)buffer & 0x3FF;
}

/* DWM_getRXTimestamp():
FUNCTION:
    Fetches the adjusted receive timestamp from the DWM
DEFS:
    hspi:       SPI peripheral address connected to DWM1000
*/
TIME DWM_getRXTimestamp(SPI_HandleTypeDef* hspi){  
    TIME buffer;
    DWM_read(hspi, 0x15U, 0x00U, (uint8_t*)&buffer, 5U);
    return buffer;
}

/* DWM_getTXTimestamp():
FUNCTION:
    Fetches the adjusted transmit timestamp from the DWM
DEFS:
    hspi:       SPI peripheral address connected to DWM1000
*/
TIME DWM_getTXTimestamp(SPI_HandleTypeDef* hspi){  
    TIME buffer;
    DWM_read(hspi, 0x17U, 0x00U, (uint8_t*)&buffer, 5U);
    return buffer;
}

/* DWM_getSYSTIME():
FUNCTION:
    Fetches the system time from the DWM
DEFS:
    hspi:       SPI peripheral address connected to DWM1000
*/
TIME DWM_getSYSTIME(SPI_HandleTypeDef* hspi){  
    TIME buffer;
    DWM_read(hspi, 0x06U, 0x00U, (uint8_t*)&buffer, 5U);
    return buffer;
}

/* DWM_clearIRQ():
FUNCTION:
    clears both the TXPHS/TXFRS/RXDFR flags
DEFS:
    hspi:       SPI peripheral address connected to DWM1000
*/
void DWM_clearIRQ(SPI_HandleTypeDef* hspi){
    // clear TXPHS/TXFRS/RXDFR
    uint16_t clear_IRQ = (1U << 6) | (1U << 7) | (1U << 13);
    DWM_write(hspi, 0x0FU, 0x00U, (uint8_t*)&clear_IRQ, 2U);
}

/* DWM_setTXPHS_or_TXFRS_IRQ():
FUNCTION:
    Sets either TXPHS or TXFRS for the IRQ flag. TXPHS will be the min. time to
    read TX timestamp. TXFRS will be the min. time the radio can be turned off (via DWM_TRXOFF()).
    This assumes no other masks in the first byte are active.
DEFS:
    hspi:       SPI peripheral address connected to DWM1000
    PHS:        True if PHS mask. False if FRS mask.
*/
void DWM_setTXPHS_or_TXFRS_IRQ(SPI_HandleTypeDef* hspi, uint8_t PHS) {
    uint8_t IRQ_masks = (1U << (PHS ? 6 : 7)); // set PHS or FRS mask for TX
    DWM_write(hspi, 0x0EU, 0x0U, &IRQ_masks, 1U);
}

/* DWM_enableRX():
FUNCTION:
    turns on the receiver (RX) of the DWM1000
DEFS:
    hspi:       SPI peripheral address connected to DWM1000
*/
void DWM_enableRX(SPI_HandleTypeDef* hspi){
    uint8_t data = 1U;
    DWM_write(hspi, 0x0DU, 0x01U, &data, 1U); // RXENAB on DWM1000
}
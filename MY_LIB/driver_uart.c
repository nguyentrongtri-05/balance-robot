#include "driver_uart.h"
#include <stdarg.h>
#include <string.h>

/* Ring Buffer nhận dữ liệu từ USART3 ngắt RXNE */
static volatile uint8_t  s_rxBuf[UART3_RX_BUFFER_SIZE];
static volatile uint16_t s_rxHead = 0;
static volatile uint16_t s_rxTail = 0;

/* Buffer truyền DMA cho USART3 TX (DMA1 Stream 3 Channel 4) */
static char s_txDmaBuf[UART3_TX_DMA_BUF_SIZE];
static volatile uint8_t s_dmaTxBusy = 0;

void UART3_Init(uint32_t baudrate)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef  NVIC_InitStructure;

    /* 1. Cấp clock cho GPIOD và USART3 */
    RCC_AHB1PeriphClockCmd(UART3_GPIO_CLK, ENABLE);
    RCC_APB1PeriphClockCmd(UART3_CLK, ENABLE);

    /* 2. Cấu hình chân PD8 (TX) và PD9 (RX) ở chế độ Alternate Function AF7 */
    GPIO_InitStructure.GPIO_Pin   = UART3_TX_PIN | UART3_RX_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(UART3_PORT, &GPIO_InitStructure);

    /* Nối chân PD8 và PD9 vào ngoại vi USART3 (AF7) */
    GPIO_PinAFConfig(UART3_PORT, GPIO_PinSource8, GPIO_AF_USART3);
    GPIO_PinAFConfig(UART3_PORT, GPIO_PinSource9, GPIO_AF_USART3);

    /* 3. Cấu hình thông số truyền thông USART3 */
    USART_InitStructure.USART_BaudRate            = baudrate;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_Parity              = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART3, &USART_InitStructure);

    /* 4. Cấu hình ngắt nhận RXNE
     * Độ ưu tiên 6 (phải >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY = 5 của FreeRTOS)
     */
    NVIC_InitStructure.NVIC_IRQChannel                   = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* Bật cờ ngắt khi có byte nhận về */
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

    /* 5. Cấu hình DMA cho kênh truyền TX */
    UART3_DMA_Init();

    /* 6. Kích hoạt USART3 */
    USART_Cmd(USART3, ENABLE);
}

void UART3_DMA_Init(void)
{
    DMA_InitTypeDef  DMA_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* 1. Cấp clock cho DMA1 (STM32F4: USART3 TX nối với DMA1 Stream 3, Channel 4) */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

    /* 2. Đảm bảo tắt stream trước khi cấu hình */
    DMA_Cmd(DMA1_Stream3, DISABLE);
    while (DMA_GetCmdStatus(DMA1_Stream3) != DISABLE)
    {
    }

    /* 3. Cấu hình thông số DMA1_Stream3 */
    DMA_DeInit(DMA1_Stream3);
    DMA_InitStructure.DMA_Channel            = DMA_Channel_4;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(USART3->DR);
    DMA_InitStructure.DMA_Memory0BaseAddr    = (uint32_t)s_txDmaBuf;
    DMA_InitStructure.DMA_DIR                = DMA_DIR_MemoryToPeripheral;
    DMA_InitStructure.DMA_BufferSize         = 0;
    DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode               = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority           = DMA_Priority_Medium;
    DMA_InitStructure.DMA_FIFOMode           = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold      = DMA_FIFOThreshold_HalfFull;
    DMA_InitStructure.DMA_MemoryBurst        = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;
    DMA_Init(DMA1_Stream3, &DMA_InitStructure);

    /* 4. Cấu hình ngắt hoàn tất truyền DMA (Transfer Complete Interrupt) */
    NVIC_InitStructure.NVIC_IRQChannel                   = DMA1_Stream3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6; /* An toàn với FreeRTOS */
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    DMA_ITConfig(DMA1_Stream3, DMA_IT_TC, ENABLE);

    /* 5. Cho phép USART3 phát tín hiệu kích hoạt DMA khi cần gửi */
    USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);

    s_dmaTxBusy = 0;
}

uint8_t UART3_DMA_IsBusy(void)
{
    return s_dmaTxBusy;
}

uint8_t UART3_SendDMA(const void *data, uint16_t len)
{
    if (data == NULL || len == 0) return 0;
    if (len > UART3_TX_DMA_BUF_SIZE) len = UART3_TX_DMA_BUF_SIZE;

    /* Nếu DMA đang bận truyền gói trước đó thì bỏ qua để không ghi đè */
    if (s_dmaTxBusy)
    {
        return 0;
    }

    s_dmaTxBusy = 1;

    /* Sao chép dữ liệu vào bộ đệm tĩnh của DMA */
    memcpy((void *)s_txDmaBuf, data, len);

    /* Tắt stream để nạp số lượng byte mới */
    DMA_Cmd(DMA1_Stream3, DISABLE);
    while (DMA_GetCmdStatus(DMA1_Stream3) != DISABLE)
    {
    }

    /* Xóa các cờ ngắt của DMA1_Stream3 */
    DMA_ClearFlag(DMA1_Stream3, DMA_FLAG_TCIF3 | DMA_FLAG_HTIF3 |
                                DMA_FLAG_TEIF3 | DMA_FLAG_DMEIF3 | DMA_FLAG_FEIF3);

    /* Nạp độ dài và kích hoạt truyền */
    DMA_SetCurrDataCounter(DMA1_Stream3, len);
    DMA_Cmd(DMA1_Stream3, ENABLE);

    return 1;
}

uint8_t UART3_PrintfDMA(const char *fmt, ...)
{
    char tempBuf[UART3_TX_DMA_BUF_SIZE];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(tempBuf, sizeof(tempBuf), fmt, args);
    va_end(args);

    if (len <= 0) return 0;
    return UART3_SendDMA(tempBuf, (uint16_t)len);
}

uint8_t UART_CalculateChecksum(uint8_t id, uint8_t len, const uint8_t *pData)
{
    uint8_t cs = 0;
#if (UART_CS_CURRENT_MODE == UART_CS_MODE_OR)
    cs = (uint8_t)(id | len);
    if (pData != NULL) {
        for (uint8_t i = 0; i < len; i++) {
            cs = (uint8_t)(cs | pData[i]);
        }
    }
#elif (UART_CS_CURRENT_MODE == UART_CS_MODE_XOR)
    cs = (uint8_t)(id ^ len);
    if (pData != NULL) {
        for (uint8_t i = 0; i < len; i++) {
            cs = (uint8_t)(cs ^ pData[i]);
        }
    }
#elif (UART_CS_CURRENT_MODE == UART_CS_MODE_SUM)
    cs = (uint8_t)(id + len);
    if (pData != NULL) {
        for (uint8_t i = 0; i < len; i++) {
            cs = (uint8_t)(cs + pData[i]);
        }
    }
#endif
    return cs;
}

uint8_t UART3_SendPacketDMA(uint8_t id, const void *pData, uint8_t len)
{
    uint8_t packet[UART3_TX_DMA_BUF_SIZE];
    const uint8_t *dataBytes = (const uint8_t *)pData;

    if (len > (UART3_TX_DMA_BUF_SIZE - 5)) {
        len = UART3_TX_DMA_BUF_SIZE - 5;
    }

    packet[0] = UART_FRAME_HEADER;
    packet[1] = id;
    packet[2] = len;

    if (dataBytes != NULL && len > 0) {
        memcpy(&packet[3], dataBytes, len);
    }

    uint8_t checksum = UART_CalculateChecksum(id, len, dataBytes);
    packet[3 + len] = checksum;
    packet[4 + len] = UART_FRAME_TAIL;

    return UART3_SendDMA(packet, (uint16_t)(len + 5));
}

void UART3_SendPacket(uint8_t id, const void *pData, uint8_t len)
{
    const uint8_t *dataBytes = (const uint8_t *)pData;
    uint8_t checksum = UART_CalculateChecksum(id, len, dataBytes);

    UART3_SendChar((char)UART_FRAME_HEADER);
    UART3_SendChar((char)id);
    UART3_SendChar((char)len);

    if (dataBytes != NULL) {
        for (uint8_t i = 0; i < len; i++) {
            UART3_SendChar((char)dataBytes[i]);
        }
    }

    UART3_SendChar((char)checksum);
    UART3_SendChar((char)UART_FRAME_TAIL);
}

/* ISR xử lý ngắt khi DMA1 Stream 3 truyền xong */
void DMA1_Stream3_IRQHandler(void)
{
    if (DMA_GetITStatus(DMA1_Stream3, DMA_IT_TCIF3) != RESET)
    {
        DMA_ClearITPendingBit(DMA1_Stream3, DMA_IT_TCIF3);
        s_dmaTxBusy = 0; /* Đã truyền xong toàn bộ khung dữ liệu */
    }
}

void UART3_SendChar(char c)
{
    /* Chờ cờ TXE (Transmit data register empty) */
    while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET)
    {
    }
    USART_SendData(USART3, (uint16_t)c);
}

void UART3_SendString(const char *str)
{
    if (str == NULL) return;
    while (*str)
    {
        UART3_SendChar(*str++);
    }
}

void UART3_SendBytes(const uint8_t *data, uint16_t len)
{
    if (data == NULL) return;
    while (len--)
    {
        UART3_SendChar((char)*data++);
    }
}

void UART3_Printf(const char *fmt, ...)
{
    char buf[128];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    UART3_SendString(buf);
}

uint16_t UART3_Available(void)
{
    uint16_t head = s_rxHead;
    uint16_t tail = s_rxTail;
    if (head >= tail)
    {
        return (head - tail);
    }
    return (UART3_RX_BUFFER_SIZE - tail + head);
}

int16_t UART3_ReadChar(void)
{
    if (s_rxHead == s_rxTail)
    {
        return -1; /* Bộ đệm rỗng */
    }
    uint8_t c = s_rxBuf[s_rxTail];
    s_rxTail = (s_rxTail + 1u) % UART3_RX_BUFFER_SIZE;
    return (int16_t)c;
}

uint16_t UART3_ReadLine(char *buf, uint16_t maxLen)
{
    if (buf == NULL || maxLen == 0) return 0;

    /* Kiểm tra xem trong buffer có ký tự kết thúc dòng '\n' hay chưa */
    uint16_t count = UART3_Available();
    if (count == 0) return 0;

    uint16_t i;
    uint8_t hasLine = 0;
    uint16_t tempTail = s_rxTail;
    for (i = 0; i < count; i++)
    {
        if (s_rxBuf[tempTail] == '\n' || s_rxBuf[tempTail] == '\r')
        {
            hasLine = 1;
            break;
        }
        tempTail = (tempTail + 1u) % UART3_RX_BUFFER_SIZE;
    }

    if (!hasLine) return 0;

    /* Đọc chuỗi ra buf */
    uint16_t len = 0;
    while (UART3_Available() > 0 && len < (maxLen - 1))
    {
        int16_t ch = UART3_ReadChar();
        if (ch == '\r') continue;
        if (ch == '\n') break;
        buf[len++] = (char)ch;
    }
    buf[len] = '\0';
    return len;
}

/* ISR xử lý ngắt nhận của USART3 */
void USART3_IRQHandler(void)
{
    if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {
        uint8_t c = (uint8_t)USART_ReceiveData(USART3);
        uint16_t nextHead = (s_rxHead + 1u) % UART3_RX_BUFFER_SIZE;
        if (nextHead != s_rxTail)
        {
            s_rxBuf[s_rxHead] = c;
            s_rxHead = nextHead;
        }
        USART_ClearITPendingBit(USART3, USART_IT_RXNE);
    }

    /* Xử lý cờ Overrun Error nếu truyền quá nhanh */
    if (USART_GetFlagStatus(USART3, USART_FLAG_ORE) != RESET)
    {
        (void)USART_ReceiveData(USART3);
    }
}

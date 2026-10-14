#include "driver_uart.h"
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

    RCC_AHB1PeriphClockCmd(UART3_GPIO_CLK, ENABLE);
    RCC_APB1PeriphClockCmd(UART3_CLK, ENABLE);

    GPIO_InitStructure.GPIO_Pin   = UART3_TX_PIN | UART3_RX_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(UART3_PORT, &GPIO_InitStructure);

    GPIO_PinAFConfig(UART3_PORT, GPIO_PinSource8, GPIO_AF_USART3);
    GPIO_PinAFConfig(UART3_PORT, GPIO_PinSource9, GPIO_AF_USART3);

    USART_InitStructure.USART_BaudRate            = baudrate;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_Parity              = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART3, &USART_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel                   = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

    UART3_DMA_Init();
    USART_Cmd(USART3, ENABLE);
}

void UART3_DMA_Init(void)
{
    DMA_InitTypeDef  DMA_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

    DMA_Cmd(DMA1_Stream3, DISABLE);
    while (DMA_GetCmdStatus(DMA1_Stream3) != DISABLE) {
    }

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

    NVIC_InitStructure.NVIC_IRQChannel                   = DMA1_Stream3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    DMA_ITConfig(DMA1_Stream3, DMA_IT_TC, ENABLE);
    USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);

    s_dmaTxBusy = 0;
}

uint8_t UART3_SendDMA(const void *data, uint16_t len)
{
    if (data == NULL || len == 0) return 0;
    if (len > UART3_TX_DMA_BUF_SIZE) len = UART3_TX_DMA_BUF_SIZE;

    if (s_dmaTxBusy) {
        if (DMA_GetCmdStatus(DMA1_Stream3) == DISABLE) {
            s_dmaTxBusy = 0;
        } else {
            return 0;
        }
    }

    s_dmaTxBusy = 1;
    memcpy((void *)s_txDmaBuf, data, len);

    DMA_Cmd(DMA1_Stream3, DISABLE);
    while (DMA_GetCmdStatus(DMA1_Stream3) != DISABLE) {
    }

    DMA_ClearFlag(DMA1_Stream3, DMA_FLAG_TCIF3 | DMA_FLAG_HTIF3 |
                                DMA_FLAG_TEIF3 | DMA_FLAG_DMEIF3 | DMA_FLAG_FEIF3);

    DMA_SetCurrDataCounter(DMA1_Stream3, len);
    DMA_Cmd(DMA1_Stream3, ENABLE);

    return 1;
}

uint8_t UART_CalculateChecksum(uint8_t id, uint8_t len, const uint8_t *pData)
{
    uint8_t cs = (uint8_t)(id | len);
    if (pData != NULL) {
        for (uint8_t i = 0; i < len; i++) {
            cs = (uint8_t)(cs | pData[i]);
        }
    }
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

    packet[3 + len] = UART_CalculateChecksum(id, len, dataBytes);
    packet[4 + len] = UART_FRAME_TAIL;

    return UART3_SendDMA(packet, (uint16_t)(len + 5));
}

void DMA1_Stream3_IRQHandler(void)
{
    if (DMA_GetITStatus(DMA1_Stream3, DMA_IT_TCIF3) != RESET) {
        DMA_ClearITPendingBit(DMA1_Stream3, DMA_IT_TCIF3);
        s_dmaTxBusy = 0;
    }
    if (DMA_GetITStatus(DMA1_Stream3, DMA_IT_TEIF3) != RESET) {
        DMA_ClearITPendingBit(DMA1_Stream3, DMA_IT_TEIF3);
        s_dmaTxBusy = 0;
    }
}

uint16_t UART3_Available(void)
{
    uint16_t head = s_rxHead;
    uint16_t tail = s_rxTail;
    if (head >= tail) {
        return (head - tail);
    }
    return (UART3_RX_BUFFER_SIZE - tail + head);
}

int16_t UART3_ReadChar(void)
{
    if (s_rxHead == s_rxTail) {
        return -1;
    }
    uint8_t c = s_rxBuf[s_rxTail];
    s_rxTail = (s_rxTail + 1u) % UART3_RX_BUFFER_SIZE;
    return (int16_t)c;
}

void USART3_IRQHandler(void)
{
    if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) {
        uint8_t c = (uint8_t)USART_ReceiveData(USART3);
        uint16_t nextHead = (s_rxHead + 1u) % UART3_RX_BUFFER_SIZE;
        if (nextHead != s_rxTail) {
            s_rxBuf[s_rxHead] = c;
            s_rxHead = nextHead;
        }
        USART_ClearITPendingBit(USART3, USART_IT_RXNE);
    }

    if (USART_GetFlagStatus(USART3, USART_FLAG_ORE) != RESET) {
        (void)USART_ReceiveData(USART3);
    }
}


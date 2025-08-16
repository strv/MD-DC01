#include "main.h"
#include "usart.h"

static char usart_rx_buf[UART_RX_BUF_LEN];
static int32_t usart_rx_wp = 0;
static int32_t usart_rx_rp = 0;
static int32_t usart_error = 0;
static int32_t usart_rx_nl = 0;
static int32_t usart_tx_busy = 0;

void usart_init(void)
{
  LL_USART_ClearFlag_UDR(UART);
  LL_USART_ClearFlag_TC(UART);
  LL_USART_EnableDirectionRx(UART);
  LL_USART_EnableDirectionTx(UART);
  LL_USART_EnableDMAReq_TX(UART);
  LL_USART_EnableIT_RXNE(UART);

  LL_DMA_DisableChannel(UART_DMA, UART_DMA_TX_CH);
  LL_DMA_SetPeriphAddress(UART_DMA, UART_DMA_TX_CH,
                          LL_USART_DMA_GetRegAddr(UART, LL_USART_DMA_REG_DATA_TRANSMIT));
  LL_DMA_EnableIT_TC(UART_DMA, UART_DMA_TX_CH);

  LL_USART_Enable(UART);
}

int usart_tx(const char* str, const int len)
{
  if (usart_tx_busy)
  {
    return -1;
  }
  LL_DMA_DisableChannel(UART_DMA, UART_DMA_TX_CH);
  LL_DMA_SetDataLength(UART_DMA, UART_DMA_TX_CH, len);
  LL_DMA_SetMemoryAddress(UART_DMA, UART_DMA_TX_CH, (uint32_t)str);
  LL_DMA_EnableChannel(UART_DMA, UART_DMA_TX_CH);
  usart_tx_busy = 1;
  return 0;
}

void usart_update_nl(void)
{
  int nl = 0;
  char c_prev = '\0';
  for (uint32_t i = usart_rx_rp; i != usart_rx_wp; i = (i+1) & UART_RX_BUF_MASK)
  {
    if (usart_rx_buf[i] == '\n')
    {
      if (c_prev != '\r')
      {
        nl++;
      }
      c_prev = usart_rx_buf[i];
      usart_rx_buf[i] = '\0';
    }
    else if (usart_rx_buf[i] == '\r')
    {
      nl++;
      c_prev = usart_rx_buf[i];
      usart_rx_buf[i] = '\0';
    }
    else
    {
      c_prev = usart_rx_buf[i];
    }
  }
  usart_rx_nl = nl;
}

int usart_readline(char* str, const int len)
{
  usart_update_nl();
  if (usart_rx_nl == 0)
  {
    return 0;
  }
  char c;
  for (int i = 0; i < len && usart_rx_rp != usart_rx_wp ; ++i)
  {
    c = usart_rx_buf[usart_rx_rp++];
    usart_rx_rp &= UART_RX_BUF_MASK;
    str[i] = c;
    if (c == '\0')
    {
      break;
    }
  }
  return len;
}

inline bool usart_is_tx_busy(void)
{
  return usart_tx_busy;
}

void usart_rx_cb(void)
{
  usart_rx_buf[usart_rx_wp] = LL_USART_ReceiveData8(UART);
  usart_rx_wp = (usart_rx_wp + 1) & UART_RX_BUF_MASK;
  if (usart_rx_wp == usart_rx_rp)
  {
    ++usart_rx_rp;
    usart_error |= 0x0001;
  }
}

void usart_tx_cb(void)
{
  usart_tx_busy = 0;
}

#include "user_usart.h"
#include "ZhangDaTou.h"

#define ZDT_UART_RX_BUF_SIZE 64

static uint8_t zdt_uart1_rx_buf[ZDT_UART_RX_BUF_SIZE];
static uint8_t zdt_uart3_rx_buf[ZDT_UART_RX_BUF_SIZE];

static void ZDT_UART_StartReceive(UART_HandleTypeDef *huart, uint8_t *rx_buf)
{
	if (HAL_UARTEx_ReceiveToIdle_DMA(huart, rx_buf, ZDT_UART_RX_BUF_SIZE) == HAL_OK) {
		__HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
	}
}

static void ZDT_UART_ParseMotorFrame(UART_HandleTypeDef *huart, uint8_t *data)
{
	if (pitchmotor.huart == huart) {
		ZhangDaTou_DataParm(data, &pitchmotor);
	}

	if (yawmotor.huart == huart) {
		ZhangDaTou_DataParm(data, &yawmotor);
	}
}

static void ZDT_UART_ParseFeedback(UART_HandleTypeDef *huart, uint8_t *data, uint16_t size)
{
	uint16_t index = 0;

	while ((index + 2) <= size) {
		uint16_t frame_len = 0;

		switch (data[index + 1]) {
			case 0x35:
				frame_len = 6;
				break;

			case 0x36:
				frame_len = 8;
				break;

			default:
				index++;
				continue;
		}

		if ((index + frame_len) > size) {
			return;
		}

		if (data[index + frame_len - 1] == 0x6B) {
			ZDT_UART_ParseMotorFrame(huart, &data[index]);
			index += frame_len;
		} else {
			index++;
		}
	}

}

void ZDT_UART_RxStart(void)
{
	ZDT_UART_StartReceive(&huart1, zdt_uart1_rx_buf);
	ZDT_UART_StartReceive(&huart3, zdt_uart3_rx_buf);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (huart->Instance == USART1) {
		ZDT_UART_ParseFeedback(huart, zdt_uart1_rx_buf, Size);
		ZDT_UART_StartReceive(&huart1, zdt_uart1_rx_buf);
	} else if (huart->Instance == USART3) {
		ZDT_UART_ParseFeedback(huart, zdt_uart3_rx_buf, Size);
		ZDT_UART_StartReceive(&huart3, zdt_uart3_rx_buf);
	}
}

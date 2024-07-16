#ifndef  __Communicate_H
#define  __Communicate_H

#include "main.h"
#include "usart.h"

#define Param_K  1 		// 定义蓝牙传输的倍率系数
#define RxBuffSize 100 		// 定义存储BUFF区的大小

typedef struct Rx_Struct{							//  定义接收参数结构体
	UART_HandleTypeDef *uart;   //定义接收用的串口
	uint8_t rData[RxBuffSize];  //  存储收到的信息
	uint8_t rDataBuffer[1];  		//  串口暂存区
	uint8_t rDataCount;  		//  串口指针
	uint8_t rDataFlag;  		//  等待串口传输的标志位
	char rdata_trans[10];	//	传输中继
}Rx_Str;

#endif
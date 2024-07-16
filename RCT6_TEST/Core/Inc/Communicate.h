#ifndef  __Communicate_H
#define  __Communicate_H

#include "main.h"
#include "usart.h"

#define Param_K  1 		// ������������ı���ϵ��
#define RxBuffSize 100 		// ����洢BUFF���Ĵ�С

typedef struct Rx_Struct{							//  ������ղ����ṹ��
	UART_HandleTypeDef *uart;   //��������õĴ���
	uint8_t rData[RxBuffSize];  //  �洢�յ�����Ϣ
	uint8_t rDataBuffer[1];  		//  �����ݴ���
	uint8_t rDataCount;  		//  ����ָ��
	uint8_t rDataFlag;  		//  �ȴ����ڴ���ı�־λ
	char rdata_trans[10];	//	�����м�
}Rx_Str;

#endif
#include "Communicate.h"
#include "main.h"
#include "math.h"
#include "string.h"
#include "CAR.h"
#include "Actuator.h"
/************************ ����*************************/
//����1 TX PA9   RX PA10   ������
//����2 TX PA2   RX PA3    ������
//����3 TX PB10  RX PB11   ��ݮ��
//����4 TX PC10  RX PC11
//����5 TX PC12  RX PD2
//ע�����Ŷ�Ӧ ��ͬ��Ƭ����Ӧ��ͬ
//L��Ӧ����ʵ�λ��   ����Ķ�Ӧ�������Ҹ����λ��
/*****************************************************/
Rx_Str RxS1 = {&huart3,0};		// ������սṹ��
Rx_Str *pr_RxS1 = &RxS1;

Rx_Str RxS2 = {&huart2,0};		
Rx_Str *pr_RxS2 = &RxS2;

extern float set_angel[2];  //x,y
extern char tasklist[8];
extern state actuator_state;
int task_flag;
extern messg car_messg;
extern flag carflag;
extern move ptz_move;

void Num_TransPID(char *pid_num, float *pid_val) 	//����ֵ��
{
	int i = 0;
	int float_flag = 0;
	float temp;
	int len = strlen(pid_num);
	for (; i<len; i++)
	{
		if(pid_num[i]=='.')
		{
				float_flag = 1;
			
		}
		else if(float_flag)
		{
			temp += (pid_num[i]-'0')*Param_K*pow(0.1,float_flag);
			float_flag++;
		}
		else
		{
			temp += (pid_num[i]-'0')*Param_K*pow(10,len-i-1);
		}
	}
	*pid_val = temp;
}


void communicate(Rx_Str *pr_Rx)
{
	
	if(pr_Rx->uart==&huart3)
	{
		while(HAL_UART_Receive_IT(pr_Rx->uart, pr_Rx->rDataBuffer, 1) != HAL_OK); // ���յ���һ�ֽ���Ϣ���봢����
		if(pr_Rx->rDataBuffer[0]!=0x00)
		{    		// ������յ��Ĳ���'\0'
			pr_Rx->rData[pr_Rx->rDataCount]=pr_Rx->rDataBuffer[0]; 
			pr_Rx->rData[pr_Rx->rDataCount + 1] = '\0';
			if(pr_Rx->rDataBuffer[0]=='?')
			{  		
				// ��ֹλ_���ζ�ȡ���
				
				if(pr_Rx->rData[0]=='t')
				{
					if(pr_Rx->rData[1]=='x')
					{  				// ƫ��ֵ��־
						char *bias_num = pr_Rx->rdata_trans;	//	���ݲ�����
						int i = 0;
						for(; pr_Rx->rData[i+2] >= '0' && pr_Rx->rData[i+2] <= '9' || pr_Rx->rData[i+2] == '.'; i++)
						{
							bias_num[i] = pr_Rx->rData[i+2];
						}
						bias_num[i] = '\0';
						Num_TransPID(bias_num, &(ptz_move.position[0][0]));// ������ֵ
					}
					else if(pr_Rx->rData[1]=='y')
					{
						char *bias_num = pr_Rx->rdata_trans;	//	���ݲ�����
						int i = 0;
						for(; pr_Rx->rData[i+2] >= '0' && pr_Rx->rData[i+2] <= '9' || pr_Rx->rData[i+2] == '.'; i++)
						{
							bias_num[i] = pr_Rx->rData[i+2];
						}
						bias_num[i] = '\0';
						Num_TransPID(bias_num, &(ptz_move.position[0][1]));// ������ֵ
					}
			  }
					if(pr_Rx->rData[0]=='r')
				{
					if(pr_Rx->rData[1]=='x')
					{  				// ƫ��ֵ��־
						char *bias_num = pr_Rx->rdata_trans;	//	���ݲ�����
						int i = 0;
						for(; pr_Rx->rData[i+2] >= '0' && pr_Rx->rData[i+2] <= '9' || pr_Rx->rData[i+2] == '.'; i++)
						{
							bias_num[i] = pr_Rx->rData[i+2];
						}
						bias_num[i] = '\0';
						Num_TransPID(bias_num, &(ptz_move.position[1][0]));// ������ֵ
					}
					else if(pr_Rx->rData[1]=='y')
					{
						char *bias_num = pr_Rx->rdata_trans;	//	���ݲ�����
						int i = 0;
						for(; pr_Rx->rData[i+2] >= '0' && pr_Rx->rData[i+2] <= '9' || pr_Rx->rData[i+2] == '.'; i++)
						{
							bias_num[i] = pr_Rx->rData[i+2];
						}
						bias_num[i] = '\0';
						Num_TransPID(bias_num, &(ptz_move.position[1][1]));// ������ֵ
					}
			  }
					if(pr_Rx->rData[0]=='b')
				{
					if(pr_Rx->rData[1]=='x')
					{  				// ƫ��ֵ��־
						char *bias_num = pr_Rx->rdata_trans;	//	���ݲ�����
						int i = 0;
						for(; pr_Rx->rData[i+2] >= '0' && pr_Rx->rData[i+2] <= '9' || pr_Rx->rData[i+2] == '.'; i++)
						{
							bias_num[i] = pr_Rx->rData[i+2];
						}
						bias_num[i] = '\0';
						Num_TransPID(bias_num, &(ptz_move.position[2][0]));// ������ֵ
					}
					else if(pr_Rx->rData[1]=='y')
					{
						char *bias_num = pr_Rx->rdata_trans;	//	���ݲ�����
						int i = 0;
						for(; pr_Rx->rData[i+2] >= '0' && pr_Rx->rData[i+2] <= '9' || pr_Rx->rData[i+2] == '.'; i++)
						{
							bias_num[i] = pr_Rx->rData[i+2];
						}
						bias_num[i] = '\0';
						Num_TransPID(bias_num, &(ptz_move.position[2][1]));// ������ֵ
					}
			  }
					if(pr_Rx->rData[0]=='l')
				{
					if(pr_Rx->rData[1]=='x')
					{  				// ƫ��ֵ��־
						char *bias_num = pr_Rx->rdata_trans;	//	���ݲ�����
						int i = 0;
						for(; pr_Rx->rData[i+2] >= '0' && pr_Rx->rData[i+2] <= '9' || pr_Rx->rData[i+2] == '.'; i++)
						{
							bias_num[i] = pr_Rx->rData[i+2];
						}
						bias_num[i] = '\0';
						Num_TransPID(bias_num, &(ptz_move.position[3][0]));// ������ֵ
					}
					else if(pr_Rx->rData[1]=='y')
					{
						char *bias_num = pr_Rx->rdata_trans;	//	���ݲ�����
						int i = 0;
						for(; pr_Rx->rData[i+2] >= '0' && pr_Rx->rData[i+2] <= '9' || pr_Rx->rData[i+2] == '.'; i++)
						{
							bias_num[i] = pr_Rx->rData[i+2];
						}
						bias_num[i] = '\0';
						Num_TransPID(bias_num, &(ptz_move.position[3][1]));// ������ֵ
					}
			  }
					if(pr_Rx->rData[0]=='o')
				 {
					if(pr_Rx->rData[1]=='x')
					{  				// ƫ��ֵ��־
						char *bias_num = pr_Rx->rdata_trans;	//	���ݲ�����
						int i = 0;
						for(; pr_Rx->rData[i+2] >= '0' && pr_Rx->rData[i+2] <= '9' || pr_Rx->rData[i+2] == '.'; i++)
						{
							bias_num[i] = pr_Rx->rData[i+2];
						}
						bias_num[i] = '\0';
						Num_TransPID(bias_num, &(ptz_move.position[4][0]));// ������ֵ
					}
					else if(pr_Rx->rData[1]=='y')
					{
						char *bias_num = pr_Rx->rdata_trans;	//	���ݲ�����
						int i = 0;
						for(; pr_Rx->rData[i+2] >= '0' && pr_Rx->rData[i+2] <= '9' || pr_Rx->rData[i+2] == '.'; i++)
						{
							bias_num[i] = pr_Rx->rData[i+2];
						}
						bias_num[i] = '\0';
						Num_TransPID(bias_num, &(ptz_move.position[4][1]));// ������ֵ
					}
			  }
				 
					if(pr_Rx->rData[0]=='m')
				 {
					if(pr_Rx->rData[1]=='x')
					{  				// ƫ��ֵ��־
						char *bias_num = pr_Rx->rdata_trans;	//	���ݲ�����
						int i = 0;
						for(; pr_Rx->rData[i+2] >= '0' && pr_Rx->rData[i+2] <= '9' || pr_Rx->rData[i+2] == '.'; i++)
						{
							bias_num[i] = pr_Rx->rData[i+2];
						}
						bias_num[i] = '\0';
						Num_TransPID(bias_num, &(ptz_move.position[5][0]));// ������ֵ
					}
					else if(pr_Rx->rData[1]=='y')
					{
						char *bias_num = pr_Rx->rdata_trans;	//	���ݲ�����
						int i = 0;
						for(; pr_Rx->rData[i+2] >= '0' && pr_Rx->rData[i+2] <= '9' || pr_Rx->rData[i+2] == '.'; i++)
						{
							bias_num[i] = pr_Rx->rData[i+2];
						}
						bias_num[i] = '\0';
						Num_TransPID(bias_num, &(ptz_move.position[5][1]));// ������ֵ
					}
			  }
				
				
				
				
					pr_Rx->rDataCount = -1;		// ��մ洢��(ָ�����)
				}
				pr_Rx->rDataCount++;									// ָ��䶯
			}
		
	}
	
	
	if(pr_Rx->uart==&huart2)
	{
		while(HAL_UART_Receive_IT(pr_Rx->uart, pr_Rx->rDataBuffer, 1) != HAL_OK); // ���յ���һ�ֽ���Ϣ���봢����
		if(pr_Rx->rDataBuffer[0]!=0x00)
		{    		// ������յ��Ĳ���'\0'
			pr_Rx->rData[pr_Rx->rDataCount]=pr_Rx->rDataBuffer[0]; 
			pr_Rx->rData[pr_Rx->rDataCount + 1] = '\0';
				if(pr_Rx->rDataBuffer[0]=='?')
				{  		// ��ֹλ_���ζ�ȡ���
					pr_Rx->rData[pr_Rx->rDataCount] = 0x00;
					if(pr_Rx->rData[0]=='1')
						{  				// v1pid���ڱ�־
						if(pr_Rx->rData[1]=='p')
						{				// p���ڱ�־
							char *pid_num = pr_Rx->rdata_trans;	//	����pid������
							int i = 0;
							for(; pr_Rx->rData[i+2] >= '0' && pr_Rx->rData[i+2] <= '9'; i++)
							{
								pid_num[i] = pr_Rx->rData[i+2];
							}
							pid_num[i+1] = '\0';
//							Num_TransPID(pid_num, &vxpid.kd);// ������ֵ
						}
						else if(pr_Rx->rData[1]=='i')
						{				// i���ڱ�־
							char *pid_num = pr_Rx->rdata_trans;	//	����pid������
							int i = 0;
							for(; pr_Rx->rData[i+2] >= '0' && pr_Rx->rData[i+2] <= '9'; i++){
								pid_num[i] = pr_Rx->rData[i+2];
						}
							pid_num[i+1] = '\0';
//							Num_TransPID(pid_num, &v1pid.ki);// ������ֵ
						}
						else if(pr_Rx->rData[1]=='d')
						{				// d���ڱ�־
							char *pid_num = pr_Rx->rdata_trans;	//	����pid������
							int i = 0;
							for(; pr_Rx->rData[i+2] >= '0' && pr_Rx->rData[i+2] <= '9'; i++)
							{
								pid_num[i] = pr_Rx->rData[i+2];
							}
							pid_num[i+1] = '\0';
//							Num_TransPID(pid_num, &v1pid.kd);// ������ֵ
						}
					}
				  else 	if(pr_Rx->rData[0]=='s')
				{
					if(pr_Rx->rData[1]=='x')
					{  				// ƫ��ֵ��־
						char *bias_num = pr_Rx->rdata_trans;	//	���ݲ�����
						int i = 0;
						for(; pr_Rx->rData[i+2] >= '0' && pr_Rx->rData[i+2] <= '9' || pr_Rx->rData[i+2] == '.'; i++)
						{
							bias_num[i] = pr_Rx->rData[i+2];
						}
						bias_num[i] = '\0';
						Num_TransPID(bias_num, &(set_angel[0]));// ������ֵ
					}
					else if(pr_Rx->rData[1]=='y')
					{
						char *bias_num = pr_Rx->rdata_trans;	//	���ݲ�����
						int i = 0;
						for(; pr_Rx->rData[i+2] >= '0' && pr_Rx->rData[i+2] <= '9' || pr_Rx->rData[i+2] == '.'; i++)
						{
							bias_num[i] = pr_Rx->rData[i+2];
						}
						bias_num[i] = '\0';
						Num_TransPID(bias_num, &(set_angel[1]));// ������ֵ
					}
			  }
					else if(pr_Rx->rData[0]=='T')
					{  			
							 switch (pr_Rx->rData[1])
							{
								case '1': actuator_state.task_flag = 1;
									break;
								case '2': actuator_state.task_flag = 2;
									break;
								case '3': actuator_state.task_flag = 3;
									break;
								case '4': actuator_state.task_flag = 4;
									break;
								case '5': actuator_state.task_flag = 5;
									break;
								case '6': actuator_state.task_flag = 6;
									break;
								default:
									break;
							}
				
					}
						pr_Rx->rDataCount = -1;		// ��մ洢��(ָ�����)
				}
				pr_Rx->rDataCount++;									// ָ��䶯
		}
	 }
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)//�����ж� ������ɻص�����  stm32����������һ��������������ݶ�������������
{
  
	if(huart == pr_RxS1->uart)       //��ݮ�ɴ���
	{
		HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
		communicate(pr_RxS1);
	}
	else if(huart == pr_RxS2->uart)            //������ 
	{
	  communicate(pr_RxS2);
	}
}


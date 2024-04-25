#include "pid.h"
/**
  * @brief          
  * @author         
  * @param[in] 
  * @retval	
  * @note           
  *  ������������ѣ� ��С����˳��顣
                    ���Ǳ�������֣� ����ٰ�΢�ּӡ�
                    �����񵴺�Ƶ���� ��������Ҫ�Ŵ�
                    ����Ư���ƴ��䣬 ����������С�⡣
                    ����ƫ��ظ����� ����ʱ�����½���
                    ���߲������ڳ��� ����ʱ���ټӳ���
                    ������Ƶ�ʿ죬 �Ȱ�΢�ֽ�������
                    ��������������� ΢��ʱ��Ӧ�ӳ���
                    ���������������� ǰ�ߺ���ı�һ��
                    һ������������� ������������͡�
  */

 /*�ڻ�������Ӧ�ٶȣ��⻷���������ٶ�*/
void PID_Calc(pid_t *pid)
{
	pid->err[2] = pid->err[1];
	pid->err[1] = pid->err[0];
	pid->err[0] = pid->set - pid->fdb;
	if(pid->mode==PID_DELTA)//����ʽPID
	{
		pid->iout = pid->ki*pid->err[0];
		
		pid->output += pid->kp*(pid->err[0]-pid->err[1]) \
					 + pid->iout                         \
					 + pid->kd*(pid->err[0]-2.0f*pid->err[1]+pid->err[2]);
	}
	else if(pid->mode==PID_POSITION)//λ��ʽPID
	{
		// pid->iout += pid->ki*pid->iout;
		pid->iout += pid->err[0];
		if(pid->iout > pid->ioutMax)            pid->iout = pid->ioutMax;
	    else if(pid->iout < (-pid->ioutMax))    pid->iout = (-pid->ioutMax);
		
		pid->output  = pid->kp*(pid->err[0]) \
					 + pid->ki*pid->iout             \
					 + pid->kd*(pid->err[0]-pid->err[1]);
	}
	//����޷�
	if(pid->output > pid->outputMax)            pid->output = pid->outputMax;
	else if(pid->output < (-pid->outputMax))    pid->output = (-pid->outputMax);
	

}
/**
  * @brief          
  * @author         
  * @param[in] 
  * @retval	
  * @note           
  */
void PID_Reset(pid_t *pid)
{
	pid->set = 0.0f;
	pid->fdb = 0.0f;
	pid->err[0] = 0.0f;
	pid->err[1] = 0.0f;
	pid->err[2] = 0.0f;
	pid->iout = 0.0f;
	pid->output = 0.0f;
}
 

cali_gimbal_t cali_gimbal_pid = 
{
/**
* @brief yaw pid param     PID_POSITION = 0,PID_DELTA = 1
* @note    
*/
	//��yaw�������ĵͼǵð�A��ĵ��������344��345ע�͵� 								+ ((chassis->connect->can2_rc_ctrl.gyro.yaw_set \
								+chassis->connect->can2_rc_ctrl.gyro.yaw_fdb)) );//+20.0f);// * GAMBAL_YAW_angle_VALUE)*0.08);
	.yaw_pid.position.kp =50,
	.yaw_pid.position.ki = 0,
	.yaw_pid.position.kd =5000,//5000,
	.yaw_pid.position.ioutput_max = 1000,
	.yaw_pid.position.output_max = 999,
	.yaw_pid.position.mode = PID_POSITION,	
	
	.yaw_pid.speed.kp = 200,
	.yaw_pid.speed.ki = 8,//5
	.yaw_pid.speed.kd = 90,
	.yaw_pid.speed.ioutput_max = 1000,
	.yaw_pid.speed.output_max = 25000,//22000
	.yaw_pid.speed.mode = PID_POSITION,	
/**
* @brief pitch pid param         
* @note    
*/
	// .pitch_pid.position.kp = 250,//40,//25.0,
	// .pitch_pid.position.ki = 0,
	// .pitch_pid.position.kd = 13000,//60,//0,
	// .pitch_pid.position.ioutput_max = 1000,
	// .pitch_pid.position.output_max = 9999,
	// .pitch_pid.position.mode = PID_POSITION,	
	
	// .pitch_pid.speed.kp = 60,//30,//25.0,
	// .pitch_pid.speed.ki = 0.5,//0,
	// .pitch_pid.speed.kd = 0,
	// .pitch_pid.speed.ioutput_max = 1000,
	// .pitch_pid.speed.output_max = 9999,
	// .pitch_pid.speed.mode = PID_POSITION,	
 
/***************************2��pitch**************************************/

	.pitch1_pid.position.kp =7,
	.pitch1_pid.position.ki = 0,
	.pitch1_pid.position.kd =5,//���ܸ�̫�󣬸�̫���������Ӱ���𵴵�����ȣ�̫СС��������������
	.pitch1_pid.position.ioutput_max = 1000,
	.pitch1_pid.position.output_max = 1000,
	.pitch1_pid.position.mode = PID_POSITION,	
	 
	.pitch1_pid.speed.kp = 70,
	.pitch1_pid.speed.ki = 5,
	.pitch1_pid.speed.kd = 10,
	.pitch1_pid.speed.ioutput_max = 12000,
	.pitch1_pid.speed.output_max = 15000,
	.pitch1_pid.speed.mode = PID_POSITION, 	
 
	.pitch2_pid.position.kp =7,
	.pitch2_pid.position.ki = 0,
	.pitch2_pid.position.kd =10,
	.pitch2_pid.position.ioutput_max = 1000,
	.pitch2_pid.position.output_max = 1000,
	.pitch2_pid.position.mode = PID_POSITION,	
	
	.pitch2_pid.speed.kp =80,
	.pitch2_pid.speed.ki =7,
	.pitch2_pid.speed.kd = 10,
	.pitch2_pid.speed.ioutput_max = 12000,
	.pitch2_pid.speed.output_max = 15000,
	.pitch2_pid.speed.mode = PID_POSITION,	

};

cali_shoot_t cali_shoot_pid = 
{
/**
* @brief trigger pid param         
* @note    
*/
	.trigger_pid.position.kp = 20.0,//30Խ����������Խ�죬����Ҫ��ʱ����Ʒ����ٶ�
	.trigger_pid.position.ki = 0.0,
	.trigger_pid.position.kd = 0.0,
	.trigger_pid.position.ioutput_max = 1000,
	.trigger_pid.position.output_max = 9999,
	.trigger_pid.position.mode = PID_POSITION,	
	 
	.trigger_pid.speed.kp = 100.0,//100��������Ӧ�ٶȣ����ܸ�̫С
	.trigger_pid.speed.ki = 0.0,
	.trigger_pid.speed.kd = 0.0,
	.trigger_pid.speed.ioutput_max = 1000,
	.trigger_pid.speed.output_max = 9999,
	.trigger_pid.speed.mode = PID_POSITION,	
/**
* @brief fric1 pid param         
* @note    
*/	
	.fric1_pid.kp = 20.0,
	.fric1_pid.ki = 1.0,
	.fric1_pid.kd = 0.0,
	.fric1_pid.ioutput_max = 1000,
	.fric1_pid.output_max = 9999,
	.fric1_pid.mode = PID_POSITION,	//PID_DELTA PID_POSITION

/**
* @brief fric2 pid param        
* @note    
*/	
	.fric2_pid.kp = 20.0,
	.fric2_pid.ki = 1.0,
	.fric2_pid.kd = 0.0,
	.fric2_pid.ioutput_max = 1000,
	.fric2_pid.output_max = 9999,
	.fric2_pid.mode = PID_POSITION,	

};


cali_chassis_t cali_chassis_pid = 
{
/**
* @brief fric1 pid param         
* @note    
*/ 
	.cm_pid.kp = 20.0,  //6.2  //10stable  20
	.cm_pid.ki = 0.0,  //0.3   //0
	.cm_pid.kd = 0.0,
	.cm_pid.ioutput_max = 1000,
	.cm_pid.output_max = 5000,
	.cm_pid.mode = PID_POSITION,			//PID_DELTA	PID_POSITION
	
	.rotate_pid.kp = 0.2,
	.rotate_pid.ki = 0.0,
	.rotate_pid.kd = 0.0,
	.rotate_pid.ioutput_max = 1000,
	.rotate_pid.output_max = 5000,
	.rotate_pid.mode = PID_POSITION,	//PID_POSITION	//������ȷ�� Ŀǰ�����ܴ�
};

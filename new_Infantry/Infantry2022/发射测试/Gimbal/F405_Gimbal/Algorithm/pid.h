#ifndef __PID_H
#define __PID_H

typedef struct{
		float K1;			
	  float K2;
	  float Last_DeltIn;
	  float Now_DeltIn;
	  float Out;
}FeedForward_Typedef;

/*   PID调参-----ZN方法：
		 Td/T = Kd  T/Ti = Ki   
     PID:   Ti >= 4Td    Kp = 0.6Ku   Td = 0.125Tu
     PI:    Kp = 0.45Ku  Ti = 0.83Tu
     P:     Kp = 0.5Ku
*/


typedef struct PID{
		float SetPoint;			//设定目标值
	  float SetPointLast;
		float ActualValue;

    float DeadZone;
		float P;						//比例常数
		float I;						//积分常数
		float D;						//微分常数
		
		float LastError;		//前次误差
		float PreError;			//当前误差
		float SumError;			//积分误差
		float dError;
	
		float IMax;					//积分限制
		
		float POut;					//比例输出
		float IOut;					//积分输出
		float DOut;					//微分输出
		float OutMax;       //限幅
}Pid_Typedef;

float PID_Calc(Pid_Typedef * P, float ActualValue);

/*********模糊pid部分*/
typedef struct
{
		float SetPoint;			//设定目标值
	  float ActPoint;      //实际值
		float LastError;		//前次误差
		float PreError;			//当前误差
		float SumError;			//积分误差
		float dError;
	
		float IMax;					//积分限制
		
		float POut;					//比例输出
		float IOut;					//积分输出
		float DOut;					//微分输出
	
	  float Kp;       
	  float Ki;
  	float Kd;
	  
}FuzzyPID;
float FuzzyPID_Calc(FuzzyPID *pid);
float FeedForward_Calc(FeedForward_Typedef *FF);
#endif

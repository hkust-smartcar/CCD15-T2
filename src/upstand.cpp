/**
 * =====================================================================================
 *        COPYRIGHT NOTICE
 *        Copyright (c) 2012  HUST-Renesas Lab
 *        ALL rights reserved.
 *
 *        @file     upstand_signal.c
 *
 *        @brief    直立信号处理
 *
 *        @version  0.1
 *        @date     2012/5/22 11:27:35
 *
 *        @author:  Liu Bojie , 313195046@qq.com
 * =====================================================================================
 *  @0.1    Liu Bojie   2012/5/22   create original file
 * =====================================================================================
 */
#include <car.h>
#include <array>
#include <cmath>
#include "upstand.h"

/**
 *    @brief  卡尔曼滤波器函数，处理角度信息融合过程
 *
 *    @param  TIME_CONSTANT       采样间隔时间常数（单位：ms）
 *    @param  ACCY_CONVARIANCE    加速度计Y轴协方差
 *    @param  GYRO_CONVARIANCE    陀螺仪协方差
 *    @param  ACCY_AD2DEG_RATIO   加速度计角度转换比例
 *    @param  GYRO_AD2DEG_RATIO   陀螺仪角度转换比例
 *    @param  ANGLE_ZERO          加速度计静态零点设置值（粗调）
 *    @param  ANGLE_ZERO_FIXTURE  加速度计静态零点设置值（细调）
 *
 *    @note   参数体现在宏定义里，只针对不同的车体模型做微调
 */
void Upstand::KalmanFilter(void)
{
	  /* 函数内部变量定义 */
	            float Q  = 0;
	            static         float R  = 0;
	            float KalmanGain         = 0;
	            float Priori_Estimation  = 0;
	            float Priori_Convariance = 0;
      static   float gyro_angle = 0;
	  volatile float Accelerometer = 0;
	  volatile float Gyroscope     = 0;
	  volatile float AngleGyro     = 0;
	  volatile float AngleAcc      = 0;
	  static   float Posterior_Estimation   = 0;
	  static   float Posterior_Convariance  = 0;
	            float Priori_Estimation_Real = 0;
//	  volatile float GyroscopeReal = 0;
	  volatile float AngleGyroReal = 0;
	  static   float Posterior_Estimation_Real  = 0;

	  /* 基本参数赋值 */
//	  Timer::TimerInt t = System::Time();
//	  static Timer::TimerInt pt = 0;
      float dt = 4/1000.0f;
//      pt = t;
	  Q  = GYRO_CONVARIANCE;
	  R  = (float)5000.0f/*ACCY_CONVARIANCE*/;

	  /* 传感器取值 */
//	  std::array<float, 3> omega_ = m_mpu->GetOmega();
//	  std::array<float, 3> accel_ = m_mpu->GetAccel();
//	  m_gyro_ad = (int32_t)-omega_[1];
//	  m_acc_ad = (int32_t)accel_[2];

	  /* 设定零点 */
//	  Accelerometer = accel_[2]/*(float)(ANGLE_ZERO) - (float)m_acc_ad*/;
	  Accelerometer = m_acc_adc->GetResultF();
//     Gyroscope     = ((float)gl_gyro_zero - (float)gl_gyro_ad + ((float)gl_speed_fb_fix - (float)gl_speed_lr_fix)/ 10.0);      /* 加速度计角度转化 */
//	  if(Accelerometer > 1.0f){
//		  Accelerometer = 1.0f;
//	  }else if(Accelerometer < -1.0f){
//		  Accelerometer = -1.0f;
//	  }
	  AngleAcc  = /*RAD2ANGLE * asin(*/Accelerometer/*)*/;

      /* 陀螺仪角度转化 */
//      AngleGyro = -omega_[1] / 2.1;
	  AngleGyro = m_gyro_adc->GetResultF();
	  m_angle_gyro = AngleGyro;


      /* 卡尔曼先验估计：时间更新 */
      /* Priori Estimation : X(k|k-1) = A(k,k-1)*X(k-1|k-1) + B(k)*u(k) */
      Priori_Estimation  = Posterior_Estimation + AngleGyro * dt;
       gyro_angle += AngleGyro * dt;
       Priori_Estimation_Real = Posterior_Estimation_Real + AngleGyro * dt;/*#*/
       /* Update Estimation Convariance : P(k|k-1) = A(k,k-1)*P(k-1|k-1)*A(k,k-1)'+Q(k) */
       Priori_Convariance = sqrt( Posterior_Convariance * Posterior_Convariance + Q * Q );

       /* 卡尔曼后验估计：测量更新 */
       /* Calculation of Kalman Gain : K(k) = P(k|k-1)*H(k)' / (H(k)*P(k|k-1)*H(k)' + R(k)) */
       KalmanGain = sqrt( Priori_Convariance * Priori_Convariance / ( Priori_Convariance * Priori_Convariance + R * R ) );
       /* Posterior Estimation: X(k|k) = X(k|k-1)+K(k)*(Z(k)-H(k)*X(k|k-1)) */
       Posterior_Estimation  = Priori_Estimation + KalmanGain * ( AngleAcc - Priori_Estimation );
       Posterior_Estimation_Real  = Priori_Estimation_Real + KalmanGain * ( AngleAcc - Priori_Estimation_Real );/*#*/
       /* Update Posterior Convariance : P(k|k) =（I-K(k)*H(k)）*P(k|k-1) */
       Posterior_Convariance = sqrt( ( 1 -KalmanGain ) * Priori_Convariance * Priori_Convariance );

      /* 实际角度输出:加速度计/陀螺仪/融合后角度 */
      m_angle_acc        = AngleAcc;

      /* 速度附加角度：陀螺仪/附加角 */
//      gl_speed_gyro       = (int32_t)( Gyroscope - GyroscopeReal );
//      gl_speed_angle      = (int32_t)( Posterior_Estimation * 10 - Posterior_Estimation_Real * 10 );

      /* 控制角度输出 */
      m_angle            = ( Posterior_Estimation );
//      m_angle_gyro       = (int32_t)( Gyroscope );

//      printvars[0] = gyro_angle * 10;
//      printvars[1] = AngleAcc * 10;
//      printvars[2] = Posterior_Estimation_Real * 10;
}

void Upstand::BalanceFilter(void)
{
//    float dt = 0;
//    float a  = 0;
//    float acc_fix = 0;
//    float gyr_fix = 0;
//
//    /* parameters of the Balance Filter */
//    dt = AD_INTERVAL;
//    a = TIME_CONSTANT / ( TIME_CONSTANT + dt );
//
//	/* 传感器取值 */
//	gl_gyro_ad = gyro_read(GYRO0);
//	gl_acc_ad  = acc_read(ACC0);
//
//	/* 设定精度 */
//	SET_ACCURACY(gl_gyro_ad,9);
//	SET_ACCURACY(gl_acc_ad, 9);
//	gl_acc = gl_acc_ad;
//	gl_gyr = gl_gyro_ad;
//
//	/* 设定零点 */
//	acc_fix = ( (float)(ANGLE_ZERO +(float)(gl_acc_fix/10.0) ) - (float)gl_acc_ad );
//	gyr_fix = ((float)gl_gyro_ad - (float)gl_gyro_zero );
//
//    /* 融合进速度控制，此处用加号 */
//    //gyr_fix += (((float)gl_speed_lr_fix  + (float)gl_speed_fb_fix ) / 10) ;
//
//    /* calculate angle and angular speed */
//    gl_acc = acc_fix * ACCY_AD2DEG_RATIO;
//    gl_angle_gyro_real = gyr_fix * GYRO_AD2DEG_RATIO;
//
//    /* result of BalanceFilter */
//    gl_angle = a * (gl_angle + gl_gyr * dt) + (1-a) * gl_acc;
}

/**
 *    @brief  陀螺仪零点修正函数
 */
void Upstand::GyroDriftAutoFix(void)
{
//	static int32_t calctemp = 0;
//	static int32_t cnt      = 0;
//	static int32_t step     = 0;
//	static int32_t drift    = 0;
//	static int32_t timecnt  = 0;
//	timecnt++;
//	cnt++;
//	switch(gl_angle_gyro_real)
//	{
//		case -3:
//			light_set(0x80);
//			break;
//		case -2:
//			light_set(0x40);
//			break;
//		case -1:
//			light_set(0x20);
//			break;
//		case 0:
//			light_set(0x18);
//			break;
//		case 1:
//			light_set(0x04);
//			break;
//		case 2:
//			light_set(0x02);
//			break;
//		case 3:
//			light_set(0x01);
//			break;
//		default:
//			light_set(0xff);
//			break;
//	}
//
//	switch(step)
//	{
//		case 0:
//			/* 标定开关去抖动 */
//			if(cnt>500)
//			{
//				step = 1;
//				cnt  = 0;
//			}
//			break;
//		case 1:
//			/* 快速调整零点 */
//			calctemp += gl_gyro_ad;
//			if(cnt == 100)
//			{
//				gl_gyro_zero = calctemp / 100;
//				cnt = 0;
//				calctemp = 0;
//				step = 2;
//			}
//			break;
//		case 2:
//			/* 漂移累计条件限制 */
//			drift += gl_angle_gyro_real;
//			if(cnt>300)
//			{
//				if(drift > 100)
//				{
//					gl_gyro_zero--;
//					drift = 0;
//				}
//				if(drift < -100)
//				{
//					gl_gyro_zero++;
//					drift = 0;
//				}
//				drift = 0;
//				cnt = 0;
//			}
//			if((cnt>150)&&(drift<40)&&(drift>-40))
//			{
//				step = 3;
//				drift = 0;
//				cnt = 0;
//			}
//			break;
//		case 3:
//			if(cnt > 400)
//			{
//				gl_control_drift_fix = DISABLE;
//				drift = 0;
//				cnt = 0;
//				step = 0;
//				light_set(0xaa);
//			}
//			break;
//		default:
//			break;
//	}
//	if(timecnt > 4000)
//	{
//		gl_control_drift_fix = DISABLE;
//		drift = 0;
//		cnt = 0;
//		timecnt = 0;
//		step = 0;
//		light_set(0xaa);
//	}
}



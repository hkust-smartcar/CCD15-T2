/**
 * =====================================================================================
 *        COPYRIGHT NOTICE
 *        Copyright (c) 2012  HUST-Renesas Lab
 *        ALL rights reserved.
 *
 *        @file     upstand_signal.c
 *
 *        @brief    ֱ���źŴ���
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
 *    @brief  �������˲�������������Ƕ���Ϣ�ںϹ���
 *
 *    @param  TIME_CONSTANT       �������ʱ�䳣������λ��ms��
 *    @param  ACCY_CONVARIANCE    ���ٶȼ�Y��Э����
 *    @param  GYRO_CONVARIANCE    ������Э����
 *    @param  ACCY_AD2DEG_RATIO   ���ٶȼƽǶ�ת������
 *    @param  GYRO_AD2DEG_RATIO   �����ǽǶ�ת������
 *    @param  ANGLE_ZERO          ���ٶȼƾ�̬�������ֵ���ֵ���
 *    @param  ANGLE_ZERO_FIXTURE  ���ٶȼƾ�̬�������ֵ��ϸ����
 *
 *    @note   ���������ں궨���ֻ��Բ�ͬ�ĳ���ģ����΢��
 */
void Upstand::KalmanFilter(void)
{
	  /* �����ڲ��������� */
	            float Q  = 0;
	            static         float R  = 0;
	            float KalmanGain         = 0;
	            float Priori_Estimation  = 0;
	            float Priori_Convariance = 0;
      static   float gyro_angle = 0;
	  volatile float Accelerometer = 0;
//	  volatile float Gyroscope     = 0;
	  volatile float AngleGyro     = 0;
	  volatile float AngleAcc      = 0;
	  static   float Posterior_Estimation   = 0;
	  static   float Posterior_Convariance  = 0;
	            float Priori_Estimation_Real = 0;
//	  volatile float GyroscopeReal = 0;
//	  volatile float AngleGyroReal = 0;
	  static   float Posterior_Estimation_Real  = 0;

	  /* ����������ֵ */
//	  Timer::TimerInt t = System::Time();
//	  static Timer::TimerInt pt = 0;
      float dt = 10/1000.0f;
//      pt = t;
	  Q  = GYRO_CONVARIANCE;
	  R  = ACCY_CONVARIANCE;

	  /* ������ȡֵ */
	  std::array<float, 3> omega_ = m_mpu->GetOmega();
	  std::array<float, 3> accel_ = m_mma->GetAccel();
//	  m_gyro_ad = (int32_t)-omega_[1];
//	  m_acc_ad = (int32_t)accel_[2];

	  /* �趨��� */
	  Accelerometer = accel_[1]/*(float)(ANGLE_ZERO) - (float)m_acc_ad*/;
//	  Accelerometer = m_acc_adc->GetResultF();
//     Gyroscope     = ((float)gl_gyro_zero - (float)gl_gyro_ad + ((float)gl_speed_fb_fix - (float)gl_speed_lr_fix)/ 10.0);      /* ���ٶȼƽǶ�ת�� */
	  if(Accelerometer > 1.0f){
		  Accelerometer = 1.0f;
	  }else if(Accelerometer < -1.0f){
		  Accelerometer = -1.0f;
	  }
	  AngleAcc  = RAD2ANGLE * asin(Accelerometer);

      /* �����ǽǶ�ת�� */
      AngleGyro = omega_[1];
//	  AngleGyro = m_gyro_adc->GetResultF();
	  m_angle_gyro = AngleGyro + 1.0f;


      /* ������������ƣ�ʱ����� */
      /* Priori Estimation : X(k|k-1) = A(k,k-1)*X(k-1|k-1) + B(k)*u(k) */
      Priori_Estimation  = Posterior_Estimation + AngleGyro * dt;
       gyro_angle += AngleGyro * dt;
       Priori_Estimation_Real = Posterior_Estimation_Real + AngleGyro * dt;/*#*/
       /* Update Estimation Convariance : P(k|k-1) = A(k,k-1)*P(k-1|k-1)*A(k,k-1)'+Q(k) */
       Priori_Convariance = sqrt( Posterior_Convariance * Posterior_Convariance + Q * Q );

       /* ������������ƣ��������� */
       /* Calculation of Kalman Gain : K(k) = P(k|k-1)*H(k)' / (H(k)*P(k|k-1)*H(k)' + R(k)) */
       KalmanGain = sqrt( Priori_Convariance * Priori_Convariance / ( Priori_Convariance * Priori_Convariance + R * R ) );
       /* Posterior Estimation: X(k|k) = X(k|k-1)+K(k)*(Z(k)-H(k)*X(k|k-1)) */
       Posterior_Estimation  = Priori_Estimation + KalmanGain * ( AngleAcc - Priori_Estimation );
       Posterior_Estimation_Real  = Priori_Estimation_Real + KalmanGain * ( AngleAcc - Priori_Estimation_Real );/*#*/
       /* Update Posterior Convariance : P(k|k) =��I-K(k)*H(k)��*P(k|k-1) */
       Posterior_Convariance = sqrt( ( 1 -KalmanGain ) * Priori_Convariance * Priori_Convariance );

      /* ʵ�ʽǶ����:���ٶȼ�/������/�ںϺ�Ƕ� */
      m_angle_acc        = AngleAcc;

      /* �ٶȸ��ӽǶȣ�������/���ӽ� */
//      gl_speed_gyro       = (int32_t)( Gyroscope - GyroscopeReal );
//      gl_speed_angle      = (int32_t)( Posterior_Estimation * 10 - Posterior_Estimation_Real * 10 );

      /* ���ƽǶ���� */
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
//	/* ������ȡֵ */
//	gl_gyro_ad = gyro_read(GYRO0);
//	gl_acc_ad  = acc_read(ACC0);
//
//	/* �趨���� */
//	SET_ACCURACY(gl_gyro_ad,9);
//	SET_ACCURACY(gl_acc_ad, 9);
//	gl_acc = gl_acc_ad;
//	gl_gyr = gl_gyro_ad;
//
//	/* �趨��� */
//	acc_fix = ( (float)(ANGLE_ZERO +(float)(gl_acc_fix/10.0) ) - (float)gl_acc_ad );
//	gyr_fix = ((float)gl_gyro_ad - (float)gl_gyro_zero );
//
//    /* �ںϽ��ٶȿ��ƣ��˴��üӺ� */
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
 *    @brief  �����������������
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
//			/* �궨����ȥ���� */
//			if(cnt>500)
//			{
//				step = 1;
//				cnt  = 0;
//			}
//			break;
//		case 1:
//			/* ���ٵ������ */
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
//			/* Ư���ۼ��������� */
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



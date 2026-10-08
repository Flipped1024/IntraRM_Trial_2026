#include "bsp_mpu6050.h"
#include "i2c.h"
#include "app_debug.h"
#include <math.h>

#define MPU_ADDR (0x68U << 1)
#define REG_CONFIG 0x1AU
#define REG_GYRO_CONFIG 0x1BU
#define REG_ACCEL_CONFIG 0x1CU
#define REG_INT_PIN_CFG 0x37U
#define REG_INT_ENABLE 0x38U
#define REG_ACCEL_XOUT_H 0x3BU
#define REG_PWR_MGMT_1 0x6BU
#define REG_WHO_AM_I 0x75U

static float gyro_bias_[3], q_[4] = {1.0f, 0.0f, 0.0f, 0.0f};
static float mahony_integral_[3], heater_integral_;
static uint16_t bias_samples_;
static uint32_t last_tick_;
static float acc_filter_[3], gyro_filter_[3];
static uint8_t filter_initialized_;
static float yaw_unwrapped_, yaw_prev_euler_;
static uint8_t yaw_initialized_;
static float gyro_integral_deg_[3];
static uint8_t yaw_axis_ = 2U;
static int8_t yaw_axis_sign_ = 1;

static HAL_StatusTypeDef writeReg(uint8_t r, uint8_t v) { return HAL_I2C_Mem_Write(&hi2c2, MPU_ADDR, r, I2C_MEMADD_SIZE_8BIT, &v, 1U, 20U); }
static HAL_StatusTypeDef readRegs(uint8_t r, uint8_t *d, uint16_t n) { return HAL_I2C_Mem_Read(&hi2c2, MPU_ADDR, r, I2C_MEMADD_SIZE_8BIT, d, n, 20U); }

HAL_StatusTypeDef mpu6050Init(void)
{
    uint8_t who = 0U, gyro_cfg = 0U, accel_cfg = 0U;
    if (HAL_I2C_IsDeviceReady(&hi2c2, MPU_ADDR, 3U, 20U) != HAL_OK || readRegs(REG_WHO_AM_I, &who, 1U) != HAL_OK || (who & 0x7EU) != 0x68U || writeReg(REG_PWR_MGMT_1, 0x80U) != HAL_OK) { g_debug_.imu_.i2c_error_=1U; return HAL_ERROR; }
    HAL_Delay(100U);
    /* Supplied HAL example: DLPF 21 Hz, +/-4 g, +/-2000 dps, adapted to 100 Hz. */
    if (writeReg(REG_PWR_MGMT_1,0x01U)!=HAL_OK || writeReg(REG_CONFIG,0x04U)!=HAL_OK || writeReg(0x19U,9U)!=HAL_OK || writeReg(REG_GYRO_CONFIG,0x18U)!=HAL_OK || writeReg(REG_ACCEL_CONFIG,0x08U)!=HAL_OK || writeReg(REG_INT_PIN_CFG,0x10U)!=HAL_OK || writeReg(REG_INT_ENABLE,0x01U)!=HAL_OK) { g_debug_.imu_.i2c_error_=1U; return HAL_ERROR; }
    gyro_bias_[0]=gyro_bias_[1]=gyro_bias_[2]=0.0f;
    /* Match the supplied example: collect a stationary gyro bias before the
     * first attitude update. Keep the board still during this ~1 s phase. */
    {
        int32_t sum[3] = {0, 0, 0};
        int32_t acc_sum[3] = {0, 0, 0};
        uint8_t ok = 1U;
        for (uint16_t i = 0U; i < 100U; i++) {
            uint8_t b[14];
            if (readRegs(REG_ACCEL_XOUT_H, b, sizeof(b)) != HAL_OK) { ok = 0U; break; }
            sum[0] += (int16_t)((b[8] << 8) | b[9]);
            sum[1] += (int16_t)((b[10] << 8) | b[11]);
            sum[2] += (int16_t)((b[12] << 8) | b[13]);
            acc_sum[0] += (int16_t)((b[0] << 8) | b[1]);
            acc_sum[1] += (int16_t)((b[2] << 8) | b[3]);
            acc_sum[2] += (int16_t)((b[4] << 8) | b[5]);
            HAL_Delay(10U);
        }
        if (!ok) { g_debug_.imu_.i2c_error_ = 1U; return HAL_ERROR; }
        gyro_bias_[0] = ((float)sum[0] / 100.0f) / 16.4f;
        gyro_bias_[1] = ((float)sum[1] / 100.0f) / 16.4f;
        gyro_bias_[2] = ((float)sum[2] / 100.0f) / 16.4f;
        {
            int32_t ax = acc_sum[0] < 0 ? -acc_sum[0] : acc_sum[0];
            int32_t ay = acc_sum[1] < 0 ? -acc_sum[1] : acc_sum[1];
            int32_t az = acc_sum[2] < 0 ? -acc_sum[2] : acc_sum[2];
            yaw_axis_ = 0U;
            if (ay > ax) yaw_axis_ = 1U;
            if ((yaw_axis_ == 0U ? az > ax : az > ay)) yaw_axis_ = 2U;
            yaw_axis_sign_ = ((acc_sum[yaw_axis_] >= 0) ? 1 : -1);
        }
    }
    q_[0]=1.0f; q_[1]=q_[2]=q_[3]=0.0f; mahony_integral_[0]=mahony_integral_[1]=mahony_integral_[2]=0.0f; heater_integral_=0.0f; bias_samples_=100U; filter_initialized_=0U; yaw_unwrapped_=0.0f; yaw_prev_euler_=0.0f; yaw_initialized_=0U; gyro_integral_deg_[0]=gyro_integral_deg_[1]=gyro_integral_deg_[2]=0.0f; last_tick_=HAL_GetTick(); g_debug_.imu_.yaw_axis_=yaw_axis_; g_debug_.imu_.yaw_axis_sign_=yaw_axis_sign_; g_debug_.imu_.calibration_ready_=1U;
    (void)readRegs(REG_WHO_AM_I, &who, 1U);
    (void)readRegs(REG_GYRO_CONFIG, &gyro_cfg, 1U);
    (void)readRegs(REG_ACCEL_CONFIG, &accel_cfg, 1U);
    g_debug_.imu_.who_am_i_ = who;
    g_debug_.imu_.gyro_config_reg_ = gyro_cfg;
    g_debug_.imu_.accel_config_reg_ = accel_cfg;
    g_debug_.imu_.initialized_=1U; g_debug_.imu_.online_=1U; g_debug_.imu_.i2c_error_=0U; return HAL_OK;
}

HAL_StatusTypeDef mpu6050ReadRaw(Mpu6050Raw *raw)
{
    uint8_t b[14]; if(raw==NULL || readRegs(REG_ACCEL_XOUT_H,b,sizeof(b))!=HAL_OK) return HAL_ERROR;
    raw->accel[0]=(int16_t)((b[0]<<8)|b[1]); raw->accel[1]=(int16_t)((b[2]<<8)|b[3]); raw->accel[2]=(int16_t)((b[4]<<8)|b[5]); raw->temperature=(int16_t)((b[6]<<8)|b[7]); raw->gyro[0]=(int16_t)((b[8]<<8)|b[9]); raw->gyro[1]=(int16_t)((b[10]<<8)|b[11]); raw->gyro[2]=(int16_t)((b[12]<<8)|b[13]); return HAL_OK;
}

static float invSqrt(float x){return 1.0f/sqrtf(x);} static float wrapAngle(float x){while(x>=180.0f)x-=360.0f;while(x<-180.0f)x+=360.0f;return x;}

void mpu6050Update(void)
{
    Mpu6050Raw r; uint32_t now; float dt,ax,ay,az,gx,gy,gz,m2,n,ex,ey,ez,vx,vy,vz,q0,q1,q2,q3,qd0,qd1,qd2,qd3,e;
    if(!g_debug_.imu_.initialized_ || mpu6050ReadRaw(&r)!=HAL_OK){g_debug_.imu_.online_=0U;g_debug_.imu_.i2c_error_=1U;g_debug_.imu_.error_count_++;return;}
    now=HAL_GetTick();dt=(float)(now-last_tick_)*0.001f;last_tick_=now;if(dt<=0.0f||dt>0.05f)dt=0.01f;g_debug_.imu_.dt_s_=dt;
    ax=(float)r.accel[0]/8192.0f;ay=(float)r.accel[1]/8192.0f;az=(float)r.accel[2]/8192.0f;gx=((float)r.gyro[0]/16.4f-gyro_bias_[0])*0.01745329252f;gy=((float)r.gyro[1]/16.4f-gyro_bias_[1])*0.01745329252f;gz=((float)r.gyro[2]/16.4f-gyro_bias_[2])*0.01745329252f;
    { float aa=dt/(1.0f/(2.0f*3.1415926535f*48.0f)+dt), ag=dt/(1.0f/(2.0f*3.1415926535f*88.0f)+dt); if(!filter_initialized_){acc_filter_[0]=ax;acc_filter_[1]=ay;acc_filter_[2]=az;gyro_filter_[0]=gx;gyro_filter_[1]=gy;gyro_filter_[2]=gz;filter_initialized_=1U;}else{acc_filter_[0]+=aa*(ax-acc_filter_[0]);acc_filter_[1]+=aa*(ay-acc_filter_[1]);acc_filter_[2]+=aa*(az-acc_filter_[2]);gyro_filter_[0]+=ag*(gx-gyro_filter_[0]);gyro_filter_[1]+=ag*(gy-gyro_filter_[1]);gyro_filter_[2]+=ag*(gz-gyro_filter_[2]);}ax=acc_filter_[0];ay=acc_filter_[1];az=acc_filter_[2];gx=gyro_filter_[0];gy=gyro_filter_[1];gz=gyro_filter_[2]; }
    gyro_integral_deg_[0] += gx * dt * 57.2957795f; gyro_integral_deg_[1] += gy * dt * 57.2957795f; gyro_integral_deg_[2] += gz * dt * 57.2957795f;
    m2=ax*ax+ay*ay+az*az;g_debug_.imu_.accel_norm_g_=sqrtf(m2);q0=q_[0];q1=q_[1];q2=q_[2];q3=q_[3];
    if(m2>0.01f){n=invSqrt(m2);ax*=n;ay*=n;az*=n;vx=2.0f*(q1*q3-q0*q2);vy=2.0f*(q0*q1+q2*q3);vz=q0*q0-q1*q1-q2*q2+q3*q3;ex=ay*vz-az*vy;ey=az*vx-ax*vz;ez=ax*vy-ay*vx;if(m2>0.64f&&m2<1.44f){mahony_integral_[0]+=ex*dt*0.0015f;mahony_integral_[1]+=ey*dt*0.0015f;mahony_integral_[2]+=ez*dt*0.0015f;gx+=4.8f*ex+mahony_integral_[0];gy+=4.8f*ey+mahony_integral_[1];gz+=4.8f*ez+mahony_integral_[2];}else{gx+=3.6f*ex;gy+=3.6f*ey;gz+=3.6f*ez;}}
    qd0=.5f*(-q1*gx-q2*gy-q3*gz);qd1=.5f*(q0*gx+q2*gz-q3*gy);qd2=.5f*(q0*gy-q1*gz+q3*gx);qd3=.5f*(q0*gz+q1*gy-q2*gx);q0+=qd0*dt;q1+=qd1*dt;q2+=qd2*dt;q3+=qd3*dt;n=invSqrt(q0*q0+q1*q1+q2*q2+q3*q3);q_[0]=q0*n;q_[1]=q1*n;q_[2]=q2*n;q_[3]=q3*n;
    { float yaw_now=wrapAngle(atan2f(2.0f*(q_[0]*q_[3]+q_[1]*q_[2]),1.0f-2.0f*(q_[2]*q_[2]+q_[3]*q_[3]))*57.2957795f); if(!yaw_initialized_){yaw_prev_euler_=yaw_now;yaw_initialized_=1U;}else{yaw_unwrapped_+=wrapAngle(yaw_now-yaw_prev_euler_);yaw_prev_euler_=yaw_now;} g_debug_.imu_.yaw_euler_deg_=yaw_now;g_debug_.imu_.yaw_deg_=gyro_integral_deg_[yaw_axis_]*yaw_axis_sign_; }
    g_debug_.imu_.roll_deg_=atan2f(2.0f*(q_[0]*q_[1]+q_[2]*q_[3]),1.0f-2.0f*(q_[1]*q_[1]+q_[2]*q_[2]))*57.2957795f;g_debug_.imu_.pitch_deg_=asinf(fmaxf(-1.0f,fminf(1.0f,2.0f*(q_[0]*q_[2]-q_[3]*q_[1]))))*57.2957795f;g_debug_.imu_.yaw_rate_dps_=gz*57.2957795f;g_debug_.imu_.yaw_bias_dps_=gyro_bias_[2];
    for(uint8_t i=0;i<4U;i++){g_debug_.imu_.quat_[i]=q_[i];}g_debug_.imu_.temperature_c_=36.53f+(float)r.temperature/340.0f;g_debug_.imu_.temp_raw_=r.temperature;e=g_debug_.imu_.heater_temp_target_c_-g_debug_.imu_.temperature_c_;heater_integral_+=e*dt;if(heater_integral_>100.0f)heater_integral_=100.0f;if(heater_integral_<-100.0f)heater_integral_=-100.0f;g_debug_.imu_.heater_pid_output_=20.0f*e+0.15f*heater_integral_;if(g_debug_.imu_.heater_pid_output_<0.0f)g_debug_.imu_.heater_pid_output_=0.0f;if(g_debug_.imu_.heater_pid_output_>999.0f)g_debug_.imu_.heater_pid_output_=999.0f;TIM15->CCR2=(uint32_t)g_debug_.imu_.heater_pid_output_;g_debug_.imu_.heater_pwm_=(uint16_t)g_debug_.imu_.heater_pid_output_;
    for(uint8_t i=0;i<3U;i++){g_debug_.imu_.accel_raw_[i]=r.accel[i];g_debug_.imu_.gyro_raw_[i]=r.gyro[i];{int16_t p=(r.gyro[i]<0)?(int16_t)-r.gyro[i]:r.gyro[i];if(p>g_debug_.imu_.gyro_peak_raw_[i])g_debug_.imu_.gyro_peak_raw_[i]=p;}g_debug_.imu_.accel_g_[i]=acc_filter_[i];g_debug_.imu_.gyro_dps_[i]=(i==0U)?gx*57.2957795f:(i==1U)?gy*57.2957795f:gz*57.2957795f;g_debug_.imu_.gyro_integral_deg_[i]=gyro_integral_deg_[i];}g_debug_.imu_.gyro_raw_x_=r.gyro[0];g_debug_.imu_.gyro_raw_y_=r.gyro[1];g_debug_.imu_.gyro_raw_z_=r.gyro[2];g_debug_.imu_.gyro_z_dps_=gz*57.2957795f;g_debug_.imu_.yaw_rate_dps_=((yaw_axis_==0U)?gx:((yaw_axis_==1U)?gy:gz))*57.2957795f*yaw_axis_sign_;g_debug_.imu_.data_ready_=1U;g_debug_.imu_.online_=1U;g_debug_.imu_.i2c_error_=0U;g_debug_.imu_.sample_count_++;
}

void mpu6050SetTemperatureTarget(float target_c){if(target_c<20.0f)target_c=20.0f;if(target_c>60.0f)target_c=60.0f;g_debug_.imu_.heater_temp_target_c_=target_c;}

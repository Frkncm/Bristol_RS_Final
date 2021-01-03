#ifndef _COMPLEMENTARY_FILTER_
#define _COMPLEMENTARY_FILTER_

#include "bsp.h"

#define WEIGHT 0.93

class ComplementFilter {

  private:
    uint32_t timer_;

    /* IMU Data */
    double accX_, accY_, accZ_;
    double gyroX_, gyroY_, gyroZ_;
    double gyroXangle_, gyroYangle_, gyroZangle_; // Angle calculate using the gyro only
    double compAngleX_, compAngleY_, compAngleZ_; // Calculated angle using a complementary filter
    double kalAngleX_, kalAngleY_, kalAngleZ_; // Calculated angle using a Kalman filter
    float weight_combination = WEIGHT;

  public:

    float calibration_X = 0;
    float calibration_Y = 0;
    float calibration_Z = 0;

    ComplementFilter() {
      timer_ = micros();
    }

    void setWeight(float w) {
      if ( w > 1 || w < 0)
        return;
      weight_combination = w;
    }

    float getFilteredX() {
      return compAngleX_;
    }

    float getFilteredY() {
      return compAngleY_ - calibration_Y;
    }

    float getFilteredZ() {
      return compAngleZ_ - calibration_Z;
    }

    void cal_compl_params(uint16_t cal_sample_rate) {
      float cal_val_X = 0.0f;
      float cal_val_Y = 0.0f;
      float cal_val_Z = 0.0f;
      //Calibrate the sensor
      for (int i = 0; i < cal_sample_rate; i++) {
        //wait till sensor stabilize
        while (!imu.readReg(LSM6::STATUS_REG) & 0x08);
        updateImuValues();
        updateFilter();
        cal_val_X += getFilteredX();
        cal_val_Y += getFilteredY();
        cal_val_Z += getFilteredZ();
      }
      cal_val_X /= (float)cal_sample_rate;
      cal_val_Y /= (float)cal_sample_rate;
      cal_val_Z /= (float)cal_sample_rate;

      calibration_X = cal_val_X;
      calibration_Y = cal_val_Y;
      calibration_Z = cal_val_Z;
    }

    void updateImuValues() {
      imu.read();
      accX_ = imu.a.x;
      accY_ = imu.a.y;
      accZ_ = imu.a.z;

      gyroX_ = imu.g.x;
      gyroY_ = imu.g.y;
      gyroZ_ = imu.g.z;
    }

    void updateFilter() {

      updateImuValues();

      double dt_ = (double)(micros() - timer_) / 1000000.0; // Calculate delta time
      timer_ = micros();

      //float roll  = atan2(accY_, accZ_) * RAD_TO_DEG;
      //float pitch = atan(-accX_ / sqrt(accY_ * accY_ + accZ_ * accZ_)) * RAD_TO_DEG;
      float pitch = atan (accX_ / sqrt(accY_ * accY_ + accZ_ * accZ_)) * RAD_TO_DEG;
      float roll =  atan (accY_ / sqrt(accX_ * accX_ + accZ_ * accZ_)) * RAD_TO_DEG;
      float yaw =   atan (accZ_ / sqrt(accX_ * accX_ + accZ_ * accZ_)) * RAD_TO_DEG;

      double gyroXrate_ = gyroX_ * 0.00875; // Convert to deg/s
      double gyroYrate_ = gyroY_ * 0.00875; // Convert to deg/s
      double gyroZrate_ = gyroZ_ * 0.00875; // Convert to deg/s

      gyroXangle_ += gyroXrate_ * dt_; // Calculate gyro angle without any filter
      gyroYangle_ += gyroYrate_ * dt_;
      gyroZrate_  += gyroZrate_ * dt_;

      // Calculate the angle using a Complimentary filter
      compAngleX_ = weight_combination * (compAngleX_ + gyroXrate_ * dt_) + (1 - weight_combination) * roll;
      compAngleY_ = weight_combination * (compAngleY_ + gyroYrate_ * dt_) + (1 - weight_combination) * pitch;
      compAngleZ_ = weight_combination * (compAngleZ_ + gyroZrate_ * dt_) + (1 - weight_combination) * yaw;

      if (gyroXangle_ < -180 || gyroXangle_ > 180)
        gyroXangle_ = kalAngleX_;
      if (gyroYangle_ < -180 || gyroYangle_ > 180)
        gyroYangle_ = kalAngleY_;
      if (gyroZangle_ < -180 || gyroZangle_ > 180)
        gyroZangle_ = kalAngleZ_;

    }
};

#endif



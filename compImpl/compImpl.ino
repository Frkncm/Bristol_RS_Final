#include "complementary.hpp"
#include "nonBlockingMillis.h"
#include "motor.hpp"
#include "pidLib.h"
#include "imuCalc.hpp"
#include "Romi.h"
#include "bsp.h"
#include "kinematics.h"

#define _SPEED_ 60
#define SMOOTH_STOP -_SPEED_/4
#define ACCEL_START _SPEED_/3

#define KP_VAL 45.0  // increase response time, but it'll increase oscillation also
#define KI_VAL 0.3   // minimise the total error 
#define KD_VAL 1.0   // if there is a huge changing

#define SQUARE_DISTANCE 500

#define BLUETOOTH_SEND 0

LSM6 imu;
ComplementFilter cmp;
imuCalculator imclc;
Kinematics_c knm;

//Bluetooth instance
HardwareSerial& bthc05(Serial1);

//These variables for handling the finite state machine
uint8_t current_state;
uint8_t next_state = IDLE_STATE;
uint32_t blocking_time {0};

//right and left motor instances
myMotor leftMotor(L_DIR_PIN, L_PWM_PIN);
myMotor rightMotor(R_DIR_PIN, R_PWM_PIN);

PID pidMotor(KP_VAL, KI_VAL, KD_VAL);

PID H_PID(KP_VAL, KI_VAL, KD_VAL);
PID L_PID(KP_VAL, KI_VAL, KD_VAL);
PID R_PID(KP_VAL, KI_VAL, KD_VAL);

int8_t turnRight = 1;
int8_t turnLeft  = 1;

uint8_t countTask{ 0 };

float bearing {0};

//square implementation
bool taskHandler() {
  auto driveStraight = [](float theta_demand)->float{return H_PID.updateValue(0 ,
                       (atan((theta_demand - imclc.turnRotation) * PI / 180.0)));};

  if (countTask == 0) {
    if (imclc.Xnew <= SQUARE_DISTANCE) {
      bearing = driveStraight(0);
      return false;
    }
    else {
      countTask++;
      return true;
    }
  } else if (countTask == 1) {
    if (imclc.Ynew >= -SQUARE_DISTANCE) {
      bearing = driveStraight(-90);
      return false;
    }
    else {
      countTask++;
      return true;
    }
  } else if (countTask == 2) {
    if (imclc.Xnew >= 0) {
      bearing = driveStraight(-180);
      return false;
    }
    else {
      countTask++;
      return true;
    }
  } else if (countTask == 3) {
    if (imclc.Ynew <= 0) {
      bearing = driveStraight(-270);
      return false;
    }
    else {
      countTask++;
      return true;
    }
  }
}

uint8_t loc_speed = 0;

//create the function which is called periodically
void imuTask(void);
taskInsert imuTaskIns(imuTask, 10);

void imuTask(void) {

  cmp.updateFilter();

  //get the coordinate using IMU sensor parameters
  imclc.turnUpdate();
  imclc.getCoordinate(imclc.turnRotation, cmp.getFilteredY());

  loc_speed += 3;

  if (loc_speed > _SPEED_)
    loc_speed = _SPEED_;

  leftMotor.motorControl(turnLeft * -loc_speed + bearing);
  rightMotor.motorControl(turnRight * -loc_speed - bearing);

  //Send values trough bluetooth

#if BLUETOOTH_SEND
  bthc05.print(imclc.Ynew); bthc05.print("\t");
  bthc05.print(imclc.Xnew); bthc05.print("\t");
  bthc05.print(imclc.velocity); bthc05.print("\t");
  bthc05.print(imclc.turnRotation); bthc05.println("\t");
#endif

}

void setup() {
  // put your setup code here, to run once:

  bsp_ctor();
  pidMotor.reset();

  //Let the user place the robot to calibrate values
  delay(1000);

  //Calibrate all relevant parameters
  cmp.cal_compl_params(2000);
  imclc.turnSetup(1024);

  //set the calibration value as an offset
  imclc.setOffset(cmp.calibration_Y);

  bthc05.begin(9600);
  GO_HANDLE(IDLE_STATE); // start with handling IDLE state

}

void loop() {
  //Update the time for non-blocking tasks
  taskInsert::executeTasks();
  knm.update(count_e0, count_e1);
  switch (current_state) {

    case IDLE_STATE: {
        /*Start the state machine with the idle state (do nothing here)
          GO_HANDLE macro and the others are defined in the Romi.h file.
          They are used to navigate the flow of control. */
        GO_HANDLE(PATH_TRACING);
        break;
      }

    case PATH_TRACING: {
        if (!taskHandler()) {
          turnLeft = 1;
          turnRight = 1;
          imuTaskIns.callMyTask();
          GO_HANDLE(PATH_TRACING);
        } else if (countTask >= 4) {
          BREAK_AND_GO(STOP_SYSTEM)
        } else {
          if (!imclc.stable_velocity)
            imclc.stable_velocity = true;
          //Reset the accel. value and motor speed
          leftMotor.motorControl(SMOOTH_STOP);
          rightMotor.motorControl(SMOOTH_STOP);
          GO_HANDLE(WAIT_BEFORE_TURNING);
        }

        break;
      }

    case WAIT_BEFORE_TURNING: {
        WAIT_NONBLOCKING_SAME_MS(250, WAIT_BEFORE_TURNING);
        GO_HANDLE(TURN_ROMI_STATE);
        break;
      }

    case TURN_ROMI_STATE: {
        static float last_degree {0};
        if (((imclc.turnRotation - last_degree) > -90)) {
          //turn motor right
          turnLeft = 1;
          turnRight = -1;
          imuTaskIns.callMyTask();
          GO_HANDLE(TURN_ROMI_STATE);
        } else {
          loc_speed = 0;
          last_degree = imclc.turnRotation;
          leftMotor.motorControl(SMOOTH_STOP);
          rightMotor.motorControl(SMOOTH_STOP);
          GO_HANDLE(MOTOR_STOP);
        }
        break;
      }

    case MOTOR_STOP: {
        //cmp.updateFilter();//stabilize its z axis value
        WAIT_NONBLOCKING_SAME_MS(250, MOTOR_STOP);
        GO_HANDLE(PATH_TRACING);
        break;
      }

    case STOP_SYSTEM: {
        leftMotor.motorControl(0);
        rightMotor.motorControl(0);
        break;
      }
    default : {
        GO_HANDLE(IDLE_STATE);
        break;
      }
  }
}





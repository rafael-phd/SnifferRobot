#include <SnifferDurationSensor.h>
#include <SnifferSpeedCtrl.h>
#include <SnifferMotorDrive.h>
#include <SnifferSamplingCtrl.h>
#include <SnifferSpeedSensor.h>
#include <SnifferSocketSrv.h>
#include <SnifferIRSensor.h>

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  
  SnifferIRSensor::begin(&Serial);
  SnifferSpeedSensor::begin(&Serial);
  SnifferMotorDrive::begin(&Serial);
  SnifferSpeedCtrl::begin(&Serial);
  SnifferSocketSrv::begin(&Serial);  
  SnifferDurationSensor::begin(&Serial);  
}

void loop() {
  // put your main code here, to run repeatedly:
  if (SnifferSocketSrv::socket.Listen()) {
    SnifferSocketSrv::socket.setSrvData(SNIFFER_SRVDATA_DURATION_IDX, SnifferDurationSensor::duration.read());
    SnifferSocketSrv::socket.setSrvData(SNIFFER_SRVDATA_SPEED_LEFT_IDX, SnifferSpeedSensor::left_wheel.read(SnifferMotorDrive::motor_drive.getIsForward_B()));
    SnifferSocketSrv::socket.setSrvData(SNIFFER_SRVDATA_SPEED_RIGHT_IDX, SnifferSpeedSensor::right_wheel.read(SnifferMotorDrive::motor_drive.getIsForward_A()));
    SnifferSocketSrv::socket.setSrvData(SNIFFER_SRVDATA_IR_LEFT_IDX, SnifferIRSensor::front_left.read());
    SnifferSocketSrv::socket.setSrvData(SNIFFER_SRVDATA_IR_RIGHT_IDX, SnifferIRSensor::front_right.read());
    SnifferSocketSrv::socket.setSrvData(SNIFFER_SRVDATA_CONTROL_LEFT_IDX, SnifferMotorDrive::motor_drive.getControl_B());
    SnifferSocketSrv::socket.setSrvData(SNIFFER_SRVDATA_CONTROL_RIGHT_IDX, SnifferMotorDrive::motor_drive.getControl_A());
    if (SnifferSocketSrv::socket.Talk(&Serial)) {
      switch(SnifferSocketSrv::socket.getCliCmd()) {
        case SNIFFER_CLICMD_MOTOR_CTRL:
          SnifferMotorDrive::motor_drive.setControl_A(SnifferSocketSrv::socket.getCliData(SNIFFER_CLIDATA_MOTOR_CTRL_RIGHT_IDX));
          SnifferMotorDrive::motor_drive.setControl_B(SnifferSocketSrv::socket.getCliData(SNIFFER_CLIDATA_MOTOR_CTRL_LEFT_IDX));
          break;
        case SNIFFER_CLICMD_PID_LEFT:
          Serial.print("SnifferSpeedCtrl::left_wheel: ");
          Serial.print("kp:");
          Serial.print(SnifferSocketSrv::socket.getCliData(SNIFFER_CLIDATA_PID_KP_IDX));
          Serial.print(",ki:");
          Serial.print(SnifferSocketSrv::socket.getCliData(SNIFFER_CLIDATA_PID_KI_IDX));
          Serial.print(",kd:");
          Serial.print(SnifferSocketSrv::socket.getCliData(SNIFFER_CLIDATA_PID_KD_IDX));
          Serial.print(",tau:");
          Serial.println(SnifferSocketSrv::socket.getCliData(SNIFFER_CLIDATA_PID_TAU_IDX));
          SnifferSpeedCtrl::left_wheel.setProportionalGain(SnifferSocketSrv::socket.getCliData(SNIFFER_CLIDATA_PID_KP_IDX));
          SnifferSpeedCtrl::left_wheel.setIntegralGain(SnifferSocketSrv::socket.getCliData(SNIFFER_CLIDATA_PID_KI_IDX));
          SnifferSpeedCtrl::left_wheel.setDerivativeGain(SnifferSocketSrv::socket.getCliData(SNIFFER_CLIDATA_PID_KD_IDX));
          SnifferSpeedCtrl::left_wheel.setDerivativeFilterGain(SnifferSocketSrv::socket.getCliData(SNIFFER_CLIDATA_PID_TAU_IDX));
          break;
        case SNIFFER_CLICMD_PID_RIGHT:
          Serial.print("SnifferSpeedCtrl::right_wheel: ");
          Serial.print("kp:");
          Serial.print(SnifferSocketSrv::socket.getCliData(SNIFFER_CLIDATA_PID_KP_IDX));
          Serial.print(",ki:");
          Serial.print(SnifferSocketSrv::socket.getCliData(SNIFFER_CLIDATA_PID_KI_IDX));
          Serial.print(",kd:");
          Serial.print(SnifferSocketSrv::socket.getCliData(SNIFFER_CLIDATA_PID_KD_IDX));
          Serial.print(",tau:");
          Serial.println(SnifferSocketSrv::socket.getCliData(SNIFFER_CLIDATA_PID_TAU_IDX));
          SnifferSpeedCtrl::right_wheel.setProportionalGain(SnifferSocketSrv::socket.getCliData(SNIFFER_CLIDATA_PID_KP_IDX));
          SnifferSpeedCtrl::right_wheel.setIntegralGain(SnifferSocketSrv::socket.getCliData(SNIFFER_CLIDATA_PID_KI_IDX));
          SnifferSpeedCtrl::right_wheel.setDerivativeGain(SnifferSocketSrv::socket.getCliData(SNIFFER_CLIDATA_PID_KD_IDX));
          SnifferSpeedCtrl::right_wheel.setDerivativeFilterGain(SnifferSocketSrv::socket.getCliData(SNIFFER_CLIDATA_PID_TAU_IDX));
          break;
        case SNIFFER_CLICMD_SPEED_CTRL:
          SnifferSpeedCtrl::left_wheel.setTargetValue(SnifferSocketSrv::socket.getCliData(SNIFFER_CLIDATA_SPEED_CTRL_TGT_LEFT_IDX));
          SnifferSpeedCtrl::right_wheel.setTargetValue(SnifferSocketSrv::socket.getCliData(SNIFFER_CLIDATA_SPEED_CTRL_TGT_RIGHT_IDX));
        default:
          break;
      }
    }
  } 
  switch(SnifferSocketSrv::socket.getCliCmd()) {
    case SNIFFER_CLICMD_NONE:
      SnifferMotorDrive::motor_drive.setControl_A(0.0f);
      SnifferMotorDrive::motor_drive.setControl_B(0.0f);
      break;
    case SNIFFER_CLICMD_SPEED_CTRL:
      SnifferMotorDrive::motor_drive.setControl_B(
        SnifferSpeedCtrl::left_wheel.calcControl(
          SnifferSpeedSensor::left_wheel.read(SnifferMotorDrive::motor_drive.getIsForward_B())
        )
      );
      SnifferMotorDrive::motor_drive.setControl_A(
        SnifferSpeedCtrl::right_wheel.calcControl(
          SnifferSpeedSensor::right_wheel.read(SnifferMotorDrive::motor_drive.getIsForward_A())
        )
      );
      break;
  }
}

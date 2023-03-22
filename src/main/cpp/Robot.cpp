// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/XboxController.h>
#include <frc/TimedRobot.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/CANSparkMax.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <ctre/phoenix/motorcontrol/can/WPI_VictorSPX.h>
#include <rev/AbsoluteEncoder.h>
#include <frc/DigitalInput.h>
#include <frc/AddressableLED.h>
#include "AHRS.h"

/**
 * This is a demo program showing the use of the DifferentialDrive class.
 * Runs the motors with arcade steering.
 */
class Robot : public frc::TimedRobot {
  ctre::phoenix::motorcontrol::can::WPI_VictorSPX m_rightMotor{1};
  ctre::phoenix::motorcontrol::can::WPI_VictorSPX m_backrightMotor{2};
  ctre::phoenix::motorcontrol::can::WPI_VictorSPX m_backleftMotor{3};
  ctre::phoenix::motorcontrol::can::WPI_VictorSPX m_leftMotor{4};
  rev::CANSparkMax m_armMotor{5 , rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_intakeMotor{6, rev::CANSparkMax::MotorType::kBrushless};
  
  frc::MotorControllerGroup m_left{m_leftMotor, m_backleftMotor};
  frc::MotorControllerGroup m_right{m_rightMotor, m_backrightMotor};
  frc::DifferentialDrive m_robotDrive{m_left, m_right};
  
  frc::DigitalInput ArmLimit{9};

  frc::XboxController m_stick{0};
  frc::XboxController m_stick1{1};

  AHRS *gyro;
  
  rev::SparkMaxRelativeEncoder m_armEncoder = m_armMotor.GetEncoder();
  //rev::SparkMaxPIDController m_pidController = m_armMotor.GetPIDController();

  frc::AddressableLED Led0{4};
  std::array<frc::AddressableLED::LEDData, 6> LedBuff;

  double armPos = -10;

  const double INTAKE_OUT = 1.0;
  const double INTAKE_HOLD = 0.07;
  const int ARM_CURRENT = 20;
  const int INTAKE_CURRENT = 30;
  const int INTAKE_CURRENT_HOLD = 5;

  int lastGamePiece;
  enum GamePiece{ CONE, CUBE, NONE};
  //double kP = 0.1, kI = 0, kD = 0, kIz = 0, kFF = 10, kMaxOutput = 1, kMinOuput = -1;
  //frc::SmartDashboard SmartDashboard;

public:
  void RobotInit() override {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotor.SetInverted(true);
    m_backrightMotor.SetInverted(true);
    m_leftMotor.SetInverted(true);
    m_backleftMotor.SetInverted(true);
    m_rightMotor.SetNeutralMode(ctre::phoenix::motorcontrol::Brake);
    m_backrightMotor.SetNeutralMode(ctre::phoenix::motorcontrol::Brake);
    m_leftMotor.SetNeutralMode(ctre::phoenix::motorcontrol::Brake);
    m_backleftMotor.SetNeutralMode(ctre::phoenix::motorcontrol::Brake);
    m_armMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_intakeMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    
    //m_pidController.SetP(kP);
    //m_pidController.SetI(kI);
    //m_pidController.SetD(kD);
    //m_pidController.SetIZone(kIz);
    //m_pidController.SetFF(kFF);
    //m_pidController.SetOutputRange(kMinOuput, kMaxOutput);
 
    Led0.SetLength(6);
    Led0.SetData(LedBuff);
    Led0.Start();

    //gyro init
    gyro = new AHRS(frc::SPI::Port::kMXP,30);
    gyro->Calibrate();
    while (gyro->IsCalibrating());
  }

  void TeleopInit() override {
    m_armMotor.SetSmartCurrentLimit(2);
    m_armMotor.Set(-0.2);
    while(ArmLimit.Get() == 0){
      //limit switch
      frc::SmartDashboard::PutNumber(" Limit ", ArmLimit.Get());
    }
    m_armEncoder.SetPosition(0);
    m_armMotor.Set(0.0);

    for(int i = 0; i < 6; i++) {
      LedBuff[i].SetRGB(255, 0, 0);
    }
    Led0.SetData(LedBuff);
    m_armMotor.SetSmartCurrentLimit(20);
    //m_pidController.SetReference(1, rev::CANSparkMax::ControlType::kPosition);
    armPos = 1;
  }
  void TeleopPeriodic() override {
    double armSpeed;
    double intakeSpeed;
    //int armAmps;
    int intakeAmps;

    if (m_stick1.GetAButton())
    {
      armPos = 0;
    }
    if (m_stick1.GetBButton())
    {
      armPos = 20;
    }
    if (m_stick1.GetXButton())
    {
      armPos = 30;
    }
    if (m_stick1.GetYButton())
    {
      armPos = 40;
    }

    if (m_stick1.GetLeftBumper())
    {
      intakeSpeed = INTAKE_OUT;
      intakeAmps = INTAKE_CURRENT;
      lastGamePiece = CUBE;      
    } else if (m_stick1.GetRightBumper())
    {
      intakeSpeed = -INTAKE_OUT;
      intakeAmps = INTAKE_CURRENT;
      lastGamePiece = CONE;      
    } else if (lastGamePiece == CUBE)
    {
      intakeSpeed = INTAKE_HOLD;
      intakeAmps = INTAKE_CURRENT_HOLD;
    } else if (lastGamePiece == CONE)
    {
      intakeSpeed = -INTAKE_HOLD;
      intakeAmps = INTAKE_CURRENT_HOLD;
    } else{
      intakeSpeed = 0;
      intakeAmps = 0;
    }


    // Autonomous Balancing Program: 
    if (m_stick.GetStartButton())
    {
      // Pitch Values Between 85 & 95 Is Balanced.
      if (gyro->GetPitch() < 85.0)
        m_robotDrive.ArcadeDrive(0.4, 0);
      else if (gyro->GetPitch() > 95.0)
        m_robotDrive.ArcadeDrive(-0.4, 0);
      else 
        m_robotDrive.ArcadeDrive(0, 0);
    }
    // Drive With Controller:
    else
    {
      m_robotDrive.ArcadeDrive(-m_stick.GetLeftY(), -m_stick.GetLeftX());
    }
    // Courtesy Of Caleb Taylor
    
    // Arm/Intake Controls: ---------------------------------------------------------
    if (m_stick.GetXButton())
    {
      intakeSpeed = 1;
      intakeAmps = INTAKE_CURRENT;
      //m_intakeMotor.Set(1.0);
    }else
    if (m_stick.GetBButton())
    {
      intakeSpeed = -1;
      intakeAmps = INTAKE_CURRENT;
      //m_intakeMotor.Set(-1.0);
    }else
    {
      intakeSpeed = 0;
      intakeAmps = 0;
      //m_intakeMotor.Set(0.0);
    }

    if (m_stick.GetAButton())
    {
      armSpeed = -0.4;
      armPos = -10;
      //m_armMotor.Set(-0.4);
    } else
    if (m_stick.GetYButton())
    {
      armSpeed = 0.4;
      armPos = -10;
      //m_armMotor.Set(0.4);
    } else
    {
      armSpeed = 0.0;
      armPos = -10;
      //m_armMotor.Set(0.0);
    } 

    
    if(armPos < 0){    
      SetArmMotor(armSpeed);
    } else{
      int currArmPos = m_armEncoder.GetPosition();
      if(currArmPos < armPos)
      {
        SetArmMotor((armPos - currArmPos)/30);
      }else{
        SetArmMotor(-(currArmPos - armPos)/30);
      }
    }
    SetIntakeMotor(intakeSpeed, intakeAmps);
    frc::SmartDashboard::PutNumber(" Pitch2 ", gyro->GetPitch());
    //frc::SmartDashboard::PutData(" gyro ", gyro);
    frc::SmartDashboard::PutNumber("arm pos", m_armEncoder.GetPosition());
    frc::SmartDashboard::PutNumber(" roll ", gyro->GetRoll());
    frc::SmartDashboard::PutNumber(" Y ", gyro->GetRawGyroY());
    frc::SmartDashboard::PutNumber(" Limit ", ArmLimit.Get());
  }

  void SetArmMotor(double speed)
  {
    m_armMotor.Set(speed);
    frc::SmartDashboard::PutNumber("arm Speed", speed);  
    frc::SmartDashboard::PutNumber("arm Amps ", m_armMotor.GetOutputCurrent());
    frc::SmartDashboard::PutNumber("arm Temp", m_armMotor.GetMotorTemperature());  
  }

  void SetIntakeMotor(double speed, int amps)
  {
    m_intakeMotor.Set(speed);
    m_intakeMotor.SetSmartCurrentLimit(amps);
    frc::SmartDashboard::PutNumber("intake Speed", speed);  
    frc::SmartDashboard::PutNumber("inake Amps ", m_intakeMotor.GetOutputCurrent());
    frc::SmartDashboard::PutNumber("intake Temp", m_intakeMotor.GetMotorTemperature());  
  
  }

}
#ifndef RUNNING_FRC_TESTS
;
int main() {
  return frc::StartRobot<Robot>();
}
#endif
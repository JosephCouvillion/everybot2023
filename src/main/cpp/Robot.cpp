// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <fmt/core.h>
#include <frc/XboxController.h>
#include <frc/TimedRobot.h>
#include <frc/drive/DifferentialDrive.h>
#include <rev/CANSparkMax.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <ctre/phoenix/motorcontrol/can/WPI_VictorSPX.h>
#include <rev/AbsoluteEncoder.h>
#include <frc/DigitalInput.h>
#include <frc/AddressableLED.h>
#include "AHRS.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <cameraserver/CameraServer.h>
#include <wpi/raw_ostream.h>
#include <frc/Timer.h>

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
  //frc::XboxController m_stick1{1};

  AHRS *gyro;
  
  rev::SparkMaxRelativeEncoder m_armEncoder = m_armMotor.GetEncoder();

  const int numLEDs = 16;
  frc::AddressableLED Led0{9};
  std::array<frc::AddressableLED::LEDData, 16> LedBuff;

  double desiredArmPos = -10;   

  const double INTAKE_OUT = 1.0;   //speed (range -1 to 1)
  const double INTAKE_HOLD = 0.07; //speed (range near 0) --> not currently using
  const int ARM_CURRENT = 40; //amps (range 0-40)
  const int INTAKE_CURRENT = 30;  //amps (range 0-40)
  const int INTAKE_CURRENT_HOLD = 5; //amps (range nearish 5)
  double currArmPos;
  int lastGamePiece;

  enum GamePiece{ CONE, CUBE, NONE};
private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Nothing";
  const std::string kauto1 = "High Cube Balance";
  const std::string kauto2 = "Hign Cube Leave";
  const std::string kauto3 = "High Cone Balance";
  const std::string kauto4 = "Hign Cone Leave";
  std::string m_autoSelected;
  frc::Timer m_autoTimer;
  


public:
  void RobotInit() override {
    frc::SmartDashboard::PutString(" Mode ", "RobotInit");

    // Camera Stream:
    #if defined(__linux__)
      frc::CameraServer::StartAutomaticCapture();
      
      //GetInstance()->StartAutomaticCapture();
    #else
      wpi::errs() << "Vision only available on Linux.\n";
      wpi::errs().flush();
    #endif

    // Initialize Autonomous Mode Options:
    m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
    m_chooser.AddOption(kauto1, kauto1);
    m_chooser.AddOption(kauto2, kauto2);
    m_chooser.AddOption(kauto3, kauto3);
    m_chooser.AddOption(kauto4, kauto4);
    frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

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
    
 
    Led0.SetLength(numLEDs);
    Led0.SetData(LedBuff);
    Led0.Start();

    //gyro init
    gyro = new AHRS(frc::SPI::Port::kMXP,30);
    gyro->Calibrate();
    while (gyro->IsCalibrating());
  }

  void RobotPeriodic() override{
    frc::SmartDashboard::PutNumber(" Pitch2 ", gyro->GetPitch());
    frc::SmartDashboard::PutNumber("arm pos", m_armEncoder.GetPosition());
    frc::SmartDashboard::PutNumber("currArmPos", currArmPos);
    frc::SmartDashboard::PutNumber("desiredArmPos", desiredArmPos);
    frc::SmartDashboard::PutNumber(" roll ", gyro->GetRoll());
    frc::SmartDashboard::PutNumber(" Y ", gyro->GetRawGyroY());
    frc::SmartDashboard::PutNumber(" Limit ", ArmLimit.Get());
    frc::SmartDashboard::PutString(" Mode ", "TeleopPeriodic");
    //m_robotDrive.Feed();  //Probably not needed
  }
  
  void DisabledPeriodic() override {
    Red();
    frc::SmartDashboard::PutString(" Mode ", "Disabled");
  }

  void AutonomousInit() {
    // Set Auto Mode To Text Selected In Smart Dashboard:
    m_autoSelected = m_chooser.GetSelected();
    fmt::print("Auto selected: {}\n", m_autoSelected);

    Blue(); //if robot stays blue, limit switch triggers
    frc::SmartDashboard::PutString(" Mode ", "AutoInit");

    m_armMotor.SetSmartCurrentLimit(20);
    m_armMotor.Set(-0.6);  //set the motor speed
    while(ArmLimit.Get() == 0){
      //limit switch
      frc::SmartDashboard::PutNumber(" Limit ", ArmLimit.Get());
    }
    m_armEncoder.SetPosition(0);  //when the arm hits the limit switch
    m_armMotor.Set(0.0);

    Red();  // we've inited, but haven't started running autonomous

    m_armMotor.SetSmartCurrentLimit(20);
    desiredArmPos = 1;

    // Start Timer:
    m_autoTimer.Reset();
    m_autoTimer.Start();
    
    Blue();
  }

  void AutonomousPeriodic()
  {
    // High Goal 
    if (m_autoSelected == kauto1)
    {
      AutoHighCubeBalance();
    } 
    if (m_autoSelected == kauto2)
    {
      AutoHighCubeLeave();
    }
    
    if (m_autoSelected == kauto3)
    {
      AutoHighConeBalance();
    }
    if (m_autoSelected == kauto4)
    {
      AutoHighConeLeave();
    }
    
    //checks arm position and holds it in place
    currArmPos = m_armEncoder.GetPosition();
    if(currArmPos > desiredArmPos)
    {
      SetArmMotor(-(currArmPos - desiredArmPos)/20); //Proportional done the old fashion way
      //Yellow();        
    }else{
      SetArmMotor((desiredArmPos - currArmPos)/20);
      //Blue();
    }
  }

  void TeleopInit() override {
    Blue();

    frc::SmartDashboard::PutString(" Mode ", "TeleopInit");

    m_armMotor.SetSmartCurrentLimit(20);
    m_armMotor.Set(-0.6);
    while(ArmLimit.Get() == 0){
      //limit switch
      frc::SmartDashboard::PutNumber(" Limit ", ArmLimit.Get());
    }
    m_armEncoder.SetPosition(0);
    m_armMotor.Set(0.0);

    Red();

    m_armMotor.SetSmartCurrentLimit(20);
    desiredArmPos = 1;
  }

  void TeleopPeriodic() override {
    double armSpeed;
    double intakeSpeed;
    int intakeAmps = 30;
    int ArmPos;  

    ArmPos = -5;  // No control has been actuated

    Green();

    if(ArmLimit.Get() == 1){
      //limit switch
      m_armEncoder.SetPosition(0);
      frc::SmartDashboard::PutNumber(" Limit ", ArmLimit.Get());
    }

    // Autonomous Balancing Program: 
    if (m_stick.GetStartButton())
    {
      AutoBalanceForward();
    }
    // Drive With Controller:
    else
    {
      /*  Arcade drive 
      m_robotDrive.ArcadeDrive(-m_stick.GetLeftY(), -m_stick.GetLeftX());
      */  
      //Grand Theft Auto
      double speed =  m_stick.GetRightTriggerAxis() - m_stick.GetLeftTriggerAxis();
      m_robotDrive.ArcadeDrive(speed, -m_stick.GetLeftX());
      frc::SmartDashboard::PutNumber(" speed ", speed);
    }
    // Courtesy Of Caleb Taylor  

    // Arm/Intake Controls: ---------------------------------------------------------
    if (m_stick.GetXButton())
    {
      intakeSpeed = 1;
      intakeAmps = INTAKE_CURRENT;
    }else
    if (m_stick.GetBButton())
    {
      intakeSpeed = -1;
      intakeAmps = INTAKE_CURRENT;
    }else
    {
      intakeSpeed = 0;
      intakeAmps = 0;
    }

    if (m_stick.GetAButton())
    {
      m_armMotor.SetSmartCurrentLimit(40);
      armSpeed = -0.50;
      ArmPos = -10;
    } else if (m_stick.GetYButton())
    {
      m_armMotor.SetSmartCurrentLimit(40);
      armSpeed = 0.50;
      ArmPos = -10;
    } else
    {
      if(desiredArmPos < 0){
        desiredArmPos = m_armEncoder.GetPosition();
        m_armMotor.SetSmartCurrentLimit(20);
      }
    }

    if (m_stick.GetYButtonReleased() || m_stick.GetAButtonReleased())
    {
      desiredArmPos = m_armEncoder.GetPosition();
      frc::SmartDashboard::PutString(" Button ", "Released");
    }
    
    if(ArmPos == -10){      // neg arm postion means manual control
      SetArmMotor(armSpeed);
    } else
    {        
      currArmPos = m_armEncoder.GetPosition();
      if(currArmPos > desiredArmPos)
      {
        SetArmMotor(-(currArmPos - desiredArmPos)/20); //Proportional done the old fashion way
        //Yellow();
      }else{
        SetArmMotor((desiredArmPos - currArmPos)/20);
        //Blue();
      }
    }
    
    SetIntakeMotor(intakeSpeed, intakeAmps);
    frc::SmartDashboard::PutNumber("ArmPos", ArmPos);
  }

  void SetArmMotor(double speed)
  {
    m_armMotor.Set(speed);
    frc::SmartDashboard::PutNumber("arm Speed", speed);  
    frc::SmartDashboard::PutNumber("arm Amps ", m_armMotor.GetOutputCurrent());
    frc::SmartDashboard::PutNumber("arm Temp", m_armMotor.GetMotorTemperature());  
  }

// spin intake motor between (-1 and 1) and current between 0 and 40
  void SetIntakeMotor(double speed, int amps)
  {
    m_intakeMotor.Set(speed);
    m_intakeMotor.SetSmartCurrentLimit(amps);
    frc::SmartDashboard::PutNumber("intake Speed", speed);  
    frc::SmartDashboard::PutNumber("inake Amps ", m_intakeMotor.GetOutputCurrent());
    frc::SmartDashboard::PutNumber("intake Temp", m_intakeMotor.GetMotorTemperature());    
  }

  void AutoHighCubeBalance()
  {
    if (m_autoTimer.Get() > 0_s && m_autoTimer.Get() < 1_s)
    {
      desiredArmPos = 30;  //Extend arm high goal
    } // bang less hard
    else if (m_autoTimer.Get() > 1_s && m_autoTimer.Get() < 2_s)
    {
      desiredArmPos = 40;  //Extend arm high goal
    }
    /*else if (m_autoTimer.Get() > 2_s && m_autoTimer.Get() < 4_s)
    {
      m_robotDrive.ArcadeDrive(0.5, 0);  //drive forward
    }*/
    else if (m_autoTimer.Get() > 2_s && m_autoTimer.Get() < 4_s)
    {
      m_robotDrive.ArcadeDrive(0.0, 0);  //drive stop
      SetIntakeMotor(-1, 30);  //output cube
    }
    else if (m_autoTimer.Get() > 4_s && m_autoTimer.Get() < 5_s)
    {
      desiredArmPos = 0; // lower arm 
      //m_robotDrive.ArcadeDrive(-0.50, 0);  //drive backwards
      SetIntakeMotor(0, 30);  //Stop intake
    }
    else if (m_autoTimer.Get() > 5_s && m_autoTimer.Get() < 15_s)
    //8 is balance, 10 is not balance
    {
      AutoBalanceBackward();  //charging station
    }
  }

void AutoHighCubeLeave()
  {
    if (m_autoTimer.Get() > 0_s && m_autoTimer.Get() < 1_s)
    {
      desiredArmPos = 30;  //Extend arm high goal
    } // bang less hard
    else if (m_autoTimer.Get() > 1_s && m_autoTimer.Get() < 2_s)
    {
      desiredArmPos = 40;  //Extend arm high goal
    }
    /*else if (m_autoTimer.Get() > 2_s && m_autoTimer.Get() < 4_s)
    {
      m_robotDrive.ArcadeDrive(0.5, 0);  //drive forward
    }*/
    else if (m_autoTimer.Get() > 2_s && m_autoTimer.Get() < 4_s)
    {
      m_robotDrive.ArcadeDrive(0.0, 0);  //drive stop
      SetIntakeMotor(-1, 30);  //output cube
    }
    else if (m_autoTimer.Get() > 4_s && m_autoTimer.Get() < 8_s)
    {
      desiredArmPos = 0; // lower arm 
      m_robotDrive.ArcadeDrive(-0.55, 0);  //drive backwards
      SetIntakeMotor(0, 30);  //Stop intake
    }
    else if (m_autoTimer.Get() > 8_s && m_autoTimer.Get() < 15_s)
    {
      m_robotDrive.ArcadeDrive(0.0, 0);  //stop
    }
  }

void AutoHighConeBalance()
  {
    if (m_autoTimer.Get() > 0_s && m_autoTimer.Get() < 1_s)
    {
      desiredArmPos = 30;  //Extend arm high goal
    } // bang less hard
    else if (m_autoTimer.Get() > 1_s && m_autoTimer.Get() < 2_s)
    {
      desiredArmPos = 40;  //Extend arm high goal
    }
    /*else if (m_autoTimer.Get() > 2_s && m_autoTimer.Get() < 3_s)
    {
      m_robotDrive.ArcadeDrive(0.5, 0);  //drive forward 
    } */
    else if (m_autoTimer.Get() > 2_s && m_autoTimer.Get() < 4_s)
    {
      m_robotDrive.ArcadeDrive(0.0, 0);  //drive stop
      SetIntakeMotor(1, 30);  //output cone
    }
    else if (m_autoTimer.Get() > 4_s && m_autoTimer.Get() < 5_s)
    {
      desiredArmPos = 0; // lower arm 
      m_robotDrive.ArcadeDrive(-0.50, 0);  //drive backwards
      SetIntakeMotor(0, 30);  //Stop intake
    }
    else if (m_autoTimer.Get() > 5_s && m_autoTimer.Get() < 15_s)
    {
      AutoBalanceBackward();  //charging station
    }
  }

/// @brief 
void AutoHighConeLeave()
  {
    if (m_autoTimer.Get() > 0_s && m_autoTimer.Get() < 1_s)
    {
      desiredArmPos = 30;  //Extend arm high goal
    } // bang less hard
    else if (m_autoTimer.Get() > 1_s && m_autoTimer.Get() < 2_s)
    {
      desiredArmPos = 40;  //Extend arm high goal
    }
    /*else if (m_autoTimer.Get() > 2_s && m_autoTimer.Get() < 4_s)
    {
      m_robotDrive.ArcadeDrive(0.5, 0);  //drive forward
    }*/
    else if (m_autoTimer.Get() > 2_s && m_autoTimer.Get() < 4_s)
    {
      m_robotDrive.ArcadeDrive(0.0, 0);  //drive stop
      SetIntakeMotor(1, 30);  //output cone
    }
    else if (m_autoTimer.Get() > 4_s && m_autoTimer.Get() < 7_s)
    {
      desiredArmPos = 0; // lower arm
      m_robotDrive.ArcadeDrive(-0.60, 0);  //drive backwards 
      SetIntakeMotor(0, 30);  //Stop intake
    }
    else if (m_autoTimer.Get() > 7_s && m_autoTimer.Get() < 15_s)
    {
       m_robotDrive.ArcadeDrive(0.0, 0);  //stop
    }
  }

  void AutoBalanceForward()
  {
    // Pitch Values Between 85 & 95 Is Balanced. 
    if (gyro->GetPitch() < -5.0) 
      m_robotDrive.ArcadeDrive(0.55, 0);
    else if (gyro->GetPitch() > 5.0)
      m_robotDrive.ArcadeDrive(-0.55, 0);
    else 
      m_robotDrive.ArcadeDrive(0, 0);
    // Courtesy Of Caleb Taylor  
  }

  void AutoBalanceBackward()
  {
    // Pitch Values Between 85 & 95 Is Balanced. 
    if (gyro->GetPitch() < -5.0) 
      m_robotDrive.ArcadeDrive(-0.55, 0);
    else if (gyro->GetPitch() > 5.0)
      m_robotDrive.ArcadeDrive(0.55, 0);
    else 
      m_robotDrive.ArcadeDrive(0, 0);
    // Courtesy Of Caleb Taylor  
  }

  void Red(){
    for(int i = 0; i < numLEDs; i++) {
      LedBuff[i].SetRGB(255, 0, 0);
    }
    Led0.SetData(LedBuff);
  }
  void Green(){
    for(int i = 0; i < numLEDs; i++) {
      LedBuff[i].SetRGB(0,255, 0);
    }
    Led0.SetData(LedBuff);
  }
  void Blue(){
    for(int i = 0; i < numLEDs; i++) {
      LedBuff[i].SetRGB(0, 0, 255);
    }
    Led0.SetData(LedBuff);
  }
  /*void Yellow(){
    for(int i = 0; i < numLEDs; i++) {
      LedBuff[i].SetRGB(255, 255, 0);
    }
    Led0.SetData(LedBuff);
  }
  void Magenta(){
    for(int i = 0; i < numLEDs; i++) {
      LedBuff[i].SetRGB(255, 0, 255);
    }
    Led0.SetData(LedBuff);
  }
  void Cyan(){
    for(int i = 0; i < numLEDs; i++) {
      LedBuff[i].SetRGB(0, 255, 255);
    }
    Led0.SetData(LedBuff);
  }
  void White(){
    for(int i = 0; i < numLEDs; i++) {
      LedBuff[i].SetRGB(255, 255, 255);
    }
    Led0.SetData(LedBuff);
  }*/

}

#ifndef RUNNING_FRC_TESTS
;
int main() {
  return frc::StartRobot<Robot>();
}
#endif
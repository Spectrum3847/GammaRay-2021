/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.Debugger;
import frc.lib.util.SpectrumPreferences;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class Launcher extends SubsystemBase {

  public final TalonFX motorLeft;
  public final TalonFX motorRight;
  public final Servo leftHood;
  public final Servo rightHood;

  private double kP, kI, kD, kF;
  private int iZone;

  /**
   * Creates a new Intake.
   */
  public Launcher() {
    //Pid

    kP = SpectrumPreferences.getInstance().getNumber("Launcher kP",0.05);
    kI = SpectrumPreferences.getInstance().getNumber("Launcher kI",0.001);
    kD = SpectrumPreferences.getInstance().getNumber("Launcher kD",0.07);
    kF = SpectrumPreferences.getInstance().getNumber("Launcher kF",0.0472);
    iZone = (int) SpectrumPreferences.getInstance().getNumber("Launcher I-Zone", 150);

    
    motorLeft = new TalonFX(Constants.LauncherConstants.kLauncherMotorLeft);
    motorLeft.setInverted(false);
    SupplyCurrentLimitConfiguration supplyCurrentLimit = new SupplyCurrentLimitConfiguration(true, 40, 45, 0.5);
    motorLeft.configSupplyCurrentLimit(supplyCurrentLimit);

    motorLeft.config_kP(0, kP);
    motorLeft.config_kI(0, kI);   
    motorLeft.config_kD(0, kD);  
    motorLeft.config_kF(0, kF);  
    motorLeft.config_IntegralZone(0, iZone);

    motorLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    motorRight = new TalonFX(Constants.LauncherConstants.kFollowerMotorRight);
    motorRight.setInverted(true);   //should be inverse of motorLeft
    motorRight.configSupplyCurrentLimit(supplyCurrentLimit);
    motorRight.follow(motorLeft);

    SpectrumPreferences.getInstance().getNumber("Launcher Setpoint", 1000);


    leftHood = new Servo(Constants.LauncherConstants.kHoodServoLeft);
    rightHood = new Servo(Constants.LauncherConstants.kHoodServoRight);
    leftHood.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);
    rightHood.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);

    this.setDefaultCommand(new RunCommand(() -> stop() , this));
  }

  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setPercentModeOutput(double speed){
    motorLeft.set(ControlMode.PercentOutput, speed);
  }

  public void setVelocity( double velocity){
    motorLeft.set(ControlMode.Velocity, velocity);
  }

  public void DashboardVelocity(){
    double wheelRpm = SpectrumPreferences.getInstance().getNumber("Launcher Setpoint", 1000);
    double motorVelocity = (wheelRpm * 30 / 8);
    motorLeft.set(ControlMode.Velocity, motorVelocity);
  }

  public double getWheelRPM() {
    return motorLeft.getSelectedSensorVelocity() * 8 / 30;
  }
  public void full(){
    setPercentModeOutput(1.0);
  }

  public void setHood(double value){
    leftHood.set(value);
    rightHood.set(value);
  }
  public void hoodFullFwd(){
    setHood(1.0);
  }

  public void hoodFullRear(){
    setHood(-1.0);
  }

  public void stop(){
    motorLeft.set(ControlMode.PercentOutput,0);
  }

  public void Dashboard() {
  SmartDashboard.putNumber("Launcher/Velocity", motorLeft.getSelectedSensorVelocity());
  SmartDashboard.putNumber("Launcher/WheelRPM", getWheelRPM());
  SmartDashboard.putNumber("Launcher/OutputPercentage", motorLeft.getMotorOutputPercent());
  SmartDashboard.putNumber("Launcher/LeftCurrent", motorLeft.getSupplyCurrent());
  }

  public static void printDebug(String msg){
    Debugger.println(msg, Robot._launcher, Debugger.debug2);
  }
  
  public static void printInfo(String msg){
    Debugger.println(msg, Robot._launcher, Debugger.info3);
  }
  
  public static void printWarning(String msg) {
    Debugger.println(msg, Robot._launcher, Debugger.warning4);
  }
}

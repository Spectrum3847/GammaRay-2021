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
    motorLeft.setInverted(true);
    SupplyCurrentLimitConfiguration supplyCurrentLimit = new SupplyCurrentLimitConfiguration(true, 40, 45, 0.5);
    motorLeft.configSupplyCurrentLimit(supplyCurrentLimit);

    motorLeft.config_kP(0, kP);
    motorLeft.config_kI(0, kI);   
    motorLeft.config_kD(0, kD);  
    motorLeft.config_kF(0, kF);  
    motorLeft.config_IntegralZone(0, iZone);

    motorLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    motorRight = new TalonFX(Constants.LauncherConstants.kFollowerMotorRight);
    motorRight.setInverted(false);   //should be inverse of motorLeft
    motorRight.configSupplyCurrentLimit(supplyCurrentLimit);
    motorRight.follow(motorLeft);

    SpectrumPreferences.getInstance().getNumber("Launcher Setpoint", 1000);

    //Set Dafault Command to be driven by the operator left stick and divide by 1.75 to reduce speed
    this.setDefaultCommand(new RunCommand(() -> setPercentModeOutput(RobotContainer.operatorController.leftStick.getY() /1.75) , this));
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
  public void feed(){
    setPercentModeOutput(1.0);
  }

  public void fullDown(){
    setPercentModeOutput(-1.0);
  }

  public void indexUp(){
    //setVelocity(500);
    setPercentModeOutput(0.4);
  }

  public void indexDown(){
    //setVelocity(500);
    setPercentModeOutput(-0.4);
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

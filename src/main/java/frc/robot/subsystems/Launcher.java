/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
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
  public final double closeShoot = 0;
  public final double intitationLineShot = 0.6;
  public final double farShot = 1.0;

  private double kP, kI, kD, kF;
  private int iZone;

  /**
   * Creates a new Intake.
   */
  public Launcher() {
    //Pid

    kP = SpectrumPreferences.getInstance().getNumber("Launcher kP", 0.0465);
    kI = SpectrumPreferences.getInstance().getNumber("Launcher kI", 0.0005);
    kD = SpectrumPreferences.getInstance().getNumber("Launcher kD", 0.0);
    kF = SpectrumPreferences.getInstance().getNumber("Launcher kF", 0.048);
    iZone = (int) SpectrumPreferences.getInstance().getNumber("Launcher I-Zone", 150);

    
    motorLeft = new TalonFX(Constants.LauncherConstants.kLauncherMotorLeft);
    motorLeft.setInverted(true);
    SupplyCurrentLimitConfiguration supplyCurrentLimit = new SupplyCurrentLimitConfiguration(true, 40, 45, 0.5);
    motorLeft.configSupplyCurrentLimit(supplyCurrentLimit);
    motorLeft.setNeutralMode(NeutralMode.Coast);

    motorLeft.config_kP(0, kP);
    motorLeft.config_kI(0, kI);   
    motorLeft.config_kD(0, kD);  
    motorLeft.config_kF(0, kF);  
    motorLeft.config_IntegralZone(0, iZone);
    //motorLeft.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 10);

    motorLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    motorRight = new TalonFX(Constants.LauncherConstants.kFollowerMotorRight);
    motorRight.setInverted(false);   //should be inverse of motorLeft
    motorRight.configSupplyCurrentLimit(supplyCurrentLimit);
    motorRight.follow(motorLeft);
    motorRight.setNeutralMode(NeutralMode.Coast);

    int time = 255;
    motorRight.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, time);
    motorRight.setStatusFramePeriod(StatusFrame.Status_6_Misc, time);
    motorRight.setStatusFramePeriod(StatusFrame.Status_7_CommStatus, time);
    motorRight.setStatusFramePeriod(StatusFrame.Status_9_MotProfBuffer, time);
    motorRight.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, time);
    motorRight.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, time);
    motorRight.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, time);

    SpectrumPreferences.getInstance().getNumber("Launcher Setpoint", 1000);


    leftHood = new Servo(Constants.LauncherConstants.kHoodServoLeft);
    rightHood = new Servo(Constants.LauncherConstants.kHoodServoRight);
    leftHood.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);
    rightHood.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);
    this.setHood(intitationLineShot);

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
    //4096 sensor units per rev
    //velocity is in sensor units per 100ms (0.1 secs)
    //launcher belt is 42 to 24
    //60000 milisecs in 1 min
    //RPM to U/100ms is rotations*4096 / 60000ms
    double wheelRpm = SpectrumPreferences.getInstance().getNumber("Launcher Setpoint", 1000);
    double motorVelocity = (wheelRpm / 600 * 2048) / 1.75;
    motorLeft.set(ControlMode.Velocity, motorVelocity);
  }

  public void setLauncherVelocity(double wheelRPM){
    //Sensor Velocity in ticks per 100ms / Sensor Ticks per Rev * 600 (ms to min) * 1.5 gear ratio to shooter
    //Motor Velocity in RPM / 600 (ms to min) * Sensor ticks per rev / Gear Ratio 42to24
    double motorVelocity = (wheelRPM / 600 * 2048) / 1.75;
    setVelocity(motorVelocity);
  }

  public double getWheelRPM() {
    return (motorLeft.getSelectedSensorVelocity() * 1.75) / 2048 * 600;
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

  public void dashboard() {
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

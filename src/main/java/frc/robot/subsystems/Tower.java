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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.Debugger;
import frc.lib.util.SpectrumPreferences;
import frc.robot.Constants;
import frc.robot.Robot;

public class Tower extends SubsystemBase {

  public final TalonFX motorFront;
  public final TalonFX motorRear;

  private double kP, kI, kD, kF;
  private int iZone;

  public final double towerRPM = 1700;

  /**
   * Creates a new Intake.
   */
  public Tower() {
    //Pid

    kP = SpectrumPreferences.getInstance().getNumber("Tower kP",0.0465);
    kI = SpectrumPreferences.getInstance().getNumber("Tower kI",0.0005);
    kD = SpectrumPreferences.getInstance().getNumber("Tower kD",0.0);
    kF = SpectrumPreferences.getInstance().getNumber("Tower kF",0.048);
    iZone = (int) SpectrumPreferences.getInstance().getNumber("Tower I-Zone", 150);

    
    motorFront = new TalonFX(Constants.TowerConstants.kTowerMotorFront);
    motorFront.setInverted(true);
    SupplyCurrentLimitConfiguration supplyCurrentLimit = new SupplyCurrentLimitConfiguration(true, 60, 65, 0.5);
    motorFront.configSupplyCurrentLimit(supplyCurrentLimit);
    motorFront.setNeutralMode(NeutralMode.Coast);

    motorFront.config_kP(0, kP);
    motorFront.config_kI(0, kI);   
    motorFront.config_kD(0, kD);  
    motorFront.config_kF(0, kF);  
    motorFront.config_IntegralZone(0, iZone);
   // motorFront.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 10);

    motorFront.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    motorRear = new TalonFX(Constants.TowerConstants.kTowerMotorRear);
    motorRear.setInverted(false);   //should be inverse of motorFront
    motorRear.configSupplyCurrentLimit(supplyCurrentLimit);
    motorRear.setNeutralMode(NeutralMode.Coast);
    motorRear.follow(motorFront);

    int time = 255;
    //motorRear.setStatusFramePeriod(StatusFrame.Status_1_General, 10);
    motorRear.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, time);
    motorRear.setStatusFramePeriod(StatusFrame.Status_6_Misc, time);
    motorRear.setStatusFramePeriod(StatusFrame.Status_7_CommStatus, time);
    motorRear.setStatusFramePeriod(StatusFrame.Status_9_MotProfBuffer, time);
    motorRear.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, time);
    motorRear.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, time);
    motorRear.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, time);

    SpectrumPreferences.getInstance().getNumber("Tower Setpoint", 1000);

    //Set Dafault Command to be driven by the operator left stick and divide by 1.75 to reduce speed
    this.setDefaultCommand(new RunCommand(() -> stop() , this));
  }

  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setPercentModeOutput(double speed){
    motorFront.set(ControlMode.PercentOutput, speed);
  }

  public void setVelocity( double velocity){
    motorFront.set(ControlMode.Velocity, velocity);
  }

  public void DashboardVelocity(){
    double wheelRpm = SpectrumPreferences.getInstance().getNumber("Tower Setpoint", 1000);
    double motorVelocity = (wheelRpm * 30 / 8);
    motorFront.set(ControlMode.Velocity, motorVelocity);
  }

  public void setTowerVelocity(double wheelRPM){
    //Sensor Velocity in ticks per 100ms / Sensor Ticks per Rev * 600 (ms to min) * 1.5 gear ratio to shooter
    //Motor Velocity in RPM / 600 (ms to min) * Sensor ticks per rev / Gear Ratio 16 to 40
    double motorVelocity = (wheelRPM / 600 * 2048) / 0.4;
    setVelocity(motorVelocity);
  }

  public double getWheelRPM() {
    return (motorFront.getSelectedSensorVelocity() * 0.4) / 2048 * 600;
  }

  public void stop(){
    motorFront.set(ControlMode.PercentOutput,0);
  }

  public void dashboard() {
  SmartDashboard.putNumber("Tower/Velocity", motorFront.getSelectedSensorVelocity());
  SmartDashboard.putNumber("Tower/WheelRPM", getWheelRPM());
  SmartDashboard.putNumber("Tower/OutputPercentage", motorFront.getMotorOutputPercent());
  SmartDashboard.putNumber("Tower/FrontCurrent", motorFront.getSupplyCurrent());
  SmartDashboard.putNumber("Tower/RearCurrent", motorRear.getSupplyCurrent());
  }

  public static void printDebug(String msg){
    Debugger.println(msg, Robot._tower, Debugger.debug2);
  }
  
  public static void printInfo(String msg){
    Debugger.println(msg, Robot._tower, Debugger.info3);
  }
  
  public static void printWarning(String msg) {
    Debugger.println(msg, Robot._tower, Debugger.warning4);
  }
}

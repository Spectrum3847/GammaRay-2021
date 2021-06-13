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

public class Tower extends SubsystemBase {

  public final TalonFX motorFront;
  public final TalonFX motorRear;

  private double kP, kI, kD, kF;
  private int iZone;

  /**
   * Creates a new Intake.
   */
  public Tower() {
    //Pid

    kP = SpectrumPreferences.getInstance().getNumber("Tower kP",0.05);
    kI = SpectrumPreferences.getInstance().getNumber("Tower kI",0.001);
    kD = SpectrumPreferences.getInstance().getNumber("Tower kD",0.07);
    kF = SpectrumPreferences.getInstance().getNumber("Tower kF",0.0472);
    iZone = (int) SpectrumPreferences.getInstance().getNumber("Tower I-Zone", 150);

    
    motorFront = new TalonFX(Constants.TowerConstants.kTowerMotorFront);
    motorFront.setInverted(true);
    SupplyCurrentLimitConfiguration supplyCurrentLimit = new SupplyCurrentLimitConfiguration(true, 40, 45, 0.5);
    motorFront.configSupplyCurrentLimit(supplyCurrentLimit);

    motorFront.config_kP(0, kP);
    motorFront.config_kI(0, kI);   
    motorFront.config_kD(0, kD);  
    motorFront.config_kF(0, kF);  
    motorFront.config_IntegralZone(0, iZone);

    motorFront.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    motorRear = new TalonFX(Constants.TowerConstants.kTowerMotorRear);
    motorRear.setInverted(false);   //should be inverse of motorFront
    motorRear.configSupplyCurrentLimit(supplyCurrentLimit);
    motorRear.follow(motorFront);

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

  public double getWheelRPM() {
    return motorFront.getSelectedSensorVelocity() * 8 / 30;
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
    motorFront.set(ControlMode.PercentOutput,0);
  }

  public void Dashboard() {
  SmartDashboard.putNumber("Tower/Velocity", motorFront.getSelectedSensorVelocity());
  SmartDashboard.putNumber("Tower/WheelRPM", getWheelRPM());
  SmartDashboard.putNumber("Tower/OutputPercentage", motorFront.getMotorOutputPercent());
  SmartDashboard.putNumber("Tower/LeftCurrent", motorFront.getSupplyCurrent());
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

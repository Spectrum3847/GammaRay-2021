// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.Debugger;
import frc.robot.Constants;
import frc.robot.Robot;

public class Indexer extends SubsystemBase {
  public final double feedSpeed = 0.6;
  public final double feedOnPulse = 0.3;
  public final double feedOffPulse = 0.02;
  public final double feedRevPulse = 0.04;
  public final TalonFX motor;

  /** Creates a new Indexer. */
  public Indexer() {  

    motor = new TalonFX(Constants.IndexerConstants.kIndexerMotor);
    motor.setInverted(true);
    SupplyCurrentLimitConfiguration supplyCurrentLimit = new SupplyCurrentLimitConfiguration(true, 40, 45, 0.5);
    motor.configSupplyCurrentLimit(supplyCurrentLimit);
    motor.setNeutralMode(NeutralMode.Coast);

    this.setDefaultCommand(new RunCommand(() -> stop(), this));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setManualOutput(double speed){
    motor.set(ControlMode.PercentOutput,speed);
  }

  public void feed(){
    setManualOutput(feedSpeed);
  }
  public void feedRev(){
    setManualOutput(-1 *feedSpeed);
  }

  public void stop(){
    motor.set(ControlMode.PercentOutput, 0);
  }

  public void dashboard() {
    SmartDashboard.putNumber("Indexer/Output", motor.getMotorOutputPercent());
    SmartDashboard.putNumber("Indexer/Current", motor.getSupplyCurrent());
  }

  public static void printDebug(String msg){
    Debugger.println(msg, Robot._indexer, Debugger.debug2);
  }
  
  public static void printInfo(String msg){
    Debugger.println(msg, Robot._indexer, Debugger.info3);
  }
  
  public static void printWarning(String msg) {
    Debugger.println(msg, Robot._indexer, Debugger.warning4);
  }
}

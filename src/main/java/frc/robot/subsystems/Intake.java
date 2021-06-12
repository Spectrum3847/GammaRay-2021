// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.SpectrumSolenoid;
import frc.lib.util.Debugger;
import frc.robot.Constants;
import frc.robot.Robot;

public class Intake extends SubsystemBase {

  public final double intakeSpeed = 1.0;

  public final TalonFX motor;
  public final SpectrumSolenoid solDown;
  

  /** Creates a new Intake. */
  public Intake() {  

    motor = new TalonFX(Constants.IntakeConstants.kIntakeMotor);
    motor.setInverted(true);
    SupplyCurrentLimitConfiguration supplyCurrentLimit = new SupplyCurrentLimitConfiguration(true, 40, 45, 0.5);
    motor.configSupplyCurrentLimit(supplyCurrentLimit);

    solDown = new SpectrumSolenoid(Constants.IntakeConstants.kIntakeDown);

    this.setDefaultCommand(new RunCommand(() -> stop(), this));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setManualOutput(double speed){
    motor.set(ControlMode.PercentOutput,speed);
  }

  public void intakeOn(){
    setManualOutput(intakeSpeed);
  }

  public void stop(){
    motor.set(ControlMode.PercentOutput, 0);
  }
  
 public void up(){
    solDown.set(false);
  }

  public void down(){
    solDown.set(true);
  }

  public void dashboard() {
    SmartDashboard.putNumber("Intake/Output", motor.getMotorOutputPercent());
    SmartDashboard.putNumber("Intake/Current", motor.getSupplyCurrent());
  }

  public static void printDebug(String msg){
    Debugger.println(msg, Robot._intake, Debugger.debug2);
  }
  
  public static void printInfo(String msg){
    Debugger.println(msg, Robot._intake, Debugger.info3);
  }
  
  public static void printWarning(String msg) {
    Debugger.println(msg, Robot._intake, Debugger.warning4);
  }
}

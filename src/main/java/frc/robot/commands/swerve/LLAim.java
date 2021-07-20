/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.lib.util.Debugger;
import frc.lib.util.SpectrumPreferences;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class LLAim extends PIDCommand {

  public static double kP = 0;
  public static double kI = 0;
  public static double kD = 0;

  boolean hasTarget = false;
  
  public LLAim() {    
    super(
        // The PPIDController used by the command
        new PIDController(
            // The PID gainss
            kP, kI, kD),
        // This should return the measurement
        RobotContainer.visionLL::getLLDegToTarget,
        // This should return the goal (can also be a constant)
        0,
        // This uses the output
        (output) -> RobotContainer.swerve.useOutput(output * -1)); // -1 to turn correct direction
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  @Override
  public void initialize() {
    kP = SpectrumPreferences.getInstance().getNumber("LL-AIM kP", 0.5)/100;
    kI = SpectrumPreferences.getInstance().getNumber("LL-AIM kI", 0.0001)/100;
    kD = SpectrumPreferences.getInstance().getNumber("LL-AIM kD", 0.0001)/100;
    double tolerance = SpectrumPreferences.getInstance().getNumber("LL-AIM Tolerance", 1.5);
    this.getController().setPID(kP, kI, kD);
    getController().setTolerance(tolerance);

    if(RobotContainer.visionLL.getLimelightHasValidTarget()) {
      hasTarget = true;
    } else {
      hasTarget = false;
    }

    /*if(RobotContainer.visionLL.getLLTargetArea() < 0.5) {
      RobotContainer.visionLL.setLimeLightPipeline(1);
    } else {
      RobotContainer.visionLL.setLimeLightPipeline(0);
    } */
    
    super.initialize();
  }

  @Override
  public void execute() {
    super.execute();
    printDebug("Angle: " + this.m_measurement.getAsDouble());
    Debugger.setLevel(Debugger.debug2);
    dashboard();
  }

  public void dashboard() {
    SmartDashboard.putNumber("LL-AIM/ERROR", this.getController().getPositionError());
  }

  
  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    //currently vibrates as long as it has target, doesn't care how far from target we are
    /*if (hasTarget) {
      new ParallelCommandGroup(
        new RumbleController(RobotContainer.operatorController, 0.5),
        new RumbleController(RobotContainer.driverController, 0.5)
      ).schedule();
    }*/
    RobotContainer.swerve.useOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint(); //getController().atGoal() || !hasTarget;
  }

  public static void printDebug(String msg){
    Debugger.println(msg, Robot._visionLL, Debugger.debug2);
  }
  
  public static void printInfo(String msg){
    Debugger.println(msg, Robot._visionLL, Debugger.info3);
  }
  
  public static void printWarning(String msg) {
    Debugger.println(msg, Robot._visionLL, Debugger.warning4);
  }
}

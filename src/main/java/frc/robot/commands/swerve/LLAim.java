/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.RumbleController;

public class LLAim extends ProfiledPIDCommand {

  public static double kP = 0.011;
  public static double kI = 0; // 0.00015
  public static double kD = 0.00025; // 0.0005

  boolean hasTarget = false;
  
  public LLAim() {
    super(
        // The ProfiledPIDController used by the command
        new ProfiledPIDController(
            // The PID gainss
            kP, kI, kD,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(360, 360)),
        // This should return the measurement
        RobotContainer.visionLL::getLLDegToTarget,
        // This should return the goal (can also be a constant)
        0,
        // This uses the output
        (output, setpoint) -> RobotContainer.swerve.useOutput(output));
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(0.2);
  }

  @Override
  public void initialize() {
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
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    if (hasTarget) {
      new ParallelCommandGroup(
        new RumbleController(RobotContainer.operatorController, 0.5),
        new RumbleController(RobotContainer.driverController, 0.5)
      ).schedule();
    }
    RobotContainer.swerve.useOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; //getController().atGoal() || !hasTarget;
  }
}

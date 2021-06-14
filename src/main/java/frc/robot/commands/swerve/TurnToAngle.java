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

public class TurnToAngle extends ProfiledPIDCommand {

  public static double kP = 0.011;
  public static double kI = 0; // 0.00015
  public static double kD = 0.00025; // 0.0005

  boolean hasTarget = false;

  public TurnToAngle(double angle) {
    super(
        // The ProfiledPIDController used by the command
        new ProfiledPIDController(
            // The PID gainss
            kP, kI, kD,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(360, 360)),
        // This should return the measurement
        RobotContainer.swerve::getDegrees,
        // This should return the goal (can also be a constant)
        angle,
        // This uses the output
        (output, setpoint) -> RobotContainer.swerve.useOutput(output));
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(0.2);
  }

  @Override
  public void initialize() {
    super.initialize();
  }

  @Override
  public void execute() {
    super.execute();
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    RobotContainer.swerve.useOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

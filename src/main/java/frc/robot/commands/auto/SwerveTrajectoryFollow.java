// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve;

public class SwerveTrajectoryFollow extends SwerveControllerCommand {
  static double xControllerkP = 1;
  static double yControllerkP = 1;
  static double thetaControllerkP = 1;
  // Constraint for the motion profilied robot angle controller
  public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
    Constants.Swerve.maxSpeed, Constants.Swerve.maxAngularVelocity);
    
  public static ProfiledPIDController thetaController = new ProfiledPIDController(thetaControllerkP, 0, 0,
        kThetaControllerConstraints);

  static Swerve swerve = RobotContainer.swerve;
    

  /** Creates a new SwerveTrajectoryFollow. 
    * @param trajectory The trajectory to follow.
    * @param desiredRotation The angle that the drivetrain should be facing at the end. This is sampled at each
    *     time step.
  */
  public SwerveTrajectoryFollow(Trajectory trajectory, Supplier<Rotation2d> desiredRotation) {
    super(trajectory, swerve::getPose, Constants.Swerve.swerveKinematics, 
        new PIDController(xControllerkP, 0, 0), new PIDController(yControllerkP, 0, 0), thetaController,
        desiredRotation, swerve::setModuleStates, swerve);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    /*edu.wpi.first.wpilibj2.command.SwerveControllerCommand.SwerveControllerCommand(
      Trajectory trajectory, 
      Supplier<Pose2d> pose, 
      SwerveDriveKinematics kinematics, 
      PIDController xController, 
      PIDController yController, 
      ProfiledPIDController thetaController, 
      Supplier<Rotation2d> desiredRotation, 
      Consumer<SwerveModuleState[]> outputModuleStates, 
      Subsystem... requirements)*/
    //Example: https://github.com/timebots5275frc/robot2021/blob/8703c84d0f78f6ba19820d487e48bbe5ccec50d6/src/main/java/frc/robot/RobotContainer.java

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    super.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    super.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return super.isFinished();
  }
}

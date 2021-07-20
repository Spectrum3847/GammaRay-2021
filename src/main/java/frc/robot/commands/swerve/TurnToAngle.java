/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve;

public class TurnToAngle extends CommandBase {

  //public static double kP = 0.005;
  //public static double kI = 0; // 0.00015
  //public static double kD = 0.00025; // 0.0005
  ProfiledPIDController controller = null;
  Swerve swerve = null;
  boolean hasTarget = false;
  double goal = 0;

  /*Goal between -pi and pi radians
  * Positive is CCW
  */
  public TurnToAngle(double goalRadians) {
    goal = goalRadians;
    swerve = RobotContainer.swerve;
    controller = swerve.thetaController;
    goalRadians = MathUtil.angleModulus(goalRadians); //Make sure our goal is -pi to pi
    controller.reset(swerve.getRadians());
    controller.setGoal(goal);
    controller.setTolerance(0.1); //Tolearance of 2 degrees
    swerve.setRotationalVelocity(controller.calculate(swerve.getRadians()));
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    //calculate the deserged rotational velocity of the robot
    //calculate does input modulus for us
    controller.setGoal(goal);
    swerve.setRotationalVelocity(controller.calculate(swerve.getRadians()));
  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.swerve.setRotationalVelocity(0);
    controller.reset(swerve.getRadians());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return controller.atGoal();
  }
}

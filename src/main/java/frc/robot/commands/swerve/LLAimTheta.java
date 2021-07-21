/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.util.Debugger;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve;

//Should allow us to use our thetaController to aim the robot with the limelight
public class LLAimTheta extends CommandBase {
  ProfiledPIDController controller = null;
  Swerve swerve = null;
  boolean hasTarget = false;
  double goal = 0;
  
  public LLAimTheta() {    
    swerve = RobotContainer.swerve;
    controller = swerve.thetaController;
    controller.setTolerance(Math.PI/180); //Tolearance of 1 degrees
  }

  @Override
  public void initialize() {
    hasTarget = RobotContainer.visionLL.getLimelightHasValidTarget();
    controller.reset(swerve.getRadians());
  }

  @Override
  public void execute() {
    goal = RobotContainer.visionLL.getLLRadToTarget() + swerve.getRadians();
    controller.setGoal(goal);
    swerve.setRotationalVelocity(controller.calculate(swerve.getRadians()));
    dashboard();
  }

  public void dashboard() {
    SmartDashboard.putNumber("LL-AIM/TERROR", controller.getPositionError());
  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.swerve.setRotationalVelocity(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return controller.atGoal();
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

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class FeedBalls extends CommandBase {
  private double time;
  private double oldTime = 0;
  private double state = 0;

  /** Creates a new IntakeBalls. */
  public FeedBalls() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.indexer.feed(); //turn on to begin feeding
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    time = Timer.getFPGATimestamp();
    if ((time - oldTime) > RobotContainer.indexer.feedOnPulse && state == 0) {
      oldTime = time;
      state = 1;
      RobotContainer.indexer.stop(); //Turn off once we have been on for too long
    }
    if ((time - oldTime) > RobotContainer.indexer.feedOffPulse && state == 1) {
      //Thing that should be updated every LONG_DELAY
      oldTime = time;
      state = 2;
      RobotContainer.indexer.feedRev(); //Turn on once we have been off for too long
    }    
    if ((time - oldTime) > RobotContainer.indexer.feedRevPulse && state == 2) {
      //Thing that should be updated every LONG_DELAY
      oldTime = time;
      state = 0;
      RobotContainer.indexer.feed(); //Turn on once we have been off for too long
    }
  }   

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.intake.up();
    //intake stops using default command
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.FeedBalls;
import frc.robot.commands.SwerveDrive;
import frc.robot.commands.ballpath.shooterVel;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ThreeBall extends SequentialCommandGroup {
  /** Creates a new ThreeBall. */
  public ThreeBall() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelCommandGroup(
        new RunCommand(() -> RobotContainer.launcher.setHood(RobotContainer.launcher.intitationLineShot)),
        new shooterVel(4500),  //Shooter spinup
        new SequentialCommandGroup(
          new WaitCommand(1),
          new FeedBalls().withTimeout(4).alongWith(new RunCommand(() -> RobotContainer.intake.setManualOutput(0.3), RobotContainer.intake)) //tower feed
        )).withTimeout(4),
        new SwerveDrive(false,0.3,0).withTimeout(1.2), //Drive Fwd
        new SwerveDrive(false,0,0).withTimeout(0.5) //stop
        //new SwerveDrive(false,-0.4,0).withTimeout(1.4), //Drive Backwards
        //new SwerveDrive(false,0,0).withTimeout(0.5) //stop
    );
  }
}

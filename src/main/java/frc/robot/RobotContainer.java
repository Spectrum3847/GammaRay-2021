// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.lib.controllers.SpectrumXboxController;
import frc.robot.commands.IntakeBalls;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Tower;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    //Create Joysticks first so they can be used in defaultCommands
    public static SpectrumXboxController driverController = new SpectrumXboxController(0, .1, .1);
    public static SpectrumXboxController operatorController = new SpectrumXboxController(1, .06, .05);
  
  // The robot's subsystems and commands are defined here...

  public static Swerve swerve = new Swerve();
  public static Intake intake = new Intake();
  public static Indexer indexer = new Indexer();
  public static Tower tower = new Tower();
  public static Launcher launcher = new Launcher();
  public static Climber climber = new Climber();
  public static DriverStation DS;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    DS = DriverStation.getInstance();
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    operatorController.leftTriggerButton.whileHeld(new IntakeBalls());
    operatorController.selectButton.whileHeld(new RunCommand(() -> indexer.feed(), indexer));
    operatorController.startButton.whileHeld(new RunCommand(()-> tower.setPercentModeOutput(0.5), tower));
    operatorController.yButton.whileHeld(new RunCommand(() -> launcher.setPercentModeOutput(0.5), launcher));
    operatorController.Dpad.Down.whenPressed(new RunCommand(() -> launcher.setHood(1.0), launcher));
    operatorController.Dpad.Down.whenReleased(new RunCommand(() -> launcher.setHood(0), launcher));
  }


  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}

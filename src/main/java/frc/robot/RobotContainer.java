// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.lib.controllers.SpectrumTwoButton;
import frc.lib.controllers.SpectrumXboxController;
import frc.lib.drivers.EForwardableConnections;
import frc.lib.util.Debugger;
import frc.lib.util.SpectrumPreferences;
import frc.robot.commands.FeedBalls;
import frc.robot.commands.IntakeBalls;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.auto.ThreeBall;
import frc.robot.commands.swerve.ClimberSwerve;
import frc.robot.commands.swerve.LLAim;
import frc.robot.commands.swerve.TurnToAngle;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Tower;
import frc.robot.subsystems.VisionLL;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    //Create Joysticks first so they can be used in defaultCommands
    public static SpectrumXboxController driverController = new SpectrumXboxController(0, .15, .15);
    public static SpectrumXboxController operatorController = new SpectrumXboxController(1, .06, .05);
  
  // The robot's subsystems and commands are defined here...

  public static final Swerve swerve = new Swerve();
  public static final Intake intake = new Intake();
  public static final Indexer indexer = new Indexer();
  public static final Tower tower = new Tower();
  public static final Launcher launcher = new Launcher();
  public static final Climber climber = new Climber();
  public static final VisionLL visionLL = new VisionLL(); 
  public static final Compressor compressor = new Compressor();

  public static DriverStation DS;
  public static PowerDistributionPanel pdp = new PowerDistributionPanel();

  public static SpectrumPreferences prefs = SpectrumPreferences.getInstance();
public static Object shooter;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    DS = DriverStation.getInstance();
    portForwarding();
    printInfo("Start robotInit()");
    Dashboard.intializeDashboard();
    configureButtonBindings();
    compressor.start();
    printInfo("End robotInit()");
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //Driver Controls
    //Reset Gyro based on left bumper and the abxy buttons
    new SpectrumTwoButton(driverController.leftBumper, driverController.yButton).whileHeld(new ResetGyro(0));
    new SpectrumTwoButton(driverController.leftBumper, driverController.xButton).whileHeld(new ResetGyro(90));
    new SpectrumTwoButton(driverController.leftBumper, driverController.aButton).whileHeld(new ResetGyro(180));
    new SpectrumTwoButton(driverController.leftBumper, driverController.bButton).whileHeld(new ResetGyro(270));

    //turn the robot to a cardinal direction
    driverController.yButton.whileHeld(new TurnToAngle(0));
    driverController.xButton.whileHeld(new TurnToAngle(90));
    driverController.aButton.whileHeld(new TurnToAngle(180));
    driverController.bButton.whileHeld(new TurnToAngle(270));

    //Climber mode to disable field relative
    driverController.startButton.whileHeld(new ClimberSwerve());

    //Aim with limelight
    driverController.rightBumper.whileHeld(new LLAim());

    //Intake
    operatorController.leftTriggerButton.whileHeld(new IntakeBalls());
    
    //Indexer
    operatorController.selectButton.whileHeld(new FeedBalls());
    operatorController.selectButton.whileHeld(new RunCommand(() -> intake.setManualOutput(0.3), intake));

    //Tower
    operatorController.startButton.whileHeld(new RunCommand(()-> tower.setPercentModeOutput(0.5), tower));

    //Don't use
    new SpectrumTwoButton(operatorController.rightTriggerButton, operatorController.aButton).whileHeld(
      new RunCommand(()-> launcher.setLauncherVelocity(3500), launcher).alongWith(
        new RunCommand(()-> tower.setTowerVelocity(1700))));
  
    //Trench
    new SpectrumTwoButton(operatorController.rightTriggerButton, operatorController.bButton).whileHeld(
      new RunCommand(()-> launcher.setLauncherVelocity(5000), launcher).alongWith(
        new RunCommand(()-> tower.setTowerVelocity(1700))));

    //Intiantion line
    new SpectrumTwoButton(operatorController.rightTriggerButton, operatorController.xButton).whileHeld(
      new RunCommand(()-> launcher.setLauncherVelocity(4500), launcher).alongWith(
        new RunCommand(()-> tower.setTowerVelocity(1700))));

    //Hood
    operatorController.Dpad.Up.whenPressed(new RunCommand(() -> launcher.setHood(1.0), launcher));
    operatorController.Dpad.Down.whenPressed(new RunCommand(() -> launcher.setHood(0), launcher));
    operatorController.Dpad.Left.whenPressed(new RunCommand(() -> launcher.setHood(0.33), launcher));
    operatorController.Dpad.Right.whenPressed(new RunCommand(() -> launcher.setHood(0.66), launcher));

    //Unjam all the things
    operatorController.leftBumper.whileHeld(new RunCommand(() -> intake.setManualOutput(-0.5)).alongWith(
      new RunCommand(() -> indexer.setManualOutput(-indexer.feedSpeed), indexer).alongWith(
        new RunCommand(()-> tower.setPercentModeOutput(-0.3), tower).alongWith(
          new RunCommand(() -> launcher.setPercentModeOutput(-0.5), launcher).alongWith(
              new StartEndCommand(() -> intake.down(), () -> intake.up())) 
        )
      )
    ));

    operatorController.yButton.whileHeld(new RunCommand(()->climber.setManualOutput(-1), climber));
  }

  //foward limelight ports
  private void portForwarding() {
    EForwardableConnections.addPortForwarding(EForwardableConnections.LIMELIGHT_CAMERA_FEED);
    EForwardableConnections.addPortForwarding(EForwardableConnections.LIMELIGHT_WEB_VIEW);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new ThreeBall();
  }

  public static void printDebug(String msg){
    Debugger.println(msg, Robot._general, Debugger.debug2);
  }
  
  public static void printInfo(String msg){
    Debugger.println(msg, Robot._general, Debugger.info3);
  }
  
  public static void printWarning(String msg) {
    Debugger.println(msg, Robot._general, Debugger.warning4);
  }
}

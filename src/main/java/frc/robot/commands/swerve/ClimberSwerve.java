//Based on Code from Team364 - BaseFalconSwerve
//https://github.com/Team364/BaseFalconSwerve/tree/338c0278cb63714a617f1601a6b9648c64ee78d1

package frc.robot.commands.swerve;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class ClimberSwerve extends CommandBase {

    private double rotation;
    private Translation2d translation;
    private boolean fieldRelative;
    private boolean openLoop;
    
    private Swerve s_Swerve;

    /**
     * Driver control
     */
    public ClimberSwerve() {
        this.s_Swerve = RobotContainer.swerve;
        addRequirements(s_Swerve);
        this.fieldRelative = false;
        this.openLoop = false;
    }

    @Override
    public void execute() {
        double yAxis = RobotContainer.driverController.leftStick.getY();
        double xAxis = RobotContainer.driverController.leftStick.getX() * -1;
        double rAxis = RobotContainer.driverController.triggers.getTwist() * -1;

        
        translation = new Translation2d(yAxis, xAxis).times(Constants.Swerve.maxSpeed);
        rotation = rAxis * Constants.Swerve.maxAngularVelocity;
        s_Swerve.drive(translation, rotation, fieldRelative, openLoop);
    }
}

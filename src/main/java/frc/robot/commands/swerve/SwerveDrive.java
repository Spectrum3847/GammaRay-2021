//Based on Code from Team364 - BaseFalconSwerve
//https://github.com/Team364/BaseFalconSwerve/tree/338c0278cb63714a617f1601a6b9648c64ee78d1

package frc.robot.commands.swerve;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class SwerveDrive extends CommandBase {

    private double rotation;
    private Translation2d translation;
    private boolean fieldRelative;
    private boolean openLoop;

    private Swerve s_Swerve;
    private double m_x;
    private double m_y;

    /**
     * Driver control
     */
    public SwerveDrive(boolean fieldRelative, double y, double x) {
        this.s_Swerve = RobotContainer.swerve;
        addRequirements(s_Swerve);
        this.fieldRelative = fieldRelative;
        this.openLoop = false;
        m_y = y;
        m_x = x;
    }

    @Override
    public void execute() {
        double yAxis = m_y;
        double xAxis = m_x;
        double rAxis = 0;

        translation = new Translation2d(yAxis, xAxis).times(Constants.Swerve.maxSpeed);
        rotation = rAxis * Constants.Swerve.maxAngularVelocity;
        s_Swerve.drive(translation, rotation, fieldRelative, openLoop);
    }
}

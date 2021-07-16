//Based on Code from Team364 - BaseFalconSwerve
//https://github.com/Team364/BaseFalconSwerve/tree/338c0278cb63714a617f1601a6b9648c64ee78d1

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;

import frc.lib.util.Debugger;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.TeleopSwerve;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public PigeonIMU gyro;
    public double pidTurn = 0;
    public boolean limelightAim = false;
    public double drive_x = 0;
    public double drive_y = 0;
    public double drive_rotation = 0;

    public Swerve() {
        gyro = new PigeonIMU(Constants.Swerve.pigeonID);
        gyro.configFactoryDefault();
        zeroGyro();
        
        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw());

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        setDefaultCommand(new TeleopSwerve(this, true, false));
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        //If pidTurn is getting a value override the drivers steering control
        if (pidTurn != 0) {
            rotation = pidTurn;
        }
        
        if (Math.abs(rotation) < 0.03){
            rotation = 0;
        }
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getYaw()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
        drive_y = translation.getY();
        drive_x = translation.getX();
        drive_rotation = rotation;
    }

    public void useOutput(double output) {
            pidTurn = output * Constants.Swerve.maxAngularVelocity;
            printDebug("LL Output: " + output);
            printDebug("LL pidTurn: " + pidTurn);
      }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.normalizeWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }    

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(pose, getYaw());
    }

    public SwerveModuleState[] getStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public void zeroGyro(){
        gyro.setYaw(0);
    }

    //Set gyro to a specific value
    public void setGyro(double value){
        gyro.setYaw(value);
    }

    public Rotation2d getYaw() {
        double[] ypr = new double[3];
        gyro.getYawPitchRoll(ypr);
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - ypr[0]) : Rotation2d.fromDegrees(ypr[0]);
    }

    public double getDegrees() {
        return getYaw().getDegrees();
    }

    public void dashboard(){
        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Swerve/Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Swerve/Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
            SmartDashboard.putNumber("Swerve/Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }
        SmartDashboard.putNumber("Swerve/Gyro Yaw", getYaw().getDegrees());
        SmartDashboard.putNumber("Swerve/Drive Y", drive_y);
        SmartDashboard.putNumber("Swerve/Drive X", drive_x);
        SmartDashboard.putNumber("Swerve/Drive Rotation", drive_rotation);

    }
    
    @Override
    public void periodic(){
        swerveOdometry.update(getYaw(), getStates());  
    }

    public static void printDebug(String msg){
        Debugger.println(msg, Robot._drive, Debugger.debug2);
      }
      
      public static void printInfo(String msg){
        Debugger.println(msg, Robot._drive, Debugger.info3);
      }
      
      public static void printWarning(String msg) {
        Debugger.println(msg, Robot._drive, Debugger.warning4);
      }
}
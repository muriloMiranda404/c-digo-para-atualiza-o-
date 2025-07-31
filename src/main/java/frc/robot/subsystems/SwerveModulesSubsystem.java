package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveModules;

public class SwerveModulesSubsystem extends SubsystemBase{

    private static SwerveModulesSubsystem modules = new SwerveModulesSubsystem();

    public static SwerveModulesSubsystem getInstance(){
        if(modules == null){
            return new SwerveModulesSubsystem();
        }
        return modules;
    }
    
    private final SwerveModules backRight;
    private final SwerveModules backLeft;
    private final SwerveModules frontRight;
    private final SwerveModules frontLeft;

    private final SwerveDriveKinematics kinematics;

    public SwerveModulesSubsystem(){

        frontLeft = new SwerveModules(3, 4, 10, 249.78);
        frontRight = new SwerveModules(6, 5, 11, 181.31);
        backLeft = new SwerveModules(1, 2, 13, 72.59);
        backRight = new SwerveModules(8, 7, 12, 50.0);

        kinematics = new SwerveDriveKinematics(
        new Translation2d(0.0, 0.5),//FL
        new Translation2d(0.5, -0.5),//FR
        new Translation2d(-0.5, 0.5),//BL
        new Translation2d(-0.5, -0.5)//BR
        );
    }

    public SwerveModules getFrontRight(){
        return frontRight;
    }
    public SwerveModules getFrontLeft(){
        return frontLeft;
    }
    public SwerveModules getBackLeft(){
        return backLeft;
    }
    public SwerveModules getBackRight(){
        return backRight;
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("FrontLeft", frontLeft.getPosition());
        SmartDashboard.putNumber("backLeft", backLeft.getPosition());
        SmartDashboard.putNumber("Frontright", frontRight.getPosition());
        SmartDashboard.putNumber("BackRight", backRight.getPosition());
    }
}

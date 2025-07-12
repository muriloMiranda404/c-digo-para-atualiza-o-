package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveModules;
import frc.robot.Constants.IDs;

public class SwerveModulesSubsystem extends SubsystemBase{
    
    private final SwerveModules backRight;
    private final SwerveModules backLeft;
    private final SwerveModules frontRight;
    private final SwerveModules frontLeft;

    private final Pigeon2 pigeon = new Pigeon2(IDs.PIGEON2);
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
}

package frc.robot;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.AngleUnit;

public class SwerveModules {
    
    private SparkMax driveMotor;
    private SparkMax angleMotor;

    private CANcoder absoluteEncoder;

    private PIDController drivePID;
    private PIDController anglePID;

    private int driveID;
    private int angleID;
    private int encoderID;
    private double encoder_offset;

    public SwerveModules(int driveID, int angleID, int encoderID, double encoder_offset){
        this.driveMotor = new SparkMax(driveID, SparkMax.MotorType.kBrushless);
        this.angleMotor = new SparkMax(angleID, SparkMax.MotorType.kBrushless);
        this.absoluteEncoder = new CANcoder(encoderID);
        this.encoder_offset = encoder_offset;

        this.drivePID = new PIDController(0.01, 0, 0);
        this.anglePID = new PIDController(0.01, 0, 0);
        this.anglePID.enableContinuousInput(-180, 180);
    }

    public double getPosition(){
        return absoluteEncoder.getAbsolutePosition().getValueAsDouble() - encoder_offset;
    }

    public double getVelocity(){
        return driveMotor.get();
    }
}

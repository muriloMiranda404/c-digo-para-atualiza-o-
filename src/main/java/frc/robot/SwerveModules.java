package frc.robot;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.constants.Constants.Swerve;

public class SwerveModules {
    
    private static SparkMax driveMotor;
    private static SparkMax angleMotor;

    private static CANcoder absoluteEncoder;

    private static PIDController drivePID;
    private static PIDController anglePID;

    private static int driveID;
    private static int angleID;
    private static int encoderID;
    private static double encoder_offset;

    private static Pigeon2 pigeon = new Pigeon2(9);

    public SwerveModules(int driveID, int angleID, int encoderID, double encoder_offset){
        this.driveMotor = new SparkMax(driveID, SparkMax.MotorType.kBrushless);
        this.angleMotor = new SparkMax(angleID, SparkMax.MotorType.kBrushless);
        this.absoluteEncoder = new CANcoder(encoderID);
        this.encoder_offset = encoder_offset;

        this.drivePID = Swerve.drivePID;
        this.anglePID = Swerve.anglePID;
        this.anglePID.enableContinuousInput(-180, 180);
    }


    public double getPosition(){
        return absoluteEncoder.getAbsolutePosition().getValueAsDouble() - encoder_offset;
    }

    public double getVelocity(){
      return driveMotor.get();
    }
    
    public void setPosition(double position){
        absoluteEncoder.setPosition(position);
        System.out.println("a posição agora é " + getPosition());
    }
}

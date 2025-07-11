package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.robot.Constants.Elevator;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsytem extends SubsystemBase{
    
    public static ElevatorSubsytem elevator;
    
    public static ElevatorSubsytem getInstance(){
        if( elevator == null){
            return new ElevatorSubsytem();
        }
        return elevator;
    }

    public static  SparkMaxConfig rightMotorConfig;
    public static  SparkMaxConfig leftMotorConfig;
    
    public static  SparkMax rightMotor;
    public static  SparkMax leftMotor;
    
    public static  PIDController controller;
    
        public static  DigitalInput upSwitch;
        public static  DigitalInput downSwitch;

        public static Encoder encoder;

        
        private ElevatorSubsytem(){
            
            rightMotorConfig = new SparkMaxConfig();
            leftMotorConfig = new SparkMaxConfig();
            
            rightMotorConfig
            .inverted(false)
            .idleMode(IdleMode.kCoast);

            leftMotorConfig
            .inverted(true)
            .idleMode(IdleMode.kCoast);

            leftMotor = new SparkMax(Elevator.LEFT_MOTOR, SparkMax.MotorType.kBrushless);
            rightMotor = new SparkMax(Elevator.RIGHT_MOTOR, SparkMax.MotorType.kBrushless);

            leftMotor.configure(leftMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
            rightMotor.configure(rightMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
            
            controller = Elevator.ELEVATOR_PID;
            controller.setTolerance(Elevator.TOLERANCE_ELEVATOR);

            upSwitch = new DigitalInput(Elevator.UP_SWITCH);
            downSwitch = new DigitalInput(Elevator.DOWN_SWITCH);
            encoder = new Encoder(Elevator.ENCODER_A, Elevator.ENCODER_B);

            encoder.setDistancePerPulse(360/2048);
            encoder.setReverseDirection(true);
            encoder.reset();
        }
        

        public void setSpeed(double speed){
            leftMotor.set(speed);
            rightMotor.set(speed);
        }
        
        public double getDistance(){
            return encoder.getDistance();
        }

        public void setPosition(double setpoint){

            double angulo = getDistance();
            double output = controller.calculate(setpoint, angulo);

            if(upSwitch.get()){
                if (setpoint > 1480.0) setpoint = 1480.0;
                if (output > 0) output = 0.0;
            }
            if(downSwitch.get()){
                if (setpoint < 0.0) setpoint = 0.0;
                if (output < 0) output = 0.0;
            }

            leftMotor.set(output);
            rightMotor.set(output);
           }
           
           public double getSpeed(){
            return encoder.getRate();
           }

           public void stopMotor(){
            leftMotor.stopMotor();
            rightMotor.stopMotor();
           }
        public void periodic(){
        SmartDashboard.putNumber("speed", getSpeed());
        SmartDashboard.putNumber("angulo", getDistance());
        SmartDashboard.putBoolean("fim de curso alto", upSwitch.get());
        SmartDashboard.putBoolean("fim de curso baixo", downSwitch.get());
 }
}

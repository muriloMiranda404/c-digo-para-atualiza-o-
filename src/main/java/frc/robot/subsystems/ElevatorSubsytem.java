package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.Elevator;

public class ElevatorSubsytem extends SubsystemBase{

    public static SparkMaxConfig rightMotorConfig;
    public static SparkMaxConfig leftMotorConfig;
    
    public static SparkMax rightMotor;
    public static SparkMax leftMotor;
    
    public static PIDController controller;
    
    public static DigitalInput upSwitch;
    public static DigitalInput downSwitch;

    public static Encoder encoder;

    Timer timer = new Timer();

        
    public ElevatorSubsytem(){
            
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
        }
        

        public void setSpeed(double speed){
            leftMotor.set(speed);
            rightMotor.set(speed);
        }
        
        public double getDistance(){
            return encoder.getDistance();
        }

        public double calculateOutput(double medido, double setpoint){
            double output = controller.calculate(medido, setpoint);

            return output;
        }
        
        public void setOutput(double output){
            leftMotor.set(output);
            rightMotor.set(output);
           }
           
       public double getSpeed(){
            return encoder.getRate();
       }

       public void stopMotor(){
            leftMotor.setVoltage(0);
            rightMotor.setVoltage(0);
        }

        public void resetEncoder(){
            encoder.reset();
        }

        public Encoder getElevatorEncoder(){
            return encoder;
        }

        public PIDController getPID(){
            return controller;
        }

        public DigitalInput getUpSwicth(){
            return upSwitch;
        }

        public DigitalInput getDownSwicth(){
            return downSwitch;
        }

        public boolean atSetpoint(){
            return controller.atSetpoint();
        }
         
        @Override
        public void periodic(){
        SmartDashboard.putNumber("speed", getSpeed());
        SmartDashboard.putNumber("angulo", getDistance());
        SmartDashboard.putBoolean("fim de curso alto", upSwitch.get());
        SmartDashboard.putBoolean("fim de curso baixo", downSwitch.get());
 }
}

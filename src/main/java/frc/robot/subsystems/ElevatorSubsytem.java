package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DriveConstants.Elevator;

public class ElevatorSubsytem extends SubsystemBase{

    private static SparkMaxConfig rightMotorConfig;
    private static SparkMaxConfig leftMotorConfig;
    
    private static SparkMax rightMotor;
    private static SparkMax leftMotor;
    
    private static PIDController controller;
    
    private static DigitalInput upSwitch;
    private static DigitalInput downSwitch;

    private static Encoder encoder;

    private double setpoint;

    public static ElevatorSubsytem elevatorSubsytem = new ElevatorSubsytem();

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
    }
    
    public static ElevatorSubsytem getInstance(){
        if(elevatorSubsytem == null){
            return new ElevatorSubsytem();
        } 
        return elevatorSubsytem;
    }
        public void setSpeed(double speed){
            leftMotor.set(speed);
            rightMotor.set(speed);
        }

        public void configureElevator(){
            encoder.setDistancePerPulse(Elevator.PULSE1/Elevator.PULSE2);
            encoder.setReverseDirection(Elevator.INVERTED);
        }
        
        public double getDistance(){
            return encoder.getDistance();
        }
        
        public void setSetpoint(double setpoint){

        this.setpoint = setpoint;

        double ang = getDistance();
        double output = controller.calculate(ang, setpoint);

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
            leftMotor.setVoltage(0);
            rightMotor.setVoltage(0);
        }

        public void resetEncoder(){
            encoder.reset();
        }

        public boolean atSetpoint(){
            return controller.atSetpoint();
        }

        public String getPosition(){
            String position;

            switch ((int) setpoint) {

                case (int)Elevator.L1_POSITION:

                    position = "POSIÇÃO DE L1";
                    break;
                
                case (int)Elevator.L2_POSITION:

                    position = "POSIÇÃO DO L2";
                    break;
                 
                case (int)Elevator.L3_POSITION:
                
                    position = "POSIÇÃO DO L3";
                    break;

                case (int)Elevator.L4_POSITION:
                    
                    position = "POSIÇÃO DO L4";
                    break;

                case (int)Elevator.L2_ALGAE:
                
                    position = "ALGA DO L2";
                    break;

                case (int)Elevator.L3_ALGAE:
                
                    position = "ALGA DO L3";
                    break;

                default:
                    position = "POSIÇÃO NÃO IDENTIFICADA";
                    break;
            }

            return position;
        }
         
        @Override
        public void periodic(){
        SmartDashboard.putNumber("speed", getSpeed());
        SmartDashboard.putNumber("angulo", getDistance());
        SmartDashboard.putString("posição", getPosition());
        SmartDashboard.putBoolean("fim de curso alto", upSwitch.get());
        SmartDashboard.putBoolean("fim de curso baixo", downSwitch.get());
 }
}

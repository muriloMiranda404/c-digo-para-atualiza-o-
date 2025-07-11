package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.Intake;

public class IntakeSubsystem extends SubsystemBase{
    
    public SparkMax intake = new SparkMax(Intake.INTAKE_MOTOR, SparkMax.MotorType.kBrushless);
    public SparkMax algae = new SparkMax(Intake.ALGAE_MOTOR, SparkMax.MotorType.kBrushless);

    public PIDController controller;

    public DutyCycleEncoder encoder;
    public DigitalInput algae_swicth;

    public IntakeSubsystem(){

        controller = Intake.INTAKE_PID;
        controller.setTolerance(Intake.INTAKE_TOLERANCE);

        encoder = new DutyCycleEncoder(Intake.INTAKE_ENCODER);
        encoder.setDutyCycleRange(0, 360);

        algae_swicth = new DigitalInput(Intake.ALGAE_SWICTH);
    }
    public void setSpeed(double speed){
        algae.set(speed);
    }
    public double getDistance(){
        return encoder.get() * 360;
    }
    public void setPosition(double setpoint){
        double ang = getDistance();
        double output = controller.calculate(ang, setpoint);

        if (ang < Intake.MIN_INTAKE){ 
            if (output > 0.0) output = 0.0;

            if (setpoint < Intake.MIN_INTAKE) setpoint = Intake.MIN_INTAKE;
        }

        if (ang > Intake.MAX_INTAKE){ 
            if (output > 0.0) output = 0.0;

            if (setpoint > Intake.MAX_INTAKE) setpoint = Intake.MAX_INTAKE;
        }
        
        intake.set(output);

        if(Robot.trava == true){
            algae.set(0.2);
        }

        if(!algae_swicth.get() && Robot.trava == true){
            Robot.trava = false;
            algae.set(0);
        }
    }
    public void stopMotor(){
        algae.stopMotor();
        intake.stopMotor();
    }
}

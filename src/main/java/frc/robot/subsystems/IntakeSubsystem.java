package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.Constants.Intake;

public class IntakeSubsystem extends SubsystemBase{
    
    public SparkMax intake;
    public SparkMax coral;

    public PIDController controller;

    public DutyCycleEncoder encoder;
    public DigitalInput algae_swicth;

    public SparkMaxConfig intakeConfig;
    public SparkMaxConfig coralConfig;
    public SparkMaxConfig globalCofig;

    private static IntakeSubsystem subsystem;
    
    public static IntakeSubsystem getInstance(){
        if(subsystem == null){
            return new IntakeSubsystem();
        }
        return subsystem;
    }

    private IntakeSubsystem(){

       intake = new SparkMax(Intake.INTAKE_MOTOR, SparkMax.MotorType.kBrushless);
       coral = new SparkMax(Intake.ALGAE_MOTOR, SparkMax.MotorType.kBrushless);

       intakeConfig = new SparkMaxConfig();
       coralConfig = new SparkMaxConfig();
       globalCofig = new SparkMaxConfig();

       globalCofig
       .idleMode(IdleMode.kBrake);

       intakeConfig
       .inverted(true)
       .apply(globalCofig);

       coralConfig
       .inverted(false)
       .apply(globalCofig);

       intake.configure(intakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
       coral.configure(coralConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        controller = Intake.INTAKE_PID;
        controller.setTolerance(Intake.INTAKE_TOLERANCE);

        encoder = new DutyCycleEncoder(Intake.INTAKE_ENCODER);
        encoder.setDutyCycleRange(Intake.MIN_ENCODER, Intake.MAX_ENCODER);

        algae_swicth = new DigitalInput(Intake.ALGAE_SWICTH);
    }


    public void setSpeed(double speed){
        coral.set(speed);
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
            coral.set(0.2);
        }

        if(!algae_swicth.get() && Robot.trava == true){
            Robot.trava = false;
            coral.set(0);
        }
    }
    public void stopMotor(){
        coral.stopMotor();
        intake.stopMotor();
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("angulo", getDistance());
        SmartDashboard.putBoolean("fim de curso", algae_swicth.get());
    }
}
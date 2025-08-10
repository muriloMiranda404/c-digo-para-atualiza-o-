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
import frc.robot.constants.DriveConstants.Intake;

public class IntakeSubsystem extends SubsystemBase{
    
    private static SparkMax intake;
    private static SparkMax coral;

    private static PIDController controller;

    private static DutyCycleEncoder encoder;
    private static DigitalInput algae_swicth;

    private static SparkMaxConfig intakeConfig;
    private static SparkMaxConfig coralConfig;
    private static SparkMaxConfig globalCofig;

    private static double setpoint;

    public static IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

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

        encoder = new DutyCycleEncoder(Intake.INTAKE_ENCODER);

        algae_swicth = new DigitalInput(Intake.ALGAE_SWICTH);

    }

    public static IntakeSubsystem getInstance(){
        if(intakeSubsystem == null){
            return new IntakeSubsystem();
        }
        return intakeSubsystem;
    }

    public void setSpeed(double speed){
        coral.set(speed);
    }

    public double getDistance(){
        return encoder.get() * 360;
    }

    public double getSpeed(){
        return coral.get();
    }
    
    public void setPosition(double setpoint){

        this.setpoint = setpoint;
        double angulo = getDistance();
        double output = controller.calculate(angulo, setpoint);

        if(angulo < Intake.MIN_INTAKE){
            if(output > 0) output = 0.0;

            if(setpoint > Intake.MIN_INTAKE) setpoint = Intake.MIN_INTAKE;
        }

        if(angulo > Intake.MAX_INTAKE){
            if(output > 0.0) output = 0.0;

            if(setpoint > Intake.MAX_INTAKE) setpoint = Intake.MAX_INTAKE;
        }

       intake.set(output);
    }

    public void stopMotor(){
        coral.setVoltage(0);
        intake.setVoltage(0);
    }

    public boolean CoralDetected(){
        return algae_swicth.get();
    }

    public boolean atSetpoint(){
        return controller.atSetpoint();
    }

    public String getPosition(){
        String position;

        switch ((int) setpoint) {
            case (int)Intake.ABERTURA_L1:
                
                position = "ABERTURA DO L1";
                break;
        
            case (int)Intake.ABERTURA_COMUM:
            
                position = "POSIÇÃO DE ABERTURA COMUM";
                break;

            case (int)Intake.ALGAE_POSITION:
            
                position = "POSIÇÃO DE ALGAS";
                break;

            case (int)Intake.CORAL_L4:
            
                position = "POSIÇÃO DO L4";
                break;

            case (int)Intake.MAX_INTAKE:
            
                position = "INTAKE NA POSIÇÃO MAXIMA";
                break;

            case (int)Intake.MIN_INTAKE:
            
                position = "INTAKE NA POSIÇÃO MINIMA";
                break;

            default:

                position = "POSIÇÃO NÃO INDENTIFICADA";
                break;
        }
        return position;
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("angulo", getDistance());
        SmartDashboard.putNumber("speed", getSpeed());
        SmartDashboard.putString("posição", getPosition());
        SmartDashboard.putBoolean("fim de curso", algae_swicth.get());
    }
}

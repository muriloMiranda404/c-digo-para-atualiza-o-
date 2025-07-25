package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants.Intake;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakePosition extends Command{
    
    double setpoint;
    IntakeSubsystem intake;

    public IntakePosition(IntakeSubsystem intake, double setpoint){
        this.setpoint = setpoint;
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize(){
        
    }
    @Override
    public void execute(){

      try{  
        double angulo = intake.getDistance();
        double output = intake.calculateOutput(angulo, setpoint);

        if(angulo < Intake.MIN_INTAKE){
            if(output > 0) output = 0.0;

            if(setpoint > Intake.MIN_INTAKE) setpoint = Intake.MIN_INTAKE;
        }

        if(angulo > Intake.MAX_INTAKE){
            if(output > 0.0) output = 0.0;

            if(setpoint > Intake.MAX_INTAKE) setpoint = Intake.MAX_INTAKE;
        }

      intake.setPosition(output);
      
    } catch(Exception e){
        intake.stopMotor();
        System.out.println("erro detectado" + e);
    }
    
    }
    @Override
    public boolean isFinished(){
        return intake.atSetpoint();
    }
    @Override
    public void end(boolean interrupted){
        intake.stopMotor();
    }
}

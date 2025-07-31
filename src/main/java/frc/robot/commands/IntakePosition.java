package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakePosition extends Command{
    
    double setpoint;
    IntakeSubsystem intake;

    public IntakePosition(IntakeSubsystem intake, double setpoint){
        if(intake == null){
            throw new IllegalArgumentException("o subsystema não pode ser nulo");
        }
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

        intake.setPosition(setpoint);
        
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

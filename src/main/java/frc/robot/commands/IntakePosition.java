package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakePosition extends Command{
    
    double setpoint;
    IntakeSubsystem subsystem;
    double speed;
    int acao;

    public IntakePosition(IntakeSubsystem subsystem, double setpoint){
        this.setpoint = setpoint;
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    public void initialize(){
        
    }
    public void execute(){

      try{  
      subsystem.setPosition(setpoint);
    } catch(Exception e){
        System.out.println("erro detectado" + e);
    }
    }
    public boolean isFinished(){
        return subsystem.controller.atSetpoint();
    }
    public void end(boolean interrupted){
        subsystem.stopMotor();
    }
}

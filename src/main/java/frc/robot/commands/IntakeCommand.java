package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command{
    
    double setpoint;
    IntakeSubsystem subsystem;
    double speed;
    int acao;

    public IntakeCommand(IntakeSubsystem subsystem, double setpoint, Double speed, int acao){
        if(acao >= 3){
            throw new IllegalArgumentException("o valor da ação não pode ser maior do que 3");
        }
        this.setpoint = setpoint;
        this.subsystem = subsystem;
        this.speed = speed;
        this.acao = acao;
        addRequirements(subsystem);
    }

    public void initialize(){
        
    }
    public void execute(){
      try{  
        if(acao == 1){
            subsystem.setPosition(setpoint);
        } else if(acao == 2){
            subsystem.setSpeed(speed);
        }
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

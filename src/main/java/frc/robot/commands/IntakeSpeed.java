package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeSpeed extends Command{

    IntakeSubsystem subsystem;
    double speed;

    public IntakeSpeed(IntakeSubsystem subsystem, double speed){
        this.subsystem = subsystem;
        this.speed = speed;
        addRequirements(subsystem);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){

        try{
            subsystem.setSpeed(speed);
        } catch(Exception e){
            System.out.println("erro detectado " + e);
        }
    }

    @Override
    public boolean isFinished(){
        return false;
    }
    
    @Override
    public void end(boolean interrupted){
        subsystem.stopMotor();
    }
}
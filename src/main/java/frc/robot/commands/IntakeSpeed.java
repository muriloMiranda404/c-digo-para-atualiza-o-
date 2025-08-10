package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeSpeed extends Command{

    IntakeSubsystem intakeSubsystem;
    double speed;
    boolean stop;

    public IntakeSpeed(IntakeSubsystem intakeSubsystem, double speed){
        this(intakeSubsystem, speed, false);
    }

    public IntakeSpeed(IntakeSubsystem intakeSubsystem, boolean stop){
        this(intakeSubsystem, 0.2, stop);
    }

    private IntakeSpeed(IntakeSubsystem intakeSubsystem, double speed, boolean stop){
        if(intakeSubsystem == null){
            throw new IllegalArgumentException("o subsystema não pode ser nulo");
        }
        this.stop = stop;
        this.intakeSubsystem = intakeSubsystem;
        this.speed = speed;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        
        try{
            boolean parar = false;

            intakeSubsystem.setSpeed(speed);

        } catch(Exception e){
            System.out.println("erro detectado " + e);
        }
    }

    @Override
    public boolean isFinished(){
        boolean parar = false;
        
        if(stop == true){
            if(intakeSubsystem.CoralDetected()) parar = true;
        } else{
            parar = false;
        }

        return false || parar == true;
    }
    
    @Override
    public void end(boolean interrupted){
        intakeSubsystem.stopMotor();
    }
}
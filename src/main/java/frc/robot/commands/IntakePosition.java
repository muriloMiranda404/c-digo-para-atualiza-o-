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

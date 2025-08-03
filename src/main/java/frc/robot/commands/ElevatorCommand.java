package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsytem;

public class ElevatorCommand extends Command{
    
    ElevatorSubsytem elevatorSubsytem;
    double setpoint;

    public ElevatorCommand(ElevatorSubsytem subsytem, double setpoint){
        if(subsytem == null){
            throw new IllegalArgumentException("o subsystema não pode ser nulo");
        }
        this.elevatorSubsytem = subsytem;
        this.setpoint = setpoint;
        addRequirements(subsytem);
    }

    @Override
    public void initialize(){

    }
    @Override
    public void execute(){
        elevatorSubsytem.setSetpoint(setpoint);
    }
    @Override
    public void end(boolean interrupted){
        elevatorSubsytem.stopMotor();
    }
    @Override
    public boolean isFinished(){
        return elevatorSubsytem.atSetpoint();
    }
}

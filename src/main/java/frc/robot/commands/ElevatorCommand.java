package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsytem;

public class ElevatorCommand extends Command{
    
    ElevatorSubsytem elevatorSubsytem;
    PIDController controller;
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
        return controller.atSetpoint();
    }
}

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsytem;

public class ElevatorCommand extends Command{
    
    ElevatorSubsytem elevatorSubsytem;
    PIDController controller;
    double setpoint;

    public ElevatorCommand(ElevatorSubsytem subsytem, double setpoint){
        this.elevatorSubsytem = subsytem;
        this.setpoint = setpoint;
        addRequirements(subsytem);
    }
    public void initialize(){

    }
    public void execute(){
        elevatorSubsytem.setPosition(setpoint);
    }
    public void end(boolean interrupted){
        elevatorSubsytem.stopMotor();
    }
    public boolean isFinished(){
        return controller.atSetpoint();
    }
}

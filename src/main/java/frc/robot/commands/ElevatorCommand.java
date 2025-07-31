package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsytem;

public class ElevatorCommand extends Command{
    
    ElevatorSubsytem elevatorSubsytem;
    PIDController controller;
    double setpoint;
    DigitalInput upSwitch = elevatorSubsytem.getUpSwicth();
    DigitalInput downSwitch = elevatorSubsytem.getDownSwicth();

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
        double ang = elevatorSubsytem.getDistance();
        double output = elevatorSubsytem.calculateOutput(ang, setpoint);

        if(upSwitch.get()){
            if (setpoint > 1480.0) setpoint = 1480.0;
            if (output > 0) output = 0.0;
        }
        if(downSwitch.get()){
            if (setpoint < 0.0) setpoint = 0.0;
            if (output < 0) output = 0.0;
        }

        elevatorSubsytem.setOutput(output);
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

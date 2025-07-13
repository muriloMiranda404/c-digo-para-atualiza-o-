package frc.robot.commands;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class TurnRobot extends Command{
    
   private Pigeon2 pigeon;
   private Translation2d translation;
   private SwerveSubsystem subsystem;
   private double angulacao;
   private PIDController controller;

   public TurnRobot(Pigeon2 pigeon, SwerveSubsystem subsystem, double angulacao){
    this.pigeon = pigeon;
    this.angulacao = angulacao;
    this.subsystem = subsystem;
    this.controller = new PIDController(0.01, 0, 0);
    addRequirements(subsystem);
   }

   @Override
   public void initialize(){
    controller.setTolerance(2.0);
    controller.enableContinuousInput(-180, 180);
   }

   @Override
   public void execute(){

    double yaw = pigeon.getYaw().getValueAsDouble();
    double output = controller.calculate(angulacao, yaw);

    translation = new Translation2d(0, 0);
    subsystem.drive(translation, output, true);
   }

   @Override
   public boolean isFinished(){
    return controller.atSetpoint();
   }

   @Override
   public void end(boolean interrupted){
    subsystem.drive(translation, 0, true);
   }
}

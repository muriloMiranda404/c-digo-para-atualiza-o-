package frc.robot.commands;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Controller;
import frc.robot.subsystems.SwerveSubsystem;

public class TurnRobot extends Command{
    
   private Pigeon2 pigeon;
   private Translation2d translation;
   private SwerveSubsystem subsystem;
   private double angulacao;
   private PIDController controller;
   private XboxController joystick = new XboxController(Controller.DRIVE_CONTROLLER);

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

    try{
    double yaw = pigeon.getYaw().getValueAsDouble();
    double output = controller.calculate(angulacao, yaw);

    if(yaw >= angulacao){
        pigeon.setYaw(angulacao);
        subsystem.drive(translation, 0, true);
    }

    translation = new Translation2d(0, 0);
    subsystem.drive(translation, output, true);

        } catch(Exception e){
            System.out.println("erro ao executar comando");
            this.cancel();
        }
   }

   @Override
   public boolean isFinished(){
    return controller.atSetpoint() || joystick.getAButton();
   }

   @Override
   public void end(boolean interrupted){
    try{
    subsystem.drive(translation, 0, true);
   } catch(Exception e){
    System.out.println("erro ao finalizar");
    this.cancel();
   }
}
}
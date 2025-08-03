package frc.robot.commands;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DriveConstants.Joystick;
import frc.robot.subsystems.SwerveSubsystem;

public class TurnRobot extends Command{
    
   private Pigeon2 pigeon;
   private Translation2d translation;
   private SwerveSubsystem subsystem;
   private double angulacao;
   private PIDController controller;
   private XboxController joystick = new XboxController(Joystick.DRIVE_CONTROLLER);
   private Timer timer;

   private static final double MAX_ROTATION_SPEED = 3.0;
   private static final double MAX_TIMER = 5.0;

   public TurnRobot(Pigeon2 pigeon, SwerveSubsystem subsystem, double angulacao){
    if(subsystem == null){
        throw new IllegalArgumentException("o subsystema não pode ser nulo");
    }
    this.timer = new Timer();
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

    timer.reset();
    timer.start();
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

    output = Math.min(Math.max(output, -MAX_ROTATION_SPEED), MAX_ROTATION_SPEED);

    translation = new Translation2d(0, 0);
    subsystem.drive(translation, output, true);

        } catch(Exception e){
            System.out.println("erro ao executar comando");
            this.cancel();
        }
   }

   @Override
   public boolean isFinished(){
    return controller.atSetpoint() || joystick.getAButton() || timer.hasElapsed(MAX_TIMER);
   }

   @Override
   public void end(boolean interrupted){
    try{
        if(controller.atSetpoint()){
            System.out.println("o comando chegou o objetivo");
            subsystem.drive(translation, 0, true);
        } else if(joystick.getAButton()){
            System.out.println("o botão de cancelamento foi apertado");
            this.cancel();
        } else if(timer.hasElapsed(MAX_TIMER)){
            System.out.println("o tempo foi esgotado");
            this.cancel();
        }
   } catch(Exception e){
    System.out.println("erro ao finalizar");
    this.cancel();
   }
}
}
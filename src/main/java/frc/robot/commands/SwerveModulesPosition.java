package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveModulesSubsystem;

public class SwerveModulesPosition extends Command{
    
    SwerveModulesSubsystem modules;
    double position;

    public SwerveModulesPosition(SwerveModulesSubsystem modules, double position){
        this.position = position;
        this.modules = modules;
        addRequirements(modules);
    }

    @Override
    public void initialize(){
        System.out.println("inicializando a mudança de modulos");
    }

    @Override
    public void execute(){
       try{ 

        modules.getFrontLeft().setPosition(position);
        modules.getBackLeft().setPosition(position);
        modules.getFrontRight().setPosition(position);
        modules.getBackRight().setPosition(position);

        System.out.println("mudando ...");
      } catch(Exception e){
        System.out.println("erro ao executar " + e);
      }
    }

    @Override
    public boolean isFinished(){
        return false;
    }

    @Override
    public void end(boolean interrupted){
        System.out.println("finalizado");
    }
}

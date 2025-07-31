package frc.robot.commands;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class ResetPigeon extends Command{
    
    private final SwerveSubsystem subsystem;
    private final Pigeon2 pigeon;
    private final Timer timer;

    private static final double MAX_TIME = 5.0;

    public ResetPigeon(Pigeon2 pigeon, SwerveSubsystem subsystem){
        if(subsystem == null){
            throw new IllegalArgumentException("o subsystema não pode ser nulo");
        }
        this.timer = new Timer();
        this.pigeon = pigeon;
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }
    
    @Override
    public void execute() {
        try{
        pigeon.reset();
        subsystem.swerveDrive.zeroGyro();
        System.out.println("Pigeon Resetado");
        } catch(Exception e ){
            System.out.println("exeção capturada: " + e.getMessage());
            subsystem.drive(new Translation2d(0, 0), 0, true);
        }
    }
    
    @Override
    public boolean isFinished() {
        return pigeon.getYaw().getValueAsDouble() == 0 || timer.hasElapsed(MAX_TIME);
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.drive(new Translation2d(0, 0), 0, true);
    }
}
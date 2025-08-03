package frc.robot.shuffleboardSettings;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.DriveConstants.IDs;

public class ShuffleboardConfig{
    
    private static SendableChooser<String> teleopChooser;
    private static Pigeon2 pigeon;

    public ShuffleboardConfig(){

        teleopChooser = new SendableChooser<>();

        pigeon = new Pigeon2(IDs.PIGEON2);

        teleopChooser.setDefaultOption("TeleopConfig", "Normal_teleop");
        teleopChooser.addOption("TeleopConfig", "Second_teleop");

        var shuffleboard = Shuffleboard.getTab("config");

        shuffleboard.add(teleopChooser);
        shuffleboard.addCamera("LimelightVision", "limelight", "10.94.85.2");
        shuffleboard.addNumber("Pigeon Valor", () -> pigeon.getYaw().getValueAsDouble());

        shuffleboard.add(CameraServer.startAutomaticCapture());

        DataLogManager.start();
    }

    public String setChoosed(){
        return teleopChooser.getSelected();
    }

    public void getMode(){
        boolean auto;
        boolean teleop;
        
        auto = DriverStation.isAutonomous();
        teleop = DriverStation.isTeleop();

        SmartDashboard.putBoolean("autonomo", auto);
        SmartDashboard.putBoolean("teleop", teleop);
    }

    public void getAlliance(){
        var getAlliance = DriverStation.getAlliance().get();

        boolean alliance = getAlliance == Alliance.Blue;

        SmartDashboard.putBoolean("alliance", alliance);
    }

    public void getMathTime(){
        double time = DriverStation.getMatchTime();

        SmartDashboard.putNumber("tempo", time);

    }
}
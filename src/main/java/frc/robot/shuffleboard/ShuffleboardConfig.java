package frc.robot.shuffleboard;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.constants.Constants.IDs;

public class ShuffleboardConfig{
    
    private static SendableChooser<String> teleopChooser = new SendableChooser<>();  
    private static Pigeon2 pigeon = new Pigeon2(IDs.PIGEON2);

    public ShuffleboardConfig(){
        teleopChooser.setDefaultOption("TeleopConfig", "Normal_teleop");
        teleopChooser.addOption("TeleopConfig", "Second_teleop");

        var shuffleboard = Shuffleboard.getTab("config");
        shuffleboard.add(teleopChooser);
        shuffleboard.addCamera("LimelightVision", "limelight", "10.94.85.2");
        shuffleboard.addNumber("Pigeon Valor", () -> pigeon.getYaw().getValueAsDouble());

        DataLogManager.start();
    }

    public static String setChoosed(){
        return teleopChooser.getSelected();
    }
}
package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DriveConstants.IDs;

public class LimelightConfig extends SubsystemBase{
    
    private final NetworkTable Limelight;

    public static LimelightConfig limelightConfig = new LimelightConfig(IDs.LIMELIGHT);

    public static LimelightConfig getInstance(){
        if(limelightConfig == null){
            return new LimelightConfig(IDs.LIMELIGHT);
        }
        return limelightConfig;
    }
    
    private LimelightConfig(String table){
        Limelight = NetworkTableInstance.getDefault().getTable(table);
    }

    public boolean getHasTarget(){
        return Limelight.getEntry("tv").getDouble(0)==1;
    }
    public double getTagId(){
        return Limelight.getEntry("tid").getDouble(-1);
    }
    public double getTx(){
        return Limelight.getEntry("tx").getDouble(0.0);
    }
    public double getTy(){
        return Limelight.getEntry("ty").getDouble(0.0);
    }
    public double getTa(){
        return Limelight.getEntry("ta").getDouble(0.0);
    }
    public boolean setLedMode(int mode){
        return Limelight.getEntry("ledMode").setNumber(mode);
    }
    public double[] getPoseArena(){
        return Limelight.getEntry("botpose").getDoubleArray(new double[6]);
    }
    public double[] getTagPose(){
        return Limelight.getEntry("targetpose_robotspace").getDoubleArray(new double[6]);
    }
    public double getTxnc(){
        return Limelight.getEntry("txnc").getDouble(0.0);
    }
    public double getTync(){
        return Limelight.getEntry("tync").getDouble(0.0);
    }
    public double getPipelineLatency(){
        return Limelight.getEntry("tl").getDouble(0.0);
    }
}

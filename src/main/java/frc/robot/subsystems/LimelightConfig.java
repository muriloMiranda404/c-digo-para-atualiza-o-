package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IDs;

public class LimelightConfig extends SubsystemBase{
    
    public NetworkTable Limelight(){
        return NetworkTableInstance.getDefault().getTable(IDs.LIMELIGHT);
    }
    public boolean getHasTarget(){
        return Limelight().getEntry("tv").getDouble(0)==1;
    }
    public double getTagId(){
        return Limelight().getEntry("tid").getDouble(-1);
    }
    public double getTx(){
        return Limelight().getEntry("tx").getDouble(0.0);
    }
    public double getTy(){
        return Limelight().getEntry("ty").getDouble(0.0);
    }
    public double getTa(){
        return Limelight().getEntry("ta").getDouble(0.0);
    }
    public boolean setLedMode(int mode){
        return Limelight().getEntry("ledMode").setNumber(mode);
    }
}

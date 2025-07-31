package frc.robot.constants.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Controller extends CommandXboxController {
    
   public Controller(int Id){
    super(Id);
   }

   public double invertedAlliance(int choose, boolean OnMarcha){
    double inverter = 1.0;
    double marcha;

    if(DriverStation.getAlliance().get() == Alliance.Red){
        inverter = -1.0;
    } else{
        inverter = 1.0;
    } 

    if(OnMarcha == true){
      marcha = 0.5 + ((getRightTriggerAxis() - getLeftTriggerAxis()) * 0.5);

      if(marcha <= 0.0) marcha = 0.2;

      if(marcha > 1.0) marcha = 1.0;

    } else{
        marcha = 0.5;
    }

    switch (choose) {
        case 1:
            return getLeftY() * inverter * marcha;
        case 2:
            return getLeftX() * inverter * marcha;
        case 3:
            return getRightX() * marcha;
        case 4:
            return getRightY() * marcha;    
        }
        return choose;
   }

   public void setRumble(double strengt){
    super.getHID().setRumble(RumbleType.kBothRumble, strengt);
    super.getHID().setRumble(RumbleType.kRightRumble, strengt);
    super.getHID().setRumble(RumbleType.kLeftRumble, strengt);
   }
}

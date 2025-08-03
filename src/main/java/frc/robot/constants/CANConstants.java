package frc.robot.constants;

import edu.wpi.first.math.controller.PIDController;

public class CANConstants {
    
  public static final class Swerve{
    //velocidade maxima
    public static final double MAX_SPEED = 7.0;

    //PIDs dos modulos 
    public static final PIDController drivePID = new PIDController(0.01, 0, 0);
    public static final PIDController anglePID = new PIDController(0.01, 0, 0);
  }

}

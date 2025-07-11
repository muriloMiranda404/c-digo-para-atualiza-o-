
package frc.robot;

import edu.wpi.first.math.controller.PIDController;

public final class Constants {

  public static final class Controller{
    //joystick constants 
    public static final int DRIVE_CONTROLLER = 0;
    public static final double DEADBAND = 0.1;

    public static final int INTAKE_CONTROL = 1;
  }

  public static final class Swerve{
    //velocidade maxima
    public static final double MAX_SPEED = 7.0;
  }
  
  public static final class Elevator{
    //motores
    public static final int RIGHT_MOTOR = 14;
    public static final int LEFT_MOTOR = 15;

    //posições
    public static final double L1_POSITION = 0.0;
    public static final double L2_POSITION = 210.0;
    public static final double L3_POSITION = 769.0;
    public static final double L4_POSITION = 1480.0;
    public static final double L2_ALGAE = 624.0;
    public static final double L3_ALGAE = 1107.0;
    
    //PID e derivados
    public static final double TOLERANCE_ELEVATOR = 30.0;
    public static final PIDController ELEVATOR_PID = new PIDController(0.01, 0, 0);

    //entradas digitais
    public static final int UP_SWITCH = 3;
    public static final int DOWN_SWITCH = 2;
    public static final int ENCODER_A = 6;
    public static final int ENCODER_B = 6;
  }

  public static final class Intake{

    //motores
    public static final int INTAKE_MOTOR = 17;
    public static final int ALGAE_MOTOR = 18;

    //PID e derivados
    public static final PIDController INTAKE_PID = new PIDController(0.01, 0, 0);
    public static final double INTAKE_TOLERANCE = 4.0;

    //entradas digitais 
    public static final int INTAKE_ENCODER = 1;
    public static final int ALGAE_SWICTH = 0;

    //zona segura
    public static final double MIN_INTAKE = 55.0;
    public static final double MAX_INTAKE = 230.0;

    //posições
    public static final double ABERTURA_L1 = 68.0;
    public static final double ALGAE_POSITION = 225.0;
    public static final double CORAL_L4 = 92.0;
    public static final double ABERTURA_COMUM = 72.0;
  }

  public static final class Autonomous{
    //auto 
    public static final String AUTO = "New Auto";
  }
  //IDS diversos
  public static final class IDs{
    public static final String LIMELIGHT = "limelight";
    public static final int PIGEON2 = 9;
  }
}

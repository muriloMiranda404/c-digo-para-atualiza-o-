package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.Constants.Elevator;
import frc.robot.constants.Constants.IDs;
import frc.robot.constants.Constants.Intake;
import frc.robot.subsystems.ElevatorSubsytem;
import frc.robot.subsystems.IntakeSubsystem;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  public static boolean trava = false;
  //elevador 
  ElevatorSubsytem elevador = ElevatorSubsytem.getInstance();
  Encoder elev_encoder = elevador.getElevatorEncoder();
  PIDController elev_controller = elevador.getPID();
  double output_elev;

  //intake
  IntakeSubsystem intake = IntakeSubsystem.getInstance();
  DutyCycleEncoder encoder_intake = intake.getIntakeEncoder();
  PIDController intake_controller = intake.getIntakePID();
  double output_intake;

  Pigeon2 pigeon = new Pigeon2(IDs.PIGEON2);
  
  private final RobotContainer m_robotContainer;
  
  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotInit() {

    encoder_intake.setDutyCycleRange(Intake.MIN_ENCODER, Intake.MAX_ENCODER);
    intake_controller.setTolerance(Intake.INTAKE_TOLERANCE);

    elev_controller.setTolerance(Elevator.TOLERANCE_ELEVATOR);
    elev_encoder.setDistancePerPulse(Elevator.PULSE1/Elevator.PULSE2);
    elev_encoder.setReverseDirection(Elevator.INVERTED);
    
    CameraServer.startAutomaticCapture();
    pigeon.reset();
  }
  
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }
  
  @Override
  public void disabledInit() {}
  
  @Override
  public void disabledPeriodic() {}
  
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }
  
  @Override
  public void autonomousPeriodic() {}
  
  @Override
  public void teleopInit() {
    
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    
    output_intake = intake.calculateOutput(intake.getDistance(), Intake.ALGAE_POSITION);
    intake.setPosition(output_intake);
    
    output_elev = elevador.calculateOutput(elevador.getDistance(), Elevator.L1_POSITION);
    elevador.setOutput(output_elev);
    elevador.resetEncoder();
  }

  @Override
  public void teleopPeriodic() {

    if(m_robotContainer.intakeController.getRawButton(1)){
      trava = true;
    }
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}

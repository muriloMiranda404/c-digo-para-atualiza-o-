package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.Constants.Elevator;
import frc.robot.constants.Constants.Intake;
import frc.robot.subsystems.ElevatorSubsytem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  public static boolean trava = false;

  //elevador 
  ElevatorSubsytem elevador = RobotContainer.getElevatorInstance();

  //intake
  IntakeSubsystem intake = RobotContainer.getIntakeInstance();

  //swerve
  SwerveSubsystem swerve = RobotContainer.getSwerveInstance();
  
  private final RobotContainer robotContainer;
    
  public Robot() {
    robotContainer = new RobotContainer();
  }

  @Override
  public void robotInit() {
    intake.configureIntake();

    elevador.configureElevator();

    CameraServer.startAutomaticCapture();
    swerve.resetPigeon();
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
    m_autonomousCommand = robotContainer.getAutonomousCommand();
    
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
    
    intake.setPosition(Intake.ABERTURA_COMUM);
    
    elevador.setOutput(Elevator.L1_POSITION);
  }

  @Override
  public void teleopPeriodic() {

    if(RobotContainer.getContainerIntakeController().getRawButton(1)){
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

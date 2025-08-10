package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.DriveConstants.Intake;
import frc.robot.shuffleboardSettings.ShuffleboardConfig;
import frc.robot.subsystems.ElevatorSubsytem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class Robot extends TimedRobot {

  private Command m_autonomousCommand;
  public static boolean trava = false;

  //elevador 
  private final ElevatorSubsytem elevador;

  //intake
  private final IntakeSubsystem intake;

  //swerve
  private final SwerveSubsystem swerve;
  
  private final ShuffleboardConfig shuffleboardConfig;

  private final RobotContainer robotContainer;
    

  public Robot() {

    robotContainer = new RobotContainer();

    swerve = SwerveSubsystem.getInstance();
    
    elevador = ElevatorSubsytem.getInstance();
    
    intake = IntakeSubsystem.getInstance();

    shuffleboardConfig = ShuffleboardConfig.getInstance();
  }

  @Override
  public void robotInit() {
    
    CameraServer.startAutomaticCapture();
    swerve.resetPigeon();
  
  }
  
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    shuffleboardConfig.getMode();
    shuffleboardConfig.getAlliance();
    shuffleboardConfig.getMathTime();
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
    
    elevador.setSetpoint(shuffleboardConfig.setStartPosition());

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

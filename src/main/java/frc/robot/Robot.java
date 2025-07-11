package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.Elevator;
import frc.robot.Constants.Intake;
import frc.robot.subsystems.ElevatorSubsytem;
import frc.robot.subsystems.IntakeSubsystem;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  public static boolean trava = false;

  IntakeSubsystem intake = IntakeSubsystem.getInstance();
  ElevatorSubsytem subsytem = ElevatorSubsytem.getInstance();
  
  private final RobotContainer m_robotContainer;
  
  public Robot() {
   
    m_robotContainer = new RobotContainer();
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

    intake.setPosition(Intake.ALGAE_POSITION);

    subsytem.setPosition(Elevator.L1_POSITION);
    ElevatorSubsytem.encoder.reset();
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

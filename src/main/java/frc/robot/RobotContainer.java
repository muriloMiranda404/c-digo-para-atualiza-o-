// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.Autonomous;
import frc.robot.Constants.Controller;
import frc.robot.Constants.Elevator;
import frc.robot.Constants.IDs;
import frc.robot.Constants.Intake;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.IntakePosition;
import frc.robot.commands.IntakeSpeed;
import frc.robot.subsystems.ElevatorSubsytem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightConfig;
import frc.robot.subsystems.SwerveSubsystem;
import java.io.File;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {

  private SwerveSubsystem swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  private LimelightConfig limelight = new LimelightConfig();

  public final XboxController controleXbox = new XboxController(Controller.DRIVE_CONTROLLER);
  public final XboxController intakeController = new XboxController(Controller.INTAKE_CONTROL);
  
  private ElevatorSubsytem elevator = new ElevatorSubsytem();
  private IntakeSubsystem intake = new IntakeSubsystem();
  
  private Pigeon2 pigeon = new Pigeon2(IDs.PIGEON2);
  
  public RobotContainer() {
    swerve.setDefaultCommand(swerve.driveCommand(
      () -> MathUtil.applyDeadband(setChoose(1), Controller.DEADBAND),
      () -> MathUtil.applyDeadband(setChoose(2), Controller.DEADBAND),
      () -> MathUtil.applyDeadband(setChoose(3), Controller.DEADBAND)));
    configureBindings();
  }

  private void configureBindings() {
    
    //posições do elevador CORAL
    NamedCommands.registerCommand("L4", new ElevatorCommand(elevator, Elevator.L4_POSITION));
    NamedCommands.registerCommand("L3", new ElevatorCommand(elevator, Elevator.L3_POSITION));
    NamedCommands.registerCommand("L2", new ElevatorCommand(elevator, Elevator.L2_POSITION));
    NamedCommands.registerCommand("L1", new ElevatorCommand(elevator, Elevator.L1_POSITION));
    
    //posições do elevador ALGA
    NamedCommands.registerCommand("ALGAE L2", new ElevatorCommand(elevator, Elevator.L2_ALGAE));
    NamedCommands.registerCommand("ALGAE L3", new ElevatorCommand(elevator, Elevator.L3_ALGAE));

    //posições do intake
    NamedCommands.registerCommand("ALGAE POSITION", new IntakePosition(intake, Intake.ALGAE_POSITION));
    NamedCommands.registerCommand("ABERTURA L1", new IntakePosition(intake, Intake.ABERTURA_L1));
    NamedCommands.registerCommand("POSIÇÃO DE ABERTURA", new IntakePosition(intake, Intake.ABERTURA_COMUM));
    NamedCommands.registerCommand("CORAL L4", new IntakePosition(intake, Intake.ABERTURA_L4));
    NamedCommands.registerCommand("POSIÇÃO MINIMA L1", new IntakePosition(intake, Intake.MIN_INTAKE));
    NamedCommands.registerCommand("GIRAR CORAL", new IntakeSpeed(intake, 0.8));
    NamedCommands.registerCommand("GIRAR CORAL INVERTIDO", new IntakeSpeed(intake, -0.8));
    

    new JoystickButton(intakeController, 1).onTrue(NamedCommands.getCommand("ABERTURA L1")
    .andThen(NamedCommands.getCommand("L1"))
    .andThen(NamedCommands.getCommand("POSIÇÃO MINIMA L1"))
    );

    new JoystickButton(intakeController, 2).onTrue(NamedCommands.getCommand("POSIÇÃO ABERTURA")
    .andThen(NamedCommands.getCommand("L2"))
    );

    new JoystickButton(intakeController, 3).onTrue(NamedCommands.getCommand("POSIÇÃO ABERTURA")
    .andThen(NamedCommands.getCommand("L3"))
    );

    new JoystickButton(intakeController, 4).onTrue(NamedCommands.getCommand("POSIÇÃO ABERTURA")
    .andThen(NamedCommands.getCommand("L4"))
    .andThen(NamedCommands.getCommand("CORAL L4"))
    );
    
    new JoystickButton(intakeController, 5).whileTrue(NamedCommands.getCommand("GIRAR CORAL"));
    new JoystickButton(intakeController, 6).whileTrue(NamedCommands.getCommand("GIRAR CORAL INVERTIDO"));

  }

  public Command getAutonomousCommand() {
    return new PathPlannerAuto(Autonomous.AUTO);
  }
  //marcha
  public double setChoose(int choose){
    int inverter = 1;

    double marcha = 0.5;
    
    if(controleXbox.getRightBumperButton()) marcha = 1.0;
    if(controleXbox.getLeftBumperButton()) marcha = 0.2;
    else marcha = 0.5;
    
    if(DriverStation.getAlliance().get() == Alliance.Red){
      inverter = -1;
    } else {
      inverter = 1;
    }
    switch (choose) {
      case 1:
        return controleXbox.getLeftY() * inverter * marcha;
      case 2:
      return controleXbox.getLeftX() * inverter * marcha;
      case 3:
      return controleXbox.getRightX() * inverter * marcha;  
    }
    return choose;
  }
}
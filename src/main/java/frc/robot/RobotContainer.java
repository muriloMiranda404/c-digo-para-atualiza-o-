package frc.robot;

import frc.robot.commands.AlingToTarget;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.IntakePosition;
import frc.robot.commands.IntakeSpeed;
import frc.robot.commands.ResetPigeon;
import frc.robot.commands.TurnRobot;
import frc.robot.constants.Constants.Autonomous;
import frc.robot.constants.Constants.Controller;
import frc.robot.constants.Constants.Elevator;
import frc.robot.constants.Constants.IDs;
import frc.robot.constants.Constants.Intake;
import frc.robot.subsystems.ElevatorSubsytem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightConfig;
import frc.robot.subsystems.SwerveModulesSubsystem;
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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

public class RobotContainer {

  private static final SwerveSubsystem swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  private static final LimelightConfig limelight = LimelightConfig.getInstance();

  public final CommandXboxController driveController = new CommandXboxController(Controller.DRIVE_CONTROLLER);
  public final XboxController intakeController = new XboxController(Controller.INTAKE_CONTROL_ID);
  
  public static final ElevatorSubsytem elevatorSubsytem = new ElevatorSubsytem();
  public static final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  public static final SwerveModulesSubsystem swerveModules = new SwerveModulesSubsystem();
  
  private static final Pigeon2 pigeon = new Pigeon2(IDs.PIGEON2);
  
  public RobotContainer() {
    swerve.setDefaultCommand(swerve.driveCommand(
      () -> MathUtil.applyDeadband(setChoose(1), Controller.DEADBAND),
      () -> MathUtil.applyDeadband(setChoose(2), Controller.DEADBAND),
      () -> MathUtil.applyDeadband(setChoose(3), Controller.DEADBAND)));

    configureDriveBindings();
    configureMechanismBindings();
  }

  private void configureDriveBindings() {   
    
    //quando iniciar, o robo vira para 0 graus;
    driveController.start().onTrue(new TurnRobot(pigeon, swerve, 0.0));

    //limelight
    new POVButton(driveController.getHID(), 270).whileTrue(new AlingToTarget(limelight, swerve, true));

    //reset pigeon
    new JoystickButton(driveController.getHID(), 10).onTrue(new ResetPigeon(pigeon, swerve));

    //turn robot
    new JoystickButton(driveController.getHID(), 1).onTrue(new TurnRobot(pigeon, swerve, 45));
    new JoystickButton(driveController.getHID(), 2).onTrue(new TurnRobot(pigeon, swerve, -45));
  }

  private void configureMechanismBindings(){

     //posições do elevador CORAL
     NamedCommands.registerCommand("L4", new ElevatorCommand(elevatorSubsytem, Elevator.L4_POSITION));
     NamedCommands.registerCommand("L3", new ElevatorCommand(elevatorSubsytem, Elevator.L3_POSITION));
     NamedCommands.registerCommand("L2", new ElevatorCommand(elevatorSubsytem, Elevator.L2_POSITION));
     NamedCommands.registerCommand("L1", new ElevatorCommand(elevatorSubsytem, Elevator.L1_POSITION));
     
     //posições do elevador ALGA
     NamedCommands.registerCommand("ALGAE L2", new ElevatorCommand(elevatorSubsytem, Elevator.L2_ALGAE));
     NamedCommands.registerCommand("ALGAE L3", new ElevatorCommand(elevatorSubsytem, Elevator.L3_ALGAE));
 
     //posições do intake
     NamedCommands.registerCommand("ALGAE POSITION", new IntakePosition(intakeSubsystem, Intake.ALGAE_POSITION));
     NamedCommands.registerCommand("ABERTURA L1", new IntakePosition(intakeSubsystem, Intake.ABERTURA_L1));
     NamedCommands.registerCommand("POSIÇÃO ABERTURA", new IntakePosition(intakeSubsystem, Intake.ABERTURA_COMUM));
     NamedCommands.registerCommand("CORAL L4", new IntakePosition(intakeSubsystem, Intake.CORAL_L4));
     NamedCommands.registerCommand("POSIÇÃO MINIMA L1", new IntakePosition(intakeSubsystem, Intake.MIN_INTAKE));
     NamedCommands.registerCommand("GIRAR CORAL", new IntakeSpeed(intakeSubsystem, 0.8));
     NamedCommands.registerCommand("GIRAR CORAL INVERTIDO", new IntakeSpeed(intakeSubsystem, -0.8));
     
   ///////////////////////////////////////////// COMANDOS TELEOPERADOS////////////////////////////////////////////////////////////
 
     //L1
     new JoystickButton(intakeController, 1).onTrue(NamedCommands.getCommand("ABERTURA L1")
     .andThen(NamedCommands.getCommand("L1"))
     .andThen(NamedCommands.getCommand("POSIÇÃO MINIMA L1"))
     );
 
     //L2
     new JoystickButton(intakeController, 2).onTrue(NamedCommands.getCommand("POSIÇÃO ABERTURA")
     .andThen(NamedCommands.getCommand("L2"))
     );
 
     //L3
     new JoystickButton(intakeController, 3).onTrue(NamedCommands.getCommand("POSIÇÃO ABERTURA")
     .andThen(NamedCommands.getCommand("L3"))
     );
 
     //L4
     new JoystickButton(intakeController, 4).onTrue(NamedCommands.getCommand("POSIÇÃO ABERTURA")
     .andThen(NamedCommands.getCommand("L4"))
     .andThen(NamedCommands.getCommand("CORAL L4"))
     );

         //empurrar e puxar o coral
    new JoystickButton(intakeController, 5).whileTrue(NamedCommands.getCommand("GIRAR CORAL"));
    new JoystickButton(intakeController, 6).whileTrue(NamedCommands.getCommand("GIRAR CORAL INVERTIDO"));


      //////////////////////////////////////INICIO DOS COMANDOS AUTOMATICOS////////////////////////////////////////////////////////

    //L1
    NamedCommands.registerCommand("L1 FULL COMMAND", 
    NamedCommands.getCommand("ABERTURA L1")
    .andThen(NamedCommands.getCommand("L1"))
    .andThen(NamedCommands.getCommand("POSIÇÃO MINIMA L1"))
    );
  //L2

  NamedCommands.registerCommand("L2 FULL COMMAND", 
  NamedCommands.getCommand("POSIÇÃO ABERTURA")
  .andThen(NamedCommands.getCommand("L2"))
  );  

  //L3
  NamedCommands.registerCommand("L3 FULL COMMAND", 
  NamedCommands.getCommand("POSIÇÃO ABERTURA")
  .andThen(NamedCommands.getCommand("L3"))
  );

  //L4  
  NamedCommands.registerCommand("L4 FULL COMMAND", 
  NamedCommands.getCommand("POSIÇÃO ABERTURA")
  .andThen(NamedCommands.getCommand("L4"))
  .andThen(NamedCommands.getCommand("CORAL L4"))
  );


  ///////////////////////////////////////// FIM DOS COMANDOS AUTOMATICOS ///////////////////////////////////////////////
    
  }
  public Command getAutonomousCommand() {
    return new PathPlannerAuto(Autonomous.AUTO);
  }

  //marcha
  public double setChoose(int choose){

    int inverter = DriverStation.getAlliance().get() == Alliance.Red ? -1 : 1;
    double marcha = 0.5;
    
    if(driveController.getHID().getRightBumperButton()) marcha = 1.0;
    if(driveController.getHID().getLeftBumperButton()) marcha = 0.2;
    else marcha = 0.5;
 
    switch (choose) {
      case 1:
        return driveController.getLeftY() * inverter * marcha;
      case 2:
      return driveController.getLeftX() * inverter * marcha;
      case 3:
      return driveController.getRightX() * marcha;  
      case 4:
      return driveController.getRightY() * marcha;
    }
    
    return choose;
  }

  public static ElevatorSubsytem getElevatorInstance(){
    if(elevatorSubsytem == null){
        return new ElevatorSubsytem();
    }
    return elevatorSubsytem;
}

public static IntakeSubsystem getIntakeInstance(){
  if(intakeSubsystem == null){
    return new IntakeSubsystem();
  }
  return intakeSubsystem;
}

public static SwerveModulesSubsystem getModulesInstance(){
  if(swerveModules == null){
    return new SwerveModulesSubsystem();
  }
  return swerveModules;
}

public static SwerveSubsystem getSwerveInstance(){
  if(swerve == null){
    return new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  }
  return swerve;
}
}
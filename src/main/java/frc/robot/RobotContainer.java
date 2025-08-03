package frc.robot;

import frc.robot.commands.AlingToTarget;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.IntakePosition;
import frc.robot.commands.IntakeSpeed;
import frc.robot.commands.ResetPigeon;
import frc.robot.commands.TurnRobot;
import frc.robot.constants.DriveConstants.Autonomous;
import frc.robot.constants.DriveConstants.Joystick;
import frc.robot.constants.utils.Controller;
import frc.robot.shuffleboardSettings.ShuffleboardConfig;
import frc.robot.constants.DriveConstants.Elevator;
import frc.robot.constants.DriveConstants.IDs;
import frc.robot.constants.DriveConstants.Intake;
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
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

public class RobotContainer {

  private static final SwerveSubsystem swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  private static final LimelightConfig limelight = new LimelightConfig();

  public static final Controller driveController = new Controller(Joystick.DRIVE_CONTROLLER);
  public static final XboxController intakeController = new XboxController(Joystick.INTAKE_CONTROL_ID);
  
  public static final ElevatorSubsytem elevatorSubsytem = new ElevatorSubsytem();
  public static final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  public static final SwerveModulesSubsystem swerveModules = new SwerveModulesSubsystem();

  private static final ShuffleboardConfig shuffleboardConfig = new ShuffleboardConfig();
  
  private static final Pigeon2 pigeon = new Pigeon2(IDs.PIGEON2);

  private static String teleop = shuffleboardConfig.setChoosed();
  
  public RobotContainer() {
    if(teleop == "Normal_teleop"){

    swerve.setDefaultCommand(swerve.driveCommand(
      
      () -> MathUtil.applyDeadband(driveController.invertedAlliance(1, true), Joystick.DEADBAND),
      () -> MathUtil.applyDeadband(driveController.invertedAlliance(2, true), Joystick.DEADBAND),
      () -> MathUtil.applyDeadband(driveController.invertedAlliance(3, true), Joystick.DEADBAND)));
      
      System.out.println("o teleop escolhido foi o principal");
    configureDriveBindings();
    configureMechanismBindings();

} else if(teleop == "Second_teleop"){

  swerve.setDefaultCommand(swerve.driveCommand(

      () -> MathUtil.applyDeadband(driveController.invertedAlliance(1, true), Joystick.DEADBAND),
      () -> MathUtil.applyDeadband(driveController.invertedAlliance(2, true), Joystick.DEADBAND),
      () -> MathUtil.applyDeadband(driveController.invertedAlliance(3, true), Joystick.DEADBAND),
      () -> MathUtil.applyDeadband(driveController.invertedAlliance(4, false), Joystick.DEADBAND)));

      System.out.println("o teleop escolhido foi o secundario");
      configureDriveBindings();
      configureMechanismBindings();
    }
  }
  
  private void configureDriveBindings() {   
    
    NamedCommands.registerCommand("ALINHAMENTO", new AlingToTarget(limelight, swerve, true));
    NamedCommands.registerCommand("TURN TO 0", new TurnRobot(pigeon, swerve, 0.0));
    NamedCommands.registerCommand("RESET PIGEON", new ResetPigeon(pigeon, swerve)); 
    NamedCommands.registerCommand("TURN TO 45", new TurnRobot(pigeon, swerve, 45.0));
    NamedCommands.registerCommand("TURN TO -45", new TurnRobot(pigeon, swerve, -45.0));

    //quando iniciar, o robo vira para 0 graus;
    driveController.start().onTrue(NamedCommands.getCommand("TURN TO 0"));

    //limelight
    new POVButton(driveController.getHID(), 270).whileTrue(NamedCommands.getCommand("ALINHAMENTO"));

    //reset pigeon
    new JoystickButton(driveController.getHID(), 10).onTrue(NamedCommands.getCommand("RESET PIGEON"));

    //turn robot
    new JoystickButton(driveController.getHID(), 1).onTrue(NamedCommands.getCommand("TURN TO 45"));
    new JoystickButton(driveController.getHID(), 2).onTrue(NamedCommands.getCommand("TURN TO -45"));
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
     NamedCommands.registerCommand("GIRAR CORAL", new IntakeSpeed(intakeSubsystem, 0.3));
     NamedCommands.registerCommand("GIRAR CORAL INVERTIDO", new IntakeSpeed(intakeSubsystem, -0.3));
     
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

public static LimelightConfig getLimelightInstance(){
  if(limelight == null){
    return new LimelightConfig();
  }
  return limelight;
}

public static XboxController getContainerIntakeController(){
  if(intakeController == null){
    return new CommandXboxController(Joystick.INTAKE_CONTROL_ID).getHID();
  }
  return intakeController;
}

public static XboxController getContainerDriverController(){
  if(driveController == null){
    return new CommandXboxController(Joystick.DRIVE_CONTROLLER).getHID();
  }
  return driveController.getHID();
}

public static ShuffleboardConfig getShuffleboardInstance(){
  if(shuffleboardConfig == null){
    return new ShuffleboardConfig();
  }
  return shuffleboardConfig;
}
}
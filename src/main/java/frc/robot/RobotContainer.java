// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Autos;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  // autonomous routines/commands
  private final Command slowAuto = Autos.slowAuto(driveSubsystem);
  private final Command midAuto = Autos.autoMid(driveSubsystem, elevatorSubsystem, intakeSubsystem);
  private final Command rightAuto = Autos.rightAuto(driveSubsystem, elevatorSubsystem, intakeSubsystem);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandPS4Controller ps4Controller = new CommandPS4Controller((OperatorConstants.kPS4Controller));
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    autoChooser.setDefaultOption("Starting pos MIDDLE", midAuto);
    autoChooser.addOption("Slow Auto 0.3 for 2 sec", slowAuto);
    // autoChooser.setDefaultOption("Fast Auto 0.5 for 2 sec", fastAuto);
    autoChooser.addOption("Starting pos LEFT", rightAuto);
    autoChooser.addOption("Starting pos RIGHT", Autos.autoRight(driveSubsystem, elevatorSubsystem));
    autoChooser.addOption("TEST auto sequencing", Autos.autoTest(driveSubsystem,  elevatorSubsystem,intakeSubsystem));
    SmartDashboard.putData("chooser boozer", autoChooser);
    SmartDashboard.putNumber("Battery Voltage", RobotController.getBatteryVoltage());


  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    driveSubsystem.setDefaultCommand(
      driveSubsystem.driveArcadeCommand(
        () -> m_driverController.getLeftY(), () -> -m_driverController.getRightX()));
    
    driveSubsystem.setDefaultCommand(
      driveSubsystem.driveArcadeCommand(
        () -> ps4Controller.getLeftY(), () -> -ps4Controller.getRightX()));
    
    ps4Controller.triangle().onTrue(
      elevatorSubsystem.goToElevatorL2()
    );
    ps4Controller.cross().onTrue(
      elevatorSubsystem.goToElevatorStow()
    );
    ps4Controller.square().onTrue(
      elevatorSubsystem.setElevatorPower(-0.1)
    );
    ps4Controller.circle().onTrue(
      elevatorSubsystem.setElevatorPower(0.1)
    );
    ps4Controller.L1().toggleOnTrue(
      intakeSubsystem.intakeStowCoral()
    );
    ps4Controller.R1().toggleOnTrue(
      intakeSubsystem.scoreL2()
    );

    ps4Controller.R1().toggleOnFalse(
      intakeSubsystem.intakeStop(elevatorSubsystem)
    );

    m_driverController.y().onTrue(
      intakeSubsystem.intakeStowCoral()
    );

    m_driverController.a().onTrue(
      elevatorSubsystem.goToElevatorStow()
    );

    m_driverController.b().onTrue(
      elevatorSubsystem.goToElevatorL2()
    );

    m_driverController.povUp().onTrue(
      elevatorSubsystem.setElevatorPower(-0.1)
    );
    m_driverController.povDown().onTrue(
      elevatorSubsystem.setElevatorPower(0.1)
    );
    m_driverController.povRight().onTrue(
      elevatorSubsystem.stopElevator()
    );

    // m_driverController.leftBumper().toggleOnTrue(
    //   intakeSubsystem.scoreL1(elevatorSubsystem)
    // );
    m_driverController.leftBumper().toggleOnTrue(
      intakeSubsystem.score(elevatorSubsystem)
    );

    m_driverController.leftBumper().toggleOnFalse(
      intakeSubsystem.intakeStop(elevatorSubsystem)
    );

    m_driverController.rightBumper().toggleOnTrue(
      intakeSubsystem.score(elevatorSubsystem)
    );

    // COMMENT OUT BELOW when laserCAN is confirmed to work
    // Or, leave for manual interruption (just have driver hold it for longer)
    m_driverController.rightBumper().toggleOnFalse(
      intakeSubsystem.intakeStop(elevatorSubsystem)
    );

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}
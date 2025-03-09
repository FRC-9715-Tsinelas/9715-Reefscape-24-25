// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;


public final class Autos {
  /** Example static factory for an autonomous command. */
  public static final Command midAuto(DriveSubsystem d, ElevatorSubsystem e, IntakeSubsystem i) {
    // return d.driveArcadeCommand(() -> -0.3, () -> 0.0).withTimeout(2.0)
    //   .andThen(e.goToElevatorL2())
    //   .andThen(i.scoreL2()).withTimeout(1);

    return Commands.sequence(
      d.driveArcadeCommand(() -> -0.3, () -> 0.0).withTimeout(2.0),
      e.goToElevatorL2(),
      i.scoreL2().withTimeout(1)
    );

      // .andThen(e.goToElevatorStow())
      // .andThen(i.intakeStop(e));
  }
  public static final Command slowAuto(DriveSubsystem d) {
    return d.driveArcadeCommand(() -> -0.3, () -> 0.0).withTimeout(1.5);
      // .andThen(d.driveArcadeCommand(() -> 0.0, () -> -0.2).withTimeout(1.5));

  }
  public static final Command rightAuto(DriveSubsystem d, ElevatorSubsystem e, IntakeSubsystem i){
    return d.driveArcadeCommand(() -> -0.3, () -> 0.0).withTimeout(1.5);
      // .andThen(d.driveArcadeCommand(() -> 0.0, () -> 0.3)).withTimeout(1)
      // .andThen(d.driveArcadeCommand(() -> 0.5, () -> 0.0)).withTimeout(2.5)
      // .alongWith(e.goToElevatorL2())
      // .andThen(i.scoreL2()).withTimeout(1)
      // .andThen(e.goToElevatorStow())
      // .alongWith(i.intakeStop(e)); // !!!!!!!


  }


  public static final Command autoMid(DriveSubsystem d, ElevatorSubsystem e, IntakeSubsystem i) {
    // return Commands.runOnce(() -> {
    //   // TODO: tune distances
    //   d.arcadeDrive(0.5, 0.0, 6.0);
    //   // e.elevatorL2();
    //   i.L2();
    //   Timer.delay(1);
    //   i.stop();
    // });
    return d.arcadeDrive(0.5, 0.0, 6.0);
  }
  public static final Command autoTest(DriveSubsystem d, ElevatorSubsystem e, IntakeSubsystem i) {
    return Commands.run(() -> {
      d.driveArcadeCommand(() -> 0.0, () -> 0.1).withTimeout(9);
      e.elevatorL1();
      i.L2();
    });

  }
  public static final Command autoRight(DriveSubsystem d, ElevatorSubsystem e) {
    return Commands.run(() -> {
      // TODO: tune distances and angles
      // target: 130 INCHES
      d.driveArcadeCommand(() -> 0.5, () -> 0.0).withTimeout(6.0);
      // target: 60 deg LEFT
      d.driveArcadeCommand(() -> 0.0, () -> -0.3).withTimeout(3.0);
      // target: 7 INCHES
      d.driveArcadeCommand(() -> 0.5, () -> 0.0).withTimeout(3.0);
      e.elevatorL2();
    });
  }
  public static final Command autoLeft(DriveSubsystem d, ElevatorSubsystem e) {
    return Commands.run(() -> {
      // TODO: tune distances and angles
      // target: 130 INCHES
      d.driveArcadeCommand(() -> 0.5, () -> 0.0).withTimeout(6.0).execute();
      // target: 60 deg RIGHT
      d.driveArcadeCommand(() -> 0.0, () -> 0.3).withTimeout(3.0).execute();
      // target: 7 INCHES
      d.driveArcadeCommand(() -> 0.5, () -> 0.0).withTimeout(3.0).execute();
      e.elevatorL2();
    });
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}

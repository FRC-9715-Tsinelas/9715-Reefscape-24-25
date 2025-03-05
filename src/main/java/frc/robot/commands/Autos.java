// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;


public final class Autos {
  /** Example static factory for an autonomous command. */
  public static final Command fastAuto(DriveSubsystem driveSubsystem) {
    return driveSubsystem.driveArcadeCommand(() -> 0.5, () -> 0.0).withTimeout(2.0);
  }
  public static final Command slowAuto(DriveSubsystem driveSubsystem) {
    return driveSubsystem.driveArcadeCommand(() -> 0.3, () -> 0.0).withTimeout(2.0);
  }

  public static final Command autoMid(DriveSubsystem d, ElevatorSubsystem e) {
    return Commands.run(() -> {
      // TODO: tune distances
      d.driveArcadeCommand(() -> 0.5, () -> 0.0).withTimeout(9.0).execute();
      e.elevatorL2();
    });
  }
  public static final Command autoRight(DriveSubsystem d, ElevatorSubsystem e) {
    return Commands.run(() -> {
      // TODO: tune distances and angles
      d.driveArcadeCommand(() -> 0.5, () -> 0.0).withTimeout(6.0).execute();
      d.driveArcadeCommand(() -> 0.0, () -> -0.3).withTimeout(3.0).execute();
      d.driveArcadeCommand(() -> 0.5, () -> 0.0).withTimeout(3.0).execute();
      e.elevatorL2();
    });
  }
  public static final Command autoLeft(DriveSubsystem d, ElevatorSubsystem e) {
    return Commands.run(() -> {
      // TODO: tune distances and angles
      d.driveArcadeCommand(() -> 0.5, () -> 0.0).withTimeout(6.0).execute();
      d.driveArcadeCommand(() -> 0.0, () -> 0.3).withTimeout(3.0).execute();
      d.driveArcadeCommand(() -> 0.5, () -> 0.0).withTimeout(3.0).execute();
      e.elevatorL2();
    });
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}

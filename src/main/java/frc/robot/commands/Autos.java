// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;


public final class Autos {
  /** Example static factory for an autonomous command. */
  public static final Command fastAuto(DriveSubsystem driveSubsystem) {
    return driveSubsystem.driveArcadeCommand(driveSubsystem, () -> 0.5, () -> 0.0).withTimeout(2.0);
  }
  public static final Command slowAuto(DriveSubsystem driveSubsystem) {
    return driveSubsystem.driveArcadeCommand(driveSubsystem, () -> 0.3, () -> 0.0).withTimeout(2.0);
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}

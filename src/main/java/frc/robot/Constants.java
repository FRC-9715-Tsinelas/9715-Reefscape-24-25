// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosit'y.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 1;
    public static final int kPS4Controller = 0;
  }
  public static class DriveConstants {
    public static final int leftFrontMotor = 4;
    public static final int leftBackMotor = 5;
    public static final int rightFrontMotor = 2;
    public static final int rightBackMotor = 3;
    public static final int driveCurrentLimit = 50;
    public static final int driveVoltageCompensation = 14;
  }
  public static class ElevatorConstants {
    public static final int leftMotor = 15;
    public static final int rightMotor = 11;
    public static final double kP = 0.2;
    public static final double kD = 0.0;
    public static final double kI = 0.0;
    public static final double kIZone = 5.0;
    public static final double kG = 0.5;
    public static final double kMaxVelocity = 600; // was 170 -> 300
    public static final double kMaxAcceleration = 500; // was 500
    public static final int kMaxCurrent = 38; // SAME as DRivetraion

    // NOTE: L1 may be lower than height to recieve coral
    public static final double kStowHeight = 0.0;
    public static final double kL1Height = 55;
    public static final double kL2Height = 140.0;
  }
  public static class IntakeConstants {
    public static final int intakeleftMotor = 10;
    public static final int intakerightMotor = 12;
    public static final int intakeVoltageCompensation = 12;
    // NOTE: LaserCAN will have a DIFFERENT CAN SETUP!!
    public static final int laserCan = 30;
    public static final int kMaxCurrent = 30;
    public static final int coralPresenceMm = 100;
    public static final double intakeLL1Speed = 0.3;
    public static final double intakeLR1Speed = 0.7;
    public static final double intakeL2Speed = 0.6;
  }
}

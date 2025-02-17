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
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  
  public static class DriveConstants {
    public static final int leftFrontMotor = 3;
    public static final int leftBackMotor = 2;
    public static final int rightFrontMotor = 4;
    public static final int rightBackMotor = 1;
  }

  public static class ElevatorConstants {
    public static final int leftMotor = 10;
    public static final int rightMotor = 11;
    public static final int topLimSwitch = 12;
    // public static final int bottomLimSwitch = 7;
    public static final int encoderA = 13;
    public static final int encoderB = 14;
    // Coefficient to convert motor revolutions to the elevator height (inches).
    // DIAMETER from CAD * [Rotation/360deg] * pi
    public static final double revsToHeightIn = 1.887658 * Math.PI;
    // TODO: change PID values
    public static double kp = 1.0; public static double ki = 1.0; public static double kd = 1.0;
    public static double userPosChangeRate = 0.1;
  }
}

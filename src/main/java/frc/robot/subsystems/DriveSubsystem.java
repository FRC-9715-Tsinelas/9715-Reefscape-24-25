// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

import com.revrobotics.spark.SparkLowLevel.MotorType;

// import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkMax;


public class DriveSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final SparkMax l1 = new SparkMax(DriveConstants.leftFrontMotor, MotorType.kBrushless);
  private final SparkMax l2 = new SparkMax(DriveConstants.leftBackMotor, MotorType.kBrushless);
  private final SparkMax r1 = new SparkMax(DriveConstants.rightFrontMotor, MotorType.kBrushless);
  private final SparkMax r2 = new SparkMax(DriveConstants.rightBackMotor, MotorType.kBrushless);
  DifferentialDrive drive = new DifferentialDrive(
    (double output) -> {
        l1.set(output);
        l2.set(output);
    },
    (double output) -> {
        r1.set(output);
        r2.set(output);
    });
  public DriveSubsystem() {

  }
  

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command driveArcadeCommand(DriveSubsystem driveSubsystem, Double xSpeed, Double zRotation) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return Commands.run(
        () -> drive.arcadeDrive(xSpeed, zRotation), driveSubsystem);
  }
  

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

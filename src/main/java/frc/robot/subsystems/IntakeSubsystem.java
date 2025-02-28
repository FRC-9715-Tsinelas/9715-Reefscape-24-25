// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private final SparkMax mleftMotor = new SparkMax(IntakeConstants.intakeLeftMotor, MotorType.kBrushless);
  private final SparkMax mRightMotor = new SparkMax(IntakeConstants.intakeRightMotor, MotorType.kBrushless);
  public IntakeSubsystem() {
    

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

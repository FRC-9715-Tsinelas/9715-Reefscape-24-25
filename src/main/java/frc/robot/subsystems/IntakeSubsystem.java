// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;


public class IntakeSubsystem extends SubsystemBase {
  private final SparkMax mLeftMotor = new SparkMax(IntakeConstants.intakeleftMotor, MotorType.kBrushless);
  private final SparkMax mRightMotor = new SparkMax(IntakeConstants.intakerightMotor, MotorType.kBrushless);
  /** Creates a new IntakeSubsystem. */

  public IntakeSubsystem() {
    SparkMaxConfig intakeConfig = new SparkMaxConfig();
    intakeConfig.idleMode(IdleMode.kBrake)
                .smartCurrentLimit(IntakeConstants.kMaxCurrent);
    mLeftMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    intakeConfig.inverted(true);
    mRightMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  void L1(){
    stop();
  }
  void L2(){
    mLeftMotor.set(IntakeConstants.intakeL2Speed);
    mRightMotor.set(IntakeConstants.intakeL2Speed);
  }
  void stop(){
    mLeftMotor.set(0);
    mRightMotor.set(0);
    System.out.println("Intake stopped!");
  }

  public Command scoreL1(){
    return run(() -> L1());
  }
  public Command intakeStop(){
    return run(() -> stop());
  }
  public Command scoreL2(){
    return run(() -> L2());
  }
  

  @Override
  public void periodic() {
    // sus
  }
}

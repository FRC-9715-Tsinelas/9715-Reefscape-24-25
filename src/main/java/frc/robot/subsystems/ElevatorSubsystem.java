// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;

public class ElevatorSubsystem extends SubsystemBase {
  
  private TrapezoidProfile mProfile;
  private TrapezoidProfile.State mCurState = new TrapezoidProfile.State();
  private TrapezoidProfile.State mGoalState = new TrapezoidProfile.State();
  private final SparkMax mLeftMotor = new SparkMax(ElevatorConstants.leftMotor, MotorType.kBrushless);
  private final SparkMax mRightMotor = new SparkMax(ElevatorConstants.rightMotor, MotorType.kBrushless);





  public ElevatorSubsystem() {
    SparkMaxConfig elevatorConfig = new SparkMaxConfig();
    elevatorConfig.closedLoop.pid(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD).iZone(ElevatorConstants.kIZone);
    elevatorConfig.smartCurrentLimit(ElevatorConstants.kMaxCurrent)
                  .idleMode(IdleMode.kBrake)
                  .limitSwitch.reverseLimitSwitchEnabled(true);
    // LEFT ELEVATOR MOTOR
    





    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

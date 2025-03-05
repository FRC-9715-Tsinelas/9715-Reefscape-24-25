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
import com.revrobotics.spark.config.SparkBaseConfig;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.ConfigurationFailedException;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakeConstants;


public class IntakeSubsystem extends SubsystemBase {
  private final SparkMax mLeftMotor = new SparkMax(IntakeConstants.intakeleftMotor, MotorType.kBrushless);
  private final SparkMax mRightMotor = new SparkMax(IntakeConstants.intakerightMotor, MotorType.kBrushless);
  private final LaserCan lc = new LaserCan(IntakeConstants.laserCan);

  final int L1 = 1;
  final int L2 = 2;
  final int EMPTY = 0;
  final int STOWING = 3;
  final int STOWED = 4;
  int status = EMPTY;
  /** Creates a new IntakeSubsystem. */

  public IntakeSubsystem() {
    SparkMaxConfig intakeConfig = new SparkMaxConfig();
    intakeConfig.idleMode(IdleMode.kBrake)
                .smartCurrentLimit(IntakeConstants.kMaxCurrent);
    mLeftMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    intakeConfig.inverted(true);
    mRightMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    try {
      lc.setRangingMode(LaserCan.RangingMode.SHORT);
      lc.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
      lc.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
    } catch (ConfigurationFailedException e) {
      System.out.println("Configuration failed! " + e);
    }
  }
  // L1 and L2 SEND OUT CORAL.
  // periodic() checks for CORAL INPUT.
  void L1(){
    if (status == EMPTY) return;
    if (status != STOWED) {
      stop();
    }
    else {
      mLeftMotor.set(IntakeConstants.intakeLL1Speed);
      mRightMotor.set(IntakeConstants.intakeLR1Speed);
    }
    
  }
  void L2(){
    if (status == EMPTY) return;
    if (status != STOWED){
      stop();
    }
    else{
      mLeftMotor.set(IntakeConstants.intakeL2Speed);
      mRightMotor.set(IntakeConstants.intakeL2Speed);
    }
  }
  void stop(){
    mLeftMotor.set(0);
    mRightMotor.set(0);
    status = EMPTY;
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
    LaserCan.Measurement m;
    if ((m = lc.getMeasurement()) != null
        && m.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT
        && m.distance_mm <= IntakeConstants.coralDistanceThresholdMm
        && status == EMPTY )
    {
      mLeftMotor.set(IntakeConstants.intakeStowCoralSpeed);
      mRightMotor.set(IntakeConstants.intakeStowCoralSpeed);
      status = STOWED;
    }
  }
}

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

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;


public class IntakeSubsystem extends SubsystemBase {
  private final SparkMax mLeftMotor = new SparkMax(IntakeConstants.intakeleftMotor, MotorType.kBrushless);
  private final SparkMax mRightMotor = new SparkMax(IntakeConstants.intakerightMotor, MotorType.kBrushless);
  private final LaserCan mLaserCAN = new LaserCan(IntakeConstants.kLaserId);
  private PeriodicIO mPeriodicIO;
  /** Creates a new IntakeSubsystem. */

  public IntakeSubsystem() {
    mPeriodicIO = new PeriodicIO();
    SparkMaxConfig intakeConfig = new SparkMaxConfig();
    intakeConfig.idleMode(IdleMode.kBrake)
                .smartCurrentLimit(IntakeConstants.kMaxCurrent);
    mLeftMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    intakeConfig.inverted(true);
    mRightMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    try {
      mLaserCAN.setRangingMode(LaserCan.RangingMode.SHORT);
      mLaserCAN.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
      mLaserCAN.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
    } catch (ConfigurationFailedException e) {
      System.out.println("Configuration failed! " + e);
    }
  }
  private static class PeriodicIO {
    int index_debounce = 0;

    LaserCan.Measurement measurement;
  }
  
  @Override
  public void periodic() {
    mPeriodicIO.measurement = mLaserCAN.getMeasurement();
    if ((mPeriodicIO.measurement) != null
        && mPeriodicIO.measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT
        && mPeriodicIO.measurement.distance_mm <= IntakeConstants.coralDistanceThresholdMm )
    {
      mPeriodicIO.index_debounce++;
      
      mLeftMotor.set(IntakeConstants.intakeStowCoralSpeed);
      mRightMotor.set(IntakeConstants.intakeStowCoralSpeed);
  }





  void L1(){
    if (Math.abs(mLeftMotor.getAppliedOutput()) > 0.1){
      System.out.println(mLeftMotor.getAppliedOutput());
      stop();
    }
    else {
      mLeftMotor.set(IntakeConstants.intakeLL1Speed);
      mRightMotor.set(IntakeConstants.intakeLR1Speed);
    }
    
  }
  void L2(){
    if (Math.abs(mLeftMotor.getAppliedOutput()) > 0.1){
      System.out.println(mLeftMotor.getAppliedOutput());
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
}

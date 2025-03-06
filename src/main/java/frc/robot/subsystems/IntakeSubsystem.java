// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import java.util.function.BooleanSupplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;

public class IntakeSubsystem extends SubsystemBase {
  private final SparkMax mLeftMotor = new SparkMax(IntakeConstants.intakeleftMotor, MotorType.kBrushless);
  private final SparkMax mRightMotor = new SparkMax(IntakeConstants.intakerightMotor, MotorType.kBrushless);
  private final LaserCan lc = new LaserCan(IntakeConstants.laserCan);
  /** Creates a new IntakeSubsystem. */
  private PeriodicIO mPeriodicIO;

  public IntakeSubsystem() {
    mPeriodicIO = new PeriodicIO();
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
  private static class PeriodicIO {
    int index_debounce = 0;
    LaserCan.Measurement measurement;
    boolean bHasCoral = false;
    boolean lastbHasCoral = false;
    boolean tempbHasCoral = true;
    boolean intakeisRunning = false;
  }

  void L1(){
    mLeftMotor.set(IntakeConstants.intakeLL1Speed);
    mRightMotor.set(IntakeConstants.intakeLR1Speed);
    mPeriodicIO.intakeisRunning = true;
    // L2();
  }

  void L2(){
    // System.out.print(mPeriodicIO.intakeisRunning);
    // if (mPeriodicIO.intakeisRunning){
    //   stop();
    // }
    // else {
    mLeftMotor.set(IntakeConstants.intakeL2Speed);
    mRightMotor.set(IntakeConstants.intakeL2Speed);
    mPeriodicIO.intakeisRunning = true;
    // }
  }
  void stop(){
    mLeftMotor.set(0);
    mRightMotor.set(0);
    mPeriodicIO.intakeisRunning = false;
    System.out.println("Intake stopped!");
  }
  void stowCoral(){
    mLeftMotor.set(0.4);
    mRightMotor.set(0.4);
  }

  public Command scoreL1(ElevatorSubsystem e){
    return runOnce(() -> {
      System.out.println(e.elestate);
      L1();
    });
  }
  public Command intakeStop(){
    return runOnce(() -> stop());
  }
  public Command score(ElevatorSubsystem e) {
    return runOnce(() -> {
      System.out.println(e.elestate);
      switch (e.elestate) {
        case 0: // STOWED
          break;
        case 1:
          L1();
          System.out.println("Used l1 intake speed!");
          break;
        case 2:
          L2();
          System.out.println("Used l2 intake speed!");
          e.elevatorStow();
          System.out.println("Elevator stowed");
          break;
        default:
          System.out.println("Invalid elevator state!");
      }
    });
  }

  public Command scoreL2(ElevatorSubsystem e){
    return runOnce(() -> {
      System.out.println(e.elestate);
      // if (e.elestate == 1){
      //   mLeftMotor.set(IntakeConstants.intakeLL1Speed);
      //   mRightMotor.set(IntakeConstants.intakeLR1Speed);
      // }
      // else{
      //   mLeftMotor.set(IntakeConstants.intakeL2Speed);
      //   mRightMotor.set(IntakeConstants.intakeL2Speed);
      // }
      mLeftMotor.set(IntakeConstants.intakeL2Speed);
      mRightMotor.set(IntakeConstants.intakeL2Speed);
    }); 
  }
  public Command intakeStowCoral(){
    return runOnce(() -> {
      stowCoral();
      // Timer.delay(0.15);
      Commands.waitUntil(() -> mPeriodicIO.bHasCoral);
      Timer.delay(0.05);
      stop();
    });
  }

  public Command dealgae(ElevatorSubsystem es) {
    return runOnce(() -> {
      
    });
  }
  

  @Override
  public void periodic() {
    // sus
    mPeriodicIO.measurement = lc.getMeasurement();
    if (mPeriodicIO.measurement == null) return;
    if (mPeriodicIO.index_debounce == 10) {
      mPeriodicIO.index_debounce = 0;
      if (mPeriodicIO.lastbHasCoral == mPeriodicIO.tempbHasCoral) {
        mPeriodicIO.bHasCoral = mPeriodicIO.tempbHasCoral;
      }
      mPeriodicIO.lastbHasCoral = mPeriodicIO.tempbHasCoral;
    } else {
      mPeriodicIO.index_debounce++;
      if (mPeriodicIO.measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT)
        mPeriodicIO.tempbHasCoral = mPeriodicIO.measurement.distance_mm <= IntakeConstants.coralPresenceMm;
    }
  }
}

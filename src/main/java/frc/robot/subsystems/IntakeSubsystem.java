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

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;


public class IntakeSubsystem extends SubsystemBase {
  private final SparkMax mLeftMotor = new SparkMax(IntakeConstants.intakeleftMotor, MotorType.kBrushless);
  private final SparkMax mRightMotor = new SparkMax(IntakeConstants.intakerightMotor, MotorType.kBrushless);
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
  }
  private static class PeriodicIO {
    int index_debounce = 0;
    // LaserCan.Measurement measurement;
    boolean intakeisRunning = false;
  }

  void L1(){
    mLeftMotor.set(0.1);
    mRightMotor.set(0.7);
    mPeriodicIO.intakeisRunning = true;
  }

  void L2(){
    // System.out.print(mPeriodicIO.intakeisRunning);
    // if (mPeriodicIO.intakeisRunning){
    //   stop();
    // }
    // else {
    mLeftMotor.set(IntakeConstants.intakeL2Speed);
    mRightMotor.set(IntakeConstants.intakeL2Speed);
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
        case 0: //STOWED
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
      Timer.delay(0.15);
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
  }
}

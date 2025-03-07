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
  /** Creates a new IntakeSubsystem. */
  private PeriodicIO mPeriodicIO;
  private double intakeCurVoltage;
  private double intakeCurCurrent;

  public IntakeSubsystem() {
    mPeriodicIO = new PeriodicIO();
    SparkMaxConfig intakeConfig = new SparkMaxConfig();
    intakeConfig.idleMode(IdleMode.kBrake)
                .voltageCompensation(IntakeConstants.intakeVoltageCompensation)
                .smartCurrentLimit(IntakeConstants.kMaxCurrent);
    mLeftMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    intakeConfig.inverted(true);
    mRightMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
  }
  private static class PeriodicIO {
    
  }

  void L1(){
    mLeftMotor.set(IntakeConstants.intakeLL1Speed);
    mRightMotor.set(IntakeConstants.intakeLR1Speed);
    // L2();
  }

  public void L2(){
    mLeftMotor.set(IntakeConstants.intakeL2Speed);
    mRightMotor.set(IntakeConstants.intakeL2Speed);
  }
  public void stop(){
    mLeftMotor.set(0);
    mRightMotor.set(0);
    System.out.println("Intake stopped!");
  }
  public void stowCoral(){
    mLeftMotor.set(0.3);
    
    mRightMotor.set(0.3);
  }

  public Command scoreL1(ElevatorSubsystem e){
    return runOnce(() -> {
      System.out.println(e.elestate);
      L1();
    });
  }

  public Command intakeStop(ElevatorSubsystem e){
    return runOnce(() -> {
      stop();
      
      if (e.elestate == 2 || e.elestate == 1){
        Timer.delay(0.3);
        e.elevatorStow();
      }
    });
  }

  public Command score(ElevatorSubsystem e) {
    return runOnce(() -> {
      System.out.println(e.elestate);
      // switch (e.elestate) {
      //   case 0: // STOWED
      //     break;
      //   case 1:
      //     L1();
      //     System.out.println("Used l1 intake speed!");
      //     break;
      //   case 2:
      //     L2();
      //     Timer.delay(1);
      //     System.out.println("Used l2 intake speed!");
      //     e.elevatorStow();
      //     System.out.println("Elevator stowed");
      //     break;
      //   default:
      //     System.out.println("Invalid elevator state!");
      // }
      L2();
      if (e.elestate == 2 || e.elestate == 1){
        // Timer.delay(1);
        // e.elevatorStow();
      }
    });
  }

  public Command scoreL2(ElevatorSubsystem e){
    return runOnce(() -> {
      System.out.println(e.elestate);
      mLeftMotor.set(IntakeConstants.intakeL2Speed);
      mRightMotor.set(IntakeConstants.intakeL2Speed);
    }); 
  }
  public Command intakeStowCoral(){
    return runOnce(() -> {
      stowCoral();
      Timer.delay(0.15);
      stop();
      // intakeCurVoltage = mLeftMotor.getAppliedOutput()*mLeftMotor.getBusVoltage();

      // intakeCurVoltage = mLeftMotor.getBusVoltage()*mLeftMotor.getAppliedOutput();
      // System.out.println("Current:" + intakeCurVoltage);
      // for (int i = 0; i<=50; i++){
      //   System.out.println(mLeftMotor.getBusVoltage()*mLeftMotor.getAppliedOutput());
      // }
      // if ((intakeCurVoltage - mLeftMotor.getAppliedOutput()*mLeftMotor.getBusVoltage()) > 0.5){
      //   System.out.println("New:" + mLeftMotor.getAppliedOutput()*mLeftMotor.getBusVoltage());
      //   System.out.println("Will stow");
      // }
    });
  }

  public Command dealgae(ElevatorSubsystem es) {
    return runOnce(() -> {
      
    });
  }
  

  @Override
  public void periodic() {
    // System.out.println(mLeftMotor.getAppliedOutput()*mLeftMotor.getBusVoltage());
  }
}

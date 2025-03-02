// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;

public class ElevatorSubsystem extends SubsystemBase {
  
  private TrapezoidProfile mProfile;
  private TrapezoidProfile.State mCurState = new TrapezoidProfile.State();
  private TrapezoidProfile.State mGoalState = new TrapezoidProfile.State();
  private final SparkMax mLeftMotor = new SparkMax(ElevatorConstants.leftMotor, MotorType.kBrushless);
  private final SparkMax mRightMotor = new SparkMax(ElevatorConstants.rightMotor, MotorType.kBrushless);
  private final RelativeEncoder mLeftEncoder;
  private final RelativeEncoder mRightEncoder;
  private final SparkClosedLoopController mLeftPIDController;
  private PeriodicIO mPeriodicIO;
  private double prevUpdateTime = Timer.getFPGATimestamp();
  DigitalInput limitswitch = new DigitalInput(0);

  public ElevatorSubsystem() {
    mPeriodicIO = new PeriodicIO();
    SparkMaxConfig elevatorConfig = new SparkMaxConfig();
    elevatorConfig.closedLoop.pid(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD).iZone(ElevatorConstants.kIZone);
    elevatorConfig.smartCurrentLimit(ElevatorConstants.kMaxCurrent)
                  .idleMode(IdleMode.kBrake);

    // LEFT ELEVATOR MOTOR
    mLeftEncoder = mLeftMotor.getEncoder();
    mLeftPIDController = mLeftMotor.getClosedLoopController();
    // https://docs.revrobotics.com/revlib/spark/closed-loop/position-control-mode
    mLeftMotor.configure(elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    // RIGHT ELEVATOR MOTOR
    mRightEncoder = mRightMotor.getEncoder();
    mRightMotor.configure(elevatorConfig.follow(mLeftMotor, true), ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    // TRAPEZOID PROFILE
    mProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(ElevatorConstants.kMaxVelocity, ElevatorConstants.kMaxAcceleration));
  }
  public enum ElevatorState {
    NONE,
    STOW,
    L1,
    L2
  }
  private static class PeriodicIO {
    // elevator data to be updated in periodic
    double elevator_target = 0.0;
    double elevator_power = 0.0;
    boolean is_elevator_pos_control = false;
    ElevatorState state = ElevatorState.STOW;
  }

  @Override
  public void periodic() {
    // if (!limitswitch.get() && (mPeriodicIO.state == ElevatorState.STOW)){
    //   System.out.println("Limit switch hit!");
    //   stop();
    // }
    double curTime = Timer.getFPGATimestamp();
    double dt = curTime - prevUpdateTime;
    prevUpdateTime = curTime;
    if (mPeriodicIO.is_elevator_pos_control) {;
      // update goal
      mGoalState.position = mPeriodicIO.elevator_target;
      // elevator target is set when button is pressed
      // calculate new state
      prevUpdateTime = curTime;
      mCurState = mProfile.calculate(dt, mCurState, mGoalState);
      // set PID controller to new state
      // still need to understand this
      mLeftPIDController.setReference(
        mCurState.position, 
        SparkBase.ControlType.kPosition, 
        ClosedLoopSlot.kSlot0, 
        ElevatorConstants.kG,
        ArbFFUnits.kVoltage);
  } 
  else {
      // not using position control
      mCurState.position = mLeftEncoder.getPosition();
      mCurState.velocity = 0;
      mLeftMotor.set(mPeriodicIO.elevator_power);
  }
  }

  // ----------------- COMMANDS ----------------
  public Command getState() {
    return run(() -> getstate());
  }
  public Command setElevatorPower(double power) {
    return run(() -> setelevatorpower(power));
  }
  public Command goToElevatorL1() {
    return run(() -> elevatorL1());
  }
  public Command goToElevatorL2() {
    return run(() -> elevatorL2());
  }
  public Command goToElevatorStow() {
    return run(() -> elevatorStow());
  }

  public Command stopElevator() {
    return run(() -> stop());
  }

  public ElevatorState getstate() {
    return mPeriodicIO.state;
  }
  
  public void setelevatorpower(double power) {
    mPeriodicIO.is_elevator_pos_control = false;
    mPeriodicIO.elevator_power = power;
  }

  public void stop(){
    mPeriodicIO.is_elevator_pos_control = false;
    mPeriodicIO.elevator_power = 0.0;
    mLeftMotor.set(0.0);
    System.out.println("stopped");
  }

  public void elevatorStow(){
    mPeriodicIO.is_elevator_pos_control = true;
    mPeriodicIO.elevator_target = ElevatorConstants.kStowHeight;
    mPeriodicIO.state = ElevatorState.STOW;
  }
  public void elevatorL1() {
    mPeriodicIO.is_elevator_pos_control = true;
    mPeriodicIO.elevator_target = ElevatorConstants.kL1Height;
    mPeriodicIO.state = ElevatorState.L1;
    
  }
  public void elevatorL2(){
    mPeriodicIO.is_elevator_pos_control = true;
    mPeriodicIO.elevator_target = ElevatorConstants.kL2Height;
    mPeriodicIO.state = ElevatorState.L2;
    System.out.println("Elevator moved to L2");
  }

}

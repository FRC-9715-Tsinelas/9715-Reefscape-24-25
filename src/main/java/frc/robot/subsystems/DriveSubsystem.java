// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

// import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkMax;


public class DriveSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final SparkMax r1 = new SparkMax(DriveConstants.rightFrontMotor, MotorType.kBrushless);
  private final SparkMax r2 = new SparkMax(DriveConstants.rightBackMotor, MotorType.kBrushless);
  private final SparkMax l1 = new SparkMax(DriveConstants.leftFrontMotor, MotorType.kBrushless);
  private final SparkMax l2 = new SparkMax(DriveConstants.leftBackMotor, MotorType.kBrushless);
  
  // IRL: first output is right side, second is left
  DifferentialDrive drive = new DifferentialDrive(
    (double output) -> {
        r1.set(-output);
        r2.set(output);
        // System.out.println("Right side: "+ -output);
    },
    (double output) -> {
        l1.set(output*0.7);
        l2.set(output*0.7);
        // System.out.println("Left side: "+ -output);
    });
  // DifferentialDrive drive = new DifferentialDrive(r1, l1);

  public DriveSubsystem() {

    r1.setCANTimeout(250);
    r2.setCANTimeout(250);
    l1.setCANTimeout(250);
    l2.setCANTimeout(250);


    SparkMaxConfig config = new SparkMaxConfig();
    config.voltageCompensation(DriveConstants.driveVoltageCompensation);
    config.smartCurrentLimit(DriveConstants.driveCurrentLimit);
    config.idleMode(IdleMode.kCoast);
    config.follow(r1);
    r2.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    config.follow(l1);
    l2.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    config.disableFollowerMode();
    r1.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    
    config.inverted(true);
    l1.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    drive.setDeadband(0.04);
  }
  

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command driveArcadeCommand(DriveSubsystem driveSubsystem, DoubleSupplier xSpeed, DoubleSupplier zRotation) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return Commands.run(
        () -> 
          drive.arcadeDrive(xSpeed.getAsDouble(), zRotation.getAsDouble()), driveSubsystem);
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
    System.out.printf("->1 vel: %.3f, out: %.3f\n", l1.getEncoder().getVelocity(), l1.getAppliedOutput());
    System.out.printf("->2 vel: %.3f, out: %.3f\n", l2.getEncoder().getVelocity(), l1.getAppliedOutput());
    System.out.printf("<-1 vel: %.3f, out: %.3f\n", r1.getEncoder().getVelocity(), r1.getAppliedOutput());
    System.out.printf("<-2 vel: %.3f, out: %.3f\n", r2.getEncoder().getVelocity(), r2.getAppliedOutput());
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

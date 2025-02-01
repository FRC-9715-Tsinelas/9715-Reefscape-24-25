package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Drivetrain extends SubsystemBase implements Subsystem {
  private final SparkMax l1 = new SparkMax(3, MotorType.kBrushless);
  private final SparkMax l2 = new SparkMax(2, MotorType.kBrushless);
  private final SparkMax r1 = new SparkMax(4, MotorType.kBrushless);
  private final SparkMax r2 = new SparkMax(1, MotorType.kBrushless);

  private final CommandXboxController controller;

  DifferentialDrive drive = new DifferentialDrive(
    (double output) -> {
      l1.set(output);
      l2.set(output);
    },
    (double output) -> {
      r1.set(output);
      r2.set(output);
    });

  public Drivetrain(CommandXboxController c) {
    controller = c;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  @Override
  public void reset() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'reset'");
  }

  @Override
  public void stop() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'stop'");
  }

  @Override
  public void outputTelemetry() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'outputTelemetry'");
  }

  @Override
  public void writeToLog() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'writeToLog'");
  }

  @Override
  public void teleopPeriodic() {
    // TODO Auto-generated method stub
    // Operator control: drive from controller input
    // +X = forward on joystick. Y is left/right (turn)
    drive.arcadeDrive(controller.getLeftX(), controller.getRightY());
  }
}
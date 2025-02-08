package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;


public class ElevatorSubsystem extends SubsystemBase {

  private final SparkMax leftMotor = new SparkMax(ElevatorConstants.leftMotor, MotorType.kBrushless);
  private final SparkMax rightMotor = new SparkMax(ElevatorConstants.rightMotor, MotorType.kBrushless);
  private final DigitalInput topLimSwitch = new DigitalInput(ElevatorConstants.topLimSwitch);
  // private final DigitalInput bottomLimSwitch = new DigitalInput(ElevatorConstants.bottomLimSwitch);
  private final RelativeEncoder lencoder;

  public ElevatorSubsystem() {
    lencoder = leftMotor.getEncoder();
    System.out.println(lencoder.getPosition());
  }

  private final class LPos{
    static final int none = -1;
    static final int l1 = 0;
    static final int l2 = 1;
    static final int l3 = 2;
  }
  private int targetPos = LPos.none;
  private double currentPos = 0.0; // inches!!!

  @Override
  public void periodic() {
    // Determine current elevator position
    currentPos = lencoder.getPosition() * ElevatorConstants.revsToHeightIn;
    // Move elevator to target position
  }

  @Override
  public void simulationPeriodic() {}

  // Elevator will need to go to 3 different heights for the game.
  public Command elevatorL1Command() {
    return Commands.run(() -> {
      targetPos = LPos.l1;
    }, this);
  }
  public Command elevatorL2Command() {
    return Commands.run(() -> {
      targetPos = LPos.l2;
    }, this);
  }
  public Command elevatorL3Command() {
    return Commands.run(() -> {
      targetPos = LPos.l3;
    }, this);
  }
  
}

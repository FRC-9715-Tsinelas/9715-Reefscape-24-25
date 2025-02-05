package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
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
  private final DigitalInput bottomLimSwitch = new DigitalInput(ElevatorConstants.bottomLimSwitch);

  private final class LPos{
    static final int none = -1;
    static final int l1 = 0;
    static final int l2 = 1;
    static final int l3 = 2;
  }
  private int targetPos = LPos.none;

  @Override
  public void periodic() {
    // Move elevator to target position
  }

  @Override
  public void simulationPeriodic() {}

  // Elevator will need to go to 3 different heights for the game.
  public Command elevatorL1Command() {
    return Commands.run(() -> {}, this);
  }
  public Command elevatorL2Command() {return null;}
  public Command elevatorL3Command() {return null;}
  
}

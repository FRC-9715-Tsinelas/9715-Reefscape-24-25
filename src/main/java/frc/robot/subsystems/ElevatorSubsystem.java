package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ElevatorConstants;


public class ElevatorSubsystem extends SubsystemBase {
  private final DigitalInput topLimSwitch = new DigitalInput(ElevatorConstants.topLimSwitch);
  private final PIDController pidController =
      new PIDController(ElevatorConstants.kp, ElevatorConstants.ki, ElevatorConstants.kd);
  // private final DigitalInput bottomLimSwitch = new DigitalInput(ElevatorConstants.bottomLimSwitch);

  private class Elevator {
    static SparkMax lMotor = new SparkMax(ElevatorConstants.leftMotor, MotorType.kBrushless);
    static SparkMax rMotor = new SparkMax(ElevatorConstants.rightMotor, MotorType.kBrushless);
    static RelativeEncoder lencoder = Elevator.lMotor.getEncoder();
    static RelativeEncoder rencoder = Elevator.rMotor.getEncoder();
    /**
     * Returns the average rotational position of both motors.
     */
    static double rPos() {
      return (lencoder.getPosition() + rencoder.getPosition())/2;
    }
    static double position() {
      return rPos() * ElevatorConstants.revsToHeightIn;
    }
    /**
     * Set the speed of both motors.
     */
    static void mset(double s) {
      lMotor.set(s);
      rMotor.set(s);
    }
    /** Returns the difference between the two motors' positions.
      * When negative, the left motor's position is lesser/lower.
      */
    static double diff() {
      return lencoder.getPosition() - rencoder.getPosition();
    }
  }

  public ElevatorSubsystem() {
    Elevator.lencoder = Elevator.lMotor.getEncoder();
    Elevator.rencoder = Elevator.rMotor.getEncoder();
    System.out.println(Elevator.rPos());
  }

  private final class LPos{
    static double user = -1;
    // all in inches
    static final double l1 = 1;
    static final double l2 = 2;
    static final double l3 = 3;
  }
  private double userPos = 0;
  private double targetPos = LPos.user;
  // private double currentPos = 0.0; // inches!!!

  @Override
  public void periodic() {
    // Determine current elevator position
    double currentPos = Elevator.position();
    double setpointPos = targetPos;
    // Move elevator to target position
    if (targetPos < 0) {
      setpointPos = userPos;
    }
    if (topLimSwitch.get()) {
      // Reset targetposition and user position
      targetPos = LPos.l1;
      userPos = LPos.l1;
    }
    Elevator.mset(pidController.calculate(currentPos / ElevatorConstants.revsToHeightIn,
        setpointPos / ElevatorConstants.revsToHeightIn));
  }

  @Override
  public void simulationPeriodic() {}

  // Elevator will need to go to 3 different heights for the game.
  public Command L1Command() {
    return Commands.run(() -> {
      targetPos = LPos.l1;
    }, this);
  }
  public Command L2Command() {
    return Commands.run(() -> {
      targetPos = LPos.l2;
    }, this);
  }
  public Command L3Command() {
    return Commands.run(() -> {
      targetPos = LPos.l3;
    }, this);
  }
  public Command userPosChangeCommand(boolean raise, CommandXboxController controller) {
    return Commands.run(() -> {
      userPos += ElevatorConstants.userPosChangeRate * (raise?1:-1);
      if (userPos < 0) userPos = 0;
    });
  }

}

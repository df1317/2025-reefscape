package frc.robot.subsystems.elevator;


import static frc.robot.subsystems.elevator.ElevatorConstants.elevatorCurrentLimit;
import static frc.robot.subsystems.elevator.ElevatorConstants.kd;
import static frc.robot.subsystems.elevator.ElevatorConstants.ki;
import static frc.robot.subsystems.elevator.ElevatorConstants.kp;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.CanConstants;

public class ElevatorIOSpark implements ElevatorIO {
  private SparkMax motorL;
  private SparkMax motorR;

  private RelativeEncoder encoderL;
  private RelativeEncoder encoderR;

  private SparkMaxConfig config = new SparkMaxConfig();

  private SparkClosedLoopController controllerL;
  private SparkClosedLoopController controllerR;

  public ElevatorIOSpark() {
    motorL = new SparkMax(CanConstants.elevatorMotorL, MotorType.kBrushless);
    motorR = new SparkMax(CanConstants.elevatorMotorR, MotorType.kBrushless);
    // the defualt is lsot zero
    config.closedLoop.p(kp, ClosedLoopSlot.kSlot0).i(ki, ClosedLoopSlot.kSlot0).d(kd,
        ClosedLoopSlot.kSlot0);
    config.smartCurrentLimit(elevatorCurrentLimit).idleMode(IdleMode.kBrake);
    config.signals.primaryEncoderPositionPeriodMs(10);

    motorL.configure(config.inverted(false), ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters);
    motorR.configure(config.inverted(true), ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters);

    controllerL = motorL.getClosedLoopController();
    controllerR = motorR.getClosedLoopController();

    encoderL = motorL.getEncoder();
    encoderR = motorR.getEncoder();
  }

  @Override
  public double getRightEncoderPosition() {
    return encoderR.getPosition();
  }

  @Override
  public double getLeftEncoderPosition() {
    return encoderL.getPosition();
  }



  @Override
  public void setReference(double setpoint, double feedForwardArb) {
    controllerL.setReference(setpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0,
        feedForwardArb);

    controllerR.setReference(setpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0,
        feedForwardArb);
  }
}

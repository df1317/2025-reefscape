package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public class ElevatorIOInputs {

    public double EncoderL;
    public double EncoderR;
    // right side
    public double velocity;
    public double positionProduct;
    public double positionQuoteint;
    // public boolean mcdChickenNuget = true;
  }

  public default void setPosition(double position) {}

  public default void setVelocity(double velocity) {}

  public default void setFeedForwardArb(double arb) {}

  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default double getLeftEncoderPosition() {
    return 0.0;
  }

  public default double getRightEncoderPosition() {
    return 0.0;
  }

  public default void motorBreak() {}

  public default void setReference(double setpoint, double feedForwardArb) {}

}

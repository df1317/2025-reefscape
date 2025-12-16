package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public class ElevatorIOInputs {

    public double position;
    public double velocity;
    public double feedForwardArb;

  }

  public default void setPosition(double position) {
  }

  public default void setVelocity(double velocity) {
  }

  public default void setFeedForwardArb() {
  }

  public default void updateInputs(ElevatorIOInputs inputs) {
  }

}

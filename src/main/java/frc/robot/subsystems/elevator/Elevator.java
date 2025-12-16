package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {

  public TrapezoidProfile.Constraints ffc = new TrapezoidProfile.Constraints(ElevatorConstants.maxV,
      ElevatorConstants.maxA);
  public TrapezoidProfile.State ffState = new TrapezoidProfile.State();
  public TrapezoidProfile.State preRenfernce = new TrapezoidProfile.State();
  public TrapezoidProfile Profiler = new TrapezoidProfile(ffc);
  public ElevatorFeedforward ff = new ElevatorFeedforward(ElevatorConstants.ks, ElevatorConstants.kg,
      ElevatorConstants.kv);

  @Override
  public void periodic() {

  }

  public void setSpeed(double velo) {
  }

}

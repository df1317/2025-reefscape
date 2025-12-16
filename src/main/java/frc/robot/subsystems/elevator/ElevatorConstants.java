package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;

public class ElevatorConstants {
  public static final double ks = 0.36656, kg = 0.48642, kv = 4.7049;
  public static final double kp = 0.00065323, ki = 0, kd = 0;
  public static final double maxV = 1, maxA = 1;
  public static final double krot = 42.4; // rotations/meter

  private static final double upSpeed = 0.5;
  private static final double downSpeed = 0.1;
  private static final int elevatorCurrentLimit = 30;

  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutVoltage m_appliedVoltage = Volts.mutable(0);
  private final MutDistance m_distance = Meters.mutable(0);
  private final MutLinearVelocity m_velocity = MetersPerSecond.mutable(0);
}

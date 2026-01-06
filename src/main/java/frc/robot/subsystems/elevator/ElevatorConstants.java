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

  public static final double upSpeed = 0.5;
  public static final double downSpeed = 0.1;
  public static final int elevatorCurrentLimit = 30;

  public static final double maxHeight = 1.23;
  public static final double minHeight = 0;

  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  public static final MutVoltage m_appliedVoltage = Volts.mutable(0);
  public static final MutDistance m_distance = Meters.mutable(0);
  public static final MutLinearVelocity m_velocity = MetersPerSecond.mutable(0);
}

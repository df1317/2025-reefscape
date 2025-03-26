/* ----------
 * Copywrite 2025 FRC team 1317 under AGPL-3.0
 * ----------- */

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/** ----------
 * Constants
 * ---
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 * ---
 */
public final class Constants {

	public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
	public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Inches.of(8).in(Meters)), ROBOT_MASS);
	public static final double LOOP_TIME = 0.13; //s, 20ms + 110ms sprk max velocity lag
	public static final double MAX_SPEED = Units.feetToMeters(14.5);
	public static final double MAX_ANGULAR_SPEED = Math.toRadians(240.0);
	public static final double MAX_ACCELERATION = 1.5;
	public static final double MAX_ANGULAR_ACCELERATION = Math.toRadians(120.0);

	public static final class DrivebaseConstants {

		// Hold time on motor brakes when disabled
		public static final double WHEEL_LOCK_TIME = 10; // seconds
	}

	public static class OperatorConstants {

		// Joystick Deadband
		public static final double DEADBAND = 0.1;
		public static final double LEFT_Y_DEADBAND = 0.1;
		public static final double RIGHT_X_DEADBAND = 0.1;
		public static final double TURN_CONSTANT = 6;
	}

	public static class CanConstants {

		// Scoring Motor IDs
		public static final int scoreMotor1 = 24;
		public static final int scoreMotor2 = 25;
		public static final int scoreTiltMotor = 28;

		// Elevator Motor IDs
		public static final int elevatorMotorL = 26;
		public static final int elevatorMotorR = 27;

		// Climber Motor IDs
		public static final int beefyMotor = 29;
	}

	public static class DIOConstants {

		// Sensor ports
		public static final int coralSensorPort = 1;
		public static final int elevatorEncoderL = 2;
		public static final int elevatorEncoderR = 3;
		public static final int homingTiltClickySwitch = 4;
	}

	public static class AutoScoring {

		public static final Pose2d SCORING_AUTO_POSE = new Pose2d(6.162, 4.558, new Rotation2d(Math.toRadians(180)));

		public static class Processor {

			public static final Transform2d offset = new Transform2d(
				Inches.of(24).in(Meters),
				Inches.of(0).in(Meters),
				Rotation2d.fromDegrees(0)
			);
		}

		public static class Reef {

			public static final Transform2d coralOffsetL = new Transform2d(
				Inches.of(21.5).in(Meters),
				Inches.of(4).in(Meters),
				Rotation2d.fromDegrees(180)
			);

			public static final Transform2d coralOffsetR = new Transform2d(
				Inches.of(21.5).in(Meters),
				Inches.of(2.5).in(Meters),
				Rotation2d.fromDegrees(180)
			);

			public static final Transform2d algaeOffset = new Transform2d(
				Inches.of(18).in(Meters),
				Inches.of(0).in(Meters),
				Rotation2d.fromDegrees(0)
			);
		}

		public static class HumanPlayer {

			public static class Left {

				public static final Transform2d offset = new Transform2d(
					Inches.of(24).in(Meters),
					Inches.of(0).in(Meters),
					Rotation2d.fromDegrees(0)
				);
			}

			public static class Right {

				public static final Transform2d offset = new Transform2d(
					Inches.of(24).in(Meters),
					Inches.of(0).in(Meters),
					Rotation2d.fromDegrees(0)
				);
			}
		}
	}
}

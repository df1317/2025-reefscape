package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.CanConstants;
import frc.robot.Constants.DIOConstants;
import java.util.function.DoubleSupplier;

public class ScoringSubsystem extends SubsystemBase {

	private SparkMax motor1;
	private SparkMax motor2;
	private SparkClosedLoopController spinnyController, spinnyController2;
	private final double kp = 0.01, ki = 0, kd = 0, kv = 1 / 917;
	private SparkMax canTiltMax;
	private SparkMaxConfig tiltConfig = new SparkMaxConfig();
	private EncoderConfig tiltEncoderConfig = new EncoderConfig();
	private RelativeEncoder canTiltEncoder;
	private SparkMaxConfig motorConfig = new SparkMaxConfig();
	private SparkMaxConfig motorConfig2 = new SparkMaxConfig();
	private SparkClosedLoopController canTiltController;
	public DigitalInput coralSensor;
	private double setpoint = 0;

	private static final int scoringCurrent = 25;

	// Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
	private final MutVoltage m_appliedVoltage = Volts.mutable(0);
	private final MutAngle m_rotation = Rotations.mutable(0);
	private final MutAngularVelocity m_velocity = RotationsPerSecond.mutable(0);

	public ScoringSubsystem() {
		motorConfig
			.smartCurrentLimit(35)
			.idleMode(IdleMode.kBrake)
			.inverted(true)
			.closedLoop.p(kp, ClosedLoopSlot.kSlot0)
			.i(ki, ClosedLoopSlot.kSlot0)
			.d(kd, ClosedLoopSlot.kSlot0)
			.velocityFF(kv, ClosedLoopSlot.kSlot0);
		motor1 = new SparkMax(CanConstants.scoreMotor1, MotorType.kBrushless);
		motorConfig.signals.absoluteEncoderPositionPeriodMs(10);
		motor1.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

		motorConfig2
			.smartCurrentLimit(35)
			.idleMode(IdleMode.kBrake)
			.closedLoop.p(kp, ClosedLoopSlot.kSlot0)
			.i(ki, ClosedLoopSlot.kSlot0)
			.d(kd, ClosedLoopSlot.kSlot0)
			.velocityFF(kv, ClosedLoopSlot.kSlot0);
		motor2 = new SparkMax(CanConstants.scoreMotor2, MotorType.kBrushless);
		motor2.configure(motorConfig2, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
		spinnyController = motor1.getClosedLoopController();

		spinnyController2 = motor2.getClosedLoopController();
		coralSensor = new DigitalInput(DIOConstants.coralSensorPort);

		tiltEncoderConfig.countsPerRevolution(8192).inverted(true);

		tiltConfig.inverted(false).smartCurrentLimit(15).apply(tiltEncoderConfig);
		tiltConfig.closedLoop.p(4.5, ClosedLoopSlot.kSlot0).i(0, ClosedLoopSlot.kSlot0).d(0.4, ClosedLoopSlot.kSlot0);
		canTiltMax = new SparkMax(CanConstants.scoreTiltMotor, MotorType.kBrushed);
		canTiltMax.configure(tiltConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
		canTiltController = canTiltMax.getClosedLoopController();
		canTiltEncoder = canTiltMax.getEncoder();
	}

	@Override
	public void periodic() {
		// Only apply limits when not in test mode
		if (!DriverStation.isTest()) {
			setpoint = MathUtil.clamp(setpoint, 0, 0.4);
		}
		canTiltController.setReference(setpoint, ControlType.kPosition);

		SmartDashboard.putNumber("scoring/setpoint", setpoint);
		SmartDashboard.putNumber("scoring/tilt current", canTiltMax.getOutputCurrent());
		SmartDashboard.putNumber("scoring/tilt voltage", canTiltMax.getBusVoltage() * canTiltMax.getAppliedOutput());
		SmartDashboard.putNumber("scoring/tilt encoder", canTiltMax.getEncoder().getPosition());
		SmartDashboard.putBoolean("scoring/coral sensor", !coralSensor.get());
		SmartDashboard.putNumber("scoring/motor1 current", motor1.getOutputCurrent());
		SmartDashboard.putNumber("scoring/motor1 voltage", motor1.getBusVoltage() * motor1.getAppliedOutput());
		SmartDashboard.putNumber("scoring/motor2 current", motor2.getOutputCurrent());
		SmartDashboard.putNumber("scoring/motor2 voltage", motor2.getBusVoltage() * motor2.getAppliedOutput());

		SmartDashboard.putNumber("scoring/motor1 temp", motor1.getMotorTemperature());
		SmartDashboard.putNumber("scoring/motor2 temp", motor2.getMotorTemperature());
	}

	public Command runIntakeCommand() {
		return this.run(() -> {
				// System.out.println("intake command started");
				spinnyController.setReference(scoringCurrent, ControlType.kCurrent);
				spinnyController2.setReference(scoringCurrent, ControlType.kCurrent);
			})
			.until(() -> !coralSensor.get())
			.withTimeout(4.5)
			.andThen(
				this.run(() -> {
						spinnyController.setReference(scoringCurrent, ControlType.kCurrent);
						spinnyController2.setReference(scoringCurrent, ControlType.kCurrent);
					})
					.finallyDo(() -> {
						// System.out.println("intake command finished");
						spinnyController.setReference(0, ControlType.kCurrent);
						spinnyController2.setReference(0, ControlType.kCurrent);
					})
					.withTimeout(0.2)
			)
			.finallyDo(() -> {
				// System.out.println("intake command finished");
				spinnyController.setReference(0, ControlType.kCurrent);
				spinnyController2.setReference(0, ControlType.kCurrent);
			});
	}

	public Command runEjectCommand() {
		return this.run(() -> {
				// System.out.println("eject command started");
				spinnyController.setReference(-scoringCurrent, ControlType.kCurrent);
				spinnyController2.setReference(-scoringCurrent, ControlType.kCurrent);
			})
			.finallyDo(() -> {
				// System.out.println("eject command ended");
				spinnyController.setReference(0, ControlType.kCurrent);
				spinnyController2.setReference(0, ControlType.kCurrent);
			})
			.withTimeout(1.7);
	}

	public boolean atDesiredPosition() {
		return MathUtil.isNear(setpoint, canTiltEncoder.getPosition(), 0.1);
	}

	public boolean getCoralSensor() {
		return coralSensor.get();
	}

	public Command tiltCommand(double degrees) {
		return this.runOnce(() -> {
				double rotations = Units.degreesToRotations(degrees);
				if (!DriverStation.isTest()) {
					rotations = MathUtil.clamp(rotations, 0, 0.4);
				}
				setpoint = rotations;
			});
	}

	public Command tiltNudge(boolean direction) {
		double amount = Units.degreesToRotations(0.4);
		return Commands.run(() -> {
			if (direction) {
				if (DriverStation.isTest()) {
					setpoint = setpoint + amount;
				} else {
					setpoint = MathUtil.clamp(setpoint + amount, 0, 0.4);
				}
			} else {
				if (DriverStation.isTest()) {
					setpoint = setpoint - amount;
				} else {
					setpoint = MathUtil.clamp(setpoint - amount, 0, 0.4);
				}
			}
		});
	}

	public Command tiltJoy(DoubleSupplier joy) {
		return Commands.run(() -> {
			double joyVal = MathUtil.applyDeadband(joy.getAsDouble(), 0.1);
			double adjustment = Units.degreesToRotations(joyVal);

			if (DriverStation.isTest()) {
				setpoint = setpoint + adjustment;
			} else {
				setpoint = MathUtil.clamp(setpoint + adjustment, 0, 0.4);
			}
		});
	}

	public Command zeroEncoder() {
		return this.runOnce(() -> {
				canTiltEncoder.setPosition(0);
				setpoint = 0;
			});
	}

	public Command tiltSysIDCommand(double quasiTimeout, double timeout, double dynamicTimeout) {
		return m_sysIdRoutine
			.quasistatic(SysIdRoutine.Direction.kForward)
			.withTimeout(quasiTimeout)
			.andThen(Commands.waitSeconds(timeout))
			.andThen(m_sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse).withTimeout(quasiTimeout))
			.andThen(Commands.waitSeconds(timeout))
			.andThen(m_sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward).withTimeout(dynamicTimeout))
			.andThen(Commands.waitSeconds(timeout))
			.andThen(m_sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse).withTimeout(dynamicTimeout));
	}

	private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
		// Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
		new SysIdRoutine.Config(null, Voltage.ofBaseUnits(5, Volts), null, null),
		new SysIdRoutine.Mechanism(
			// Tell SysId how to plumb the driving voltage to the motors.
			voltage -> {
				canTiltMax.setVoltage(voltage);
			},
			// Tell SysId how to record a frame of data for each motor on the mechanism
			// being
			// characterized.
			log -> {
				log
					.motor("tilt")
					.voltage(
						m_appliedVoltage.mut_replace(canTiltMax.getBusVoltage() * canTiltMax.getAppliedOutput(), Volts)
					)
					.angularPosition(m_rotation.mut_replace(canTiltEncoder.getPosition(), Rotations))
					.angularVelocity(m_velocity.mut_replace(canTiltEncoder.getVelocity(), RotationsPerSecond));
			},
			// Tell SysId to make generated commands require this subsystem, suffix test
			// state in
			// WPILog with this subsystem's name ("drive")
			this
		)
	);
}

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.CanConstants;

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
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.DIOConstants;
import java.util.function.DoubleSupplier;

public class ElevatorSubsystem extends SubsystemBase {

	private final double maxHeight = 1.23;
	private final double minHeight = 0;
	private long t = System.nanoTime();
	private RelativeEncoder encoderL;
	private RelativeEncoder encoderR;

	public DigitalInput limitSwitchTop = new DigitalInput(DIOConstants.elevatorTopLimitSwitch);
	public DigitalInput limitSwitchBottom = new DigitalInput(DIOConstants.elevatorBottomLimitSwitch);

	private enum LimitSwitchTrigger {
		TOP,
		NONE,
		BOTTOM,
		TEST
	}

	private SparkMax motorL;
	private SparkMax motorR;
	private SparkClosedLoopController controllerL;
	private SparkClosedLoopController controllerR;
	private SparkMaxConfig config = new SparkMaxConfig();
	private double kp = 0.00065323, ki = 0, kd = 0;
	private double maxV = 1, maxA = 1;
	private double krot = 42.4; // rotations/meter
	private static final double upSpeed = 0.5;
	private static final double downSpeed = 0.1;
	private static final int elevatorCurrentLimit = 30;
	private double ks = 0.36656, kg = 0.48642, kv = 4.7049;

	private double currentMaxVel = maxV;
	private TrapezoidProfile.Constraints ffc = new TrapezoidProfile.Constraints(maxV, maxA);
	private TrapezoidProfile.State ffState = new TrapezoidProfile.State();
	private TrapezoidProfile.State preRenfernce = new TrapezoidProfile.State();
	private TrapezoidProfile Profiler = new TrapezoidProfile(ffc);
	private ElevatorFeedforward ff = new ElevatorFeedforward(ks, kg, kv);

	// Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
	private final MutVoltage m_appliedVoltage = Volts.mutable(0);
	private final MutDistance m_distance = Meters.mutable(0);
	private final MutLinearVelocity m_velocity = MetersPerSecond.mutable(0);

	public ElevatorSubsystem() {
		motorL = new SparkMax(CanConstants.elevatorMotorL, MotorType.kBrushless);
		motorR = new SparkMax(CanConstants.elevatorMotorR, MotorType.kBrushless);
		// the defualt is lsot zero
		config.closedLoop.p(kp, ClosedLoopSlot.kSlot0).i(ki, ClosedLoopSlot.kSlot0).d(kd, ClosedLoopSlot.kSlot0);
		config.smartCurrentLimit(elevatorCurrentLimit).idleMode(IdleMode.kBrake);

		motorL.configure(config.inverted(false), ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
		motorR.configure(config.inverted(true), ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

		controllerL = motorL.getClosedLoopController();
		controllerR = motorR.getClosedLoopController();

		encoderL = motorL.getEncoder();
		encoderR = motorR.getEncoder();

		System.out.println("TODO: please put reset encodes pos in");
	}

	public void setDesiredPosistion(double height, double time) {
		ffState.position = height;
		ffState.velocity = 0.0;
	}

	private LimitSwitchTrigger checkLimits() {
		if (limitSwitchTop.get()) {
			return LimitSwitchTrigger.TOP;
		} else if (limitSwitchBottom.get()) {
			return LimitSwitchTrigger.BOTTOM;
		} else {
			return LimitSwitchTrigger.NONE;
		}
	}

	private void motorBreak() {
		ffState.position = preRenfernce.position;
		ffState.velocity = 0.0;

		motorL.stopMotor();
		motorR.stopMotor();
	}

	@Override
	public void periodic() {
		double ffValue = 0.0;
		boolean running = false;
		double height = encoderL.getPosition() / krot;

		ffState.position = MathUtil.clamp(ffState.position, 0, maxHeight);
		ffState.velocity = 0.0;

		preRenfernce.position = MathUtil.isNear(ffState.position, height, 0.1) ? preRenfernce.position : height;
		// preRenfernce.velocity = (encoderL.getVelocity() / krot) / 60.0;

		preRenfernce.velocity = MathUtil.clamp(preRenfernce.velocity, -currentMaxVel, currentMaxVel);
		preRenfernce.position = MathUtil.clamp(preRenfernce.position, 0, maxHeight);

		preRenfernce = Profiler.calculate((System.nanoTime() - t) / 1e9, preRenfernce, ffState);

		t = System.nanoTime();
		ffValue = ff.calculate(MathUtil.clamp(preRenfernce.velocity, -currentMaxVel, currentMaxVel));

		switch (checkLimits()) {
			case NONE:
				running = true;
				break;
			case BOTTOM:
				if (preRenfernce.velocity > 0) {
					running = true;
				} else {
					this.motorBreak();
				}
				break;
			case TOP:
				if (preRenfernce.velocity < 0) {
					running = true;
				} else {
					this.motorBreak();
				}
				break;
			case TEST:
				System.out.println("why did you run this? you forgot to program in the limits");
				running = false;
				break;
		}
		if (running) {
			controllerL.setReference(
				preRenfernce.position * krot,
				ControlType.kPosition,
				ClosedLoopSlot.kSlot0,
				ffValue
			);
			controllerR.setReference(
				preRenfernce.position * krot,
				ControlType.kPosition,
				ClosedLoopSlot.kSlot0,
				ffValue
			);
		}

		SmartDashboard.putNumber("elevator/current-position", preRenfernce.position);
		SmartDashboard.putNumber("elevator/current-velocity", preRenfernce.velocity);
		SmartDashboard.putNumber("elevator/end-position", ffState.position);
		SmartDashboard.putNumber("elevator/currentMaxVal", currentMaxVel);
		SmartDashboard.putBoolean("elevator/topLimitSwitch", limitSwitchTop.get());
		SmartDashboard.putBoolean("elevator/bottomLimitSwitch", limitSwitchBottom.get());
		SmartDashboard.putNumber("elevator/encoderL", encoderL.getPosition());
		SmartDashboard.putNumber("elevator/encoderR", encoderR.getPosition());
		SmartDashboard.putNumber("elevator/motorL current", motorL.getOutputCurrent());
		SmartDashboard.putNumber("elevator/motorL voltage", motorL.getBusVoltage());
		SmartDashboard.putNumber("elevator/motorR current", motorR.getOutputCurrent());
		SmartDashboard.putNumber("elevator/motorR voltage", motorR.getBusVoltage());
	}

	public Command setPos(DoubleSupplier height) {
		return Commands.runOnce(() -> {
			currentMaxVel = maxV;
			ffState.position = height.getAsDouble();
			ffState.velocity = 0.0;
		});
	}

	public Command setSpeed(DoubleSupplier velo) {
		return Commands.run(() -> {
			double tol = 0.1;
			currentMaxVel = Math.abs(MathUtil.clamp(velo.getAsDouble(), -maxV, maxV));
			if (currentMaxVel > tol) {
				if (velo.getAsDouble() > 0) {
					ffState.position = maxHeight;
					ffState.velocity = 0.0;
				} else if (velo.getAsDouble() < 0) {
					ffState.position = minHeight;
					ffState.velocity = 0.0;
				}
			} else {
				ffState.position = preRenfernce.position;
				ffState.velocity = 0.0;
			}
		}).finallyDo(() -> {
			ffState.position = preRenfernce.position;
			ffState.velocity = 0.0;
			currentMaxVel = maxV;
		});
	}

	public Command manualElevatorCommand(double input) {
		return this.startEnd(
				() -> {
					if (input > 0) {
						motorL.set(upSpeed * input);
						motorR.set(upSpeed * input);
					} else {
						motorL.set(downSpeed * input);
						motorR.set(downSpeed * input);
					}
				},
				() -> {
					motorL.set(0);
					motorR.set(0);
				}
			);
	}

	public Command zeroCommand() {
		return this.runOnce(() -> {
				preRenfernce.position = 0;
				ffState.position = 0;
				encoderL.setPosition(0);
				encoderR.setPosition(0);
			});
	}

	public Command sysIDQuasistatic(Direction dir, double timeout) {
		return m_sysIdRoutine.quasistatic(dir).withTimeout(timeout);
	}

	public Command sysIDDynamic(Direction dir, double timeout) {
		return m_sysIdRoutine.dynamic(dir).withTimeout(timeout);
	}

	public Command sysIDCommand(double quasiTimeout, double timeout, double dynamicTimeout) {
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
				motorL.setVoltage(voltage);
				motorR.setVoltage(voltage);
			},
			// Tell SysId how to record a frame of data for each motor on the mechanism
			// being
			// characterized.
			log -> {
				// Record a frame for the left motors. Since these share an encoder, we consider
				// the entire group to be one motor.
				log
					.motor("elevator")
					.voltage(m_appliedVoltage.mut_replace(motorL.getBusVoltage() * motorL.getAppliedOutput(), Volts))
					.linearPosition(m_distance.mut_replace(encoderL.getPosition() / krot, Meters))
					.linearVelocity(m_velocity.mut_replace(encoderL.getVelocity() / 60.0 / krot, MetersPerSecond));
			},
			// Tell SysId to make generated commands require this subsystem, suffix test
			// state in
			// WPILog with this subsystem's name ("drive")
			this
		)
	);
}

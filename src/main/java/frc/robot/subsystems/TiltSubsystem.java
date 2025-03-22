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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.CanConstants;
import frc.robot.Constants.DIOConstants;
import java.util.function.DoubleSupplier;


public class TiltSubsystem extends SubsystemBase {

    private final MutVoltage m_appliedVoltage = Volts.mutable(0);
	private final MutAngle m_rotation = Rotations.mutable(0);
	private final MutAngularVelocity m_velocity = RotationsPerSecond.mutable(0);

    private final double kp = 0.01, ki = 0, kd = 0, kv = 1 / 917;
	private SparkMax canTiltMax;
	private SparkMaxConfig tiltConfig = new SparkMaxConfig();
	private EncoderConfig tiltEncoderConfig = new EncoderConfig();
	private RelativeEncoder canTiltEncoder;
    private SparkClosedLoopController canTiltController;
    private double setpoint = 0;

    public TiltSubsystem(){

        tiltConfig.inverted(false).smartCurrentLimit(15).apply(tiltEncoderConfig);
		tiltConfig.closedLoop.p(4.5, ClosedLoopSlot.kSlot0).i(0, ClosedLoopSlot.kSlot0).d(0.4, ClosedLoopSlot.kSlot0);
		canTiltMax = new SparkMax(CanConstants.scoreTiltMotor, MotorType.kBrushed);
		canTiltMax.configure(tiltConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
		canTiltController = canTiltMax.getClosedLoopController();
		canTiltEncoder = canTiltMax.getEncoder();

        tiltEncoderConfig.countsPerRevolution(8192).inverted(true);

    }

    @Override
    public void periodic(){
        setpoint = MathUtil.clamp(setpoint, 0, 0.4);
		canTiltController.setReference(setpoint, ControlType.kPosition);

        SmartDashboard.putNumber("scoring/setpoint", setpoint);
		SmartDashboard.putNumber("scoring/tilt current", canTiltMax.getOutputCurrent());
		SmartDashboard.putNumber("scoring/tilt voltage", canTiltMax.getBusVoltage() * canTiltMax.getAppliedOutput());
		SmartDashboard.putNumber("scoring/tilt encoder", canTiltMax.getEncoder().getPosition());
		 
    }

    public Command tiltCommand(double degrees) {
		return this.runOnce(() -> {
				setpoint = MathUtil.clamp(Units.degreesToRotations(degrees), 0, 0.4);
			});
	}

	public Command tiltNudge(boolean direction) {
		double amount = Units.degreesToRotations(0.4);
		return Commands.run(() -> {
			if (direction) {
				setpoint = MathUtil.clamp(setpoint + amount, 0, 0.4);
			} else {
				setpoint = MathUtil.clamp(setpoint - amount, 0, 0.4);
			}
		});
	}

	public Command tiltJoy(DoubleSupplier joy) {
		return Commands.run(() -> {
			double joyVal = MathUtil.applyDeadband(joy.getAsDouble(), 0.05);
			setpoint = MathUtil.clamp(setpoint + Units.degreesToRotations(joyVal), 0, 0.4);
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

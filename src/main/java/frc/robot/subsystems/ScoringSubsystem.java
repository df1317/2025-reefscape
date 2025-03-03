package frc.robot.subsystems;

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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanConstants;
import frc.robot.Constants.DIOConstants;

public class ScoringSubsystem extends SubsystemBase {

	private SparkMax motor1;
	private SparkMax motor2;
	private SparkClosedLoopController spinnyController, spinnyController2;
	private final double kp = 0.01, ki = 0, kd = 0, kv = 1 / 917;
	private SparkMax canTiltMax;
	private SparkMaxConfig tiltConfig = new SparkMaxConfig();
	private EncoderConfig tiltEncoderConfig = new EncoderConfig();
	private SparkMaxConfig motorConfig = new SparkMaxConfig();
	private SparkMaxConfig motorConfig2 = new SparkMaxConfig();
	private SparkClosedLoopController canTiltController;
	private DigitalInput homingTiltClickySwitch;
	public DigitalInput coralSensor;

	public ScoringSubsystem() {
		motorConfig
			.smartCurrentLimit(25)
			.idleMode(IdleMode.kBrake)
			.inverted(true)
			.closedLoop.p(kp, ClosedLoopSlot.kSlot0)
			.i(ki, ClosedLoopSlot.kSlot0)
			.d(kd, ClosedLoopSlot.kSlot0)
			.velocityFF(kv, ClosedLoopSlot.kSlot0);
		motor1 = new SparkMax(CanConstants.scoreMotor1, MotorType.kBrushless);
		motor1.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

		motorConfig2
			.smartCurrentLimit(25)
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

		tiltConfig.inverted(true).smartCurrentLimit(15).apply(tiltEncoderConfig);
		tiltConfig.closedLoop.p(0.1, ClosedLoopSlot.kSlot0).i(0, ClosedLoopSlot.kSlot0).d(0, ClosedLoopSlot.kSlot0);
		canTiltMax = new SparkMax(CanConstants.scoreTiltMotor, MotorType.kBrushed);
		canTiltMax.configure(tiltConfig, null, null);
		canTiltMax.configure(tiltConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
		canTiltController = canTiltMax.getClosedLoopController();
		homingTiltClickySwitch = new DigitalInput(DIOConstants.homingTiltClickySwitch);
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("scoring/tilt current", canTiltMax.getOutputCurrent());
		SmartDashboard.putNumber("scoring/tilt voltage", canTiltMax.getBusVoltage() * canTiltMax.getAppliedOutput());
		SmartDashboard.putNumber("scoring/tilt encoder", canTiltMax.getEncoder().getPosition());
		SmartDashboard.putBoolean("scoring/coral sensor", coralSensor.get());
		SmartDashboard.putNumber("scoring/motor1 current", motor1.getOutputCurrent());
		SmartDashboard.putNumber("scoring/motor1 voltage", motor1.getBusVoltage() * motor1.getAppliedOutput());
		SmartDashboard.putNumber("scoring/motor2 current", motor2.getOutputCurrent());
		SmartDashboard.putNumber("scoring/motor2 voltage", motor2.getBusVoltage() * motor2.getAppliedOutput());
	}

	public Command runIntakeCommand() {
		return this.run(() -> {
				System.out.println("intake command started");
				spinnyController.setReference(20, ControlType.kCurrent);
				spinnyController2.setReference(20, ControlType.kCurrent);
			})
			.finallyDo(() -> {
				System.out.println("intake command finished");
				spinnyController.setReference(0, ControlType.kCurrent);
				spinnyController2.setReference(0, ControlType.kCurrent);
			})
			// .until(() -> !coralSensor.get())
			.withTimeout(3);
	}

	public Command runEjectCommand() {
		return this.run(() -> {
				System.out.println("eject command started");
				spinnyController.setReference(-20, ControlType.kCurrent);
				spinnyController2.setReference(-20, ControlType.kCurrent);
			})
			.finallyDo(() -> {
				System.out.println("eject command ended");
				spinnyController.setReference(0, ControlType.kCurrent);
				spinnyController2.setReference(0, ControlType.kCurrent);
			})
			// .until(() -> coralSensor.get())
			.withTimeout(3);
	}

	public Command tiltCommand(double position) {
		return this.runEnd(
				() -> {
					canTiltController.setReference(position, ControlType.kPosition);
				},
				() -> {
					canTiltController.setReference(0, ControlType.kPosition);
				}
			);
	}

	public Command homeCommand() {
		return this.runEnd(
				() -> {
					canTiltController.setReference(0.45, ControlType.kDutyCycle);
				},
				() -> {
					canTiltMax.getEncoder().setPosition(0);
					canTiltController.setReference(0, ControlType.kPosition);
				}
			).until(() -> homingTiltClickySwitch.get());
	}
}

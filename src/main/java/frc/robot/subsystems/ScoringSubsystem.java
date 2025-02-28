package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.EncoderConfig;
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
	private SparkMax canTiltMax;
	private SparkMaxConfig tiltConfig = new SparkMaxConfig();
	private EncoderConfig tiltEncoderConfig = new EncoderConfig();
	private SparkMaxConfig motorConfig = new SparkMaxConfig();
	private SparkClosedLoopController canTiltController;
	private DigitalInput homingTiltClickySwitch;
	public DigitalInput coralSensor;

	public ScoringSubsystem() {
		motorConfig.smartCurrentLimit(10);
		motor1 = new SparkMax(CanConstants.scoreMotor1, MotorType.kBrushless);
		motor2 = new SparkMax(CanConstants.scoreMotor2, MotorType.kBrushless);
		motor1.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
		motor2.configure(
			motorConfig.follow(motor1).inverted(true),
			ResetMode.kResetSafeParameters,
			PersistMode.kNoPersistParameters
		);

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
	}

	public Command runIntakeCommand() {
		return this.startEnd(
				() -> {
					motor1.set(1);
					motor2.set(1);
				},
				() -> {
					motor1.set(0);
					motor2.set(0);
				}
			)
			.until(() -> coralSensor.get() || motor1.getOutputCurrent() > 8)
			.withTimeout(3);
	}

	public Command runEjectCommand() {
		return this.startEnd(
				() -> {
					motor1.set(-1);
					motor2.set(-1);
				},
				() -> {
					motor1.set(0);
					motor2.set(0);
				}
			)
			.until(() -> !coralSensor.get())
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

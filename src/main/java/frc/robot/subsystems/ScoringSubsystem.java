package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanConstants;

public class ScoringSubsystem extends SubsystemBase {

	private SparkMax motor1;
	private SparkMax motor2;
	private SparkMax canTiltMax;
	private SparkMaxConfig config = new SparkMaxConfig();
	private SparkClosedLoopController canTiltController;
	private DigitalInput homingTiltClickySwitch;
	public DigitalInput coralSensor;

	public ScoringSubsystem() {
		motor1 = new SparkMax(CanConstants.scoreMotor1, MotorType.kBrushless);
		motor2 = new SparkMax(CanConstants.scoreMotor2, MotorType.kBrushless);
		coralSensor = new DigitalInput(CanConstants.coralSensorPort);

		canTiltMax = new SparkMax(CanConstants.scoreTiltMotor, MotorType.kBrushless);

		config.closedLoop.p(0.1, ClosedLoopSlot.kSlot0).i(0, ClosedLoopSlot.kSlot0).d(0, ClosedLoopSlot.kSlot0);
		canTiltMax.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
		canTiltController = canTiltMax.getClosedLoopController();

		homingTiltClickySwitch = new DigitalInput(CanConstants.homingTiltClickySwitch);
	}

	public Command runIntakeCommand(boolean out) {
		return this.startEnd(
				() -> {
					motor1.set(out ? 1 : -1);
					motor2.set(out ? 1 : -1);
				},
				() -> {
					motor1.set(0);
					motor2.set(0);
				}
			)
			.until(() -> coralSensor.get())
			.withTimeout(3);
	}

	public Command tiltCommand(double position) {
		return this.run(() -> {
				canTiltController.setReference(position, ControlType.kPosition);
			});
	}

	public Command homeCommand() {
		return this.runEnd(
				() -> {
					canTiltController.setReference(-0.5, ControlType.kDutyCycle);
				},
				() -> {
					canTiltMax.getEncoder().setPosition(0);
					canTiltController.setReference(0, ControlType.kPosition);
				}
			).until(() -> homingTiltClickySwitch.get());
	}
}

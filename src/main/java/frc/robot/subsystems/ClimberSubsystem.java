/* ----------
 * Copywrite 2025 FRC team 1317 under AGPL-3.0
 * ----------- */

package frc.robot.subsystems;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanConstants;

public class ClimberSubsystem extends SubsystemBase {

	private final TalonFX beefyMotor;
	private final Orchestra marioUnderwater;
	private final Orchestra duckOrchestra;

	public ClimberSubsystem() {
		beefyMotor = new TalonFX(CanConstants.beefyMotor);

		AudioConfigs audioConfigs = new AudioConfigs().withAllowMusicDurDisable(true);
		TalonFXConfiguration config = new TalonFXConfiguration().withAudio(audioConfigs);

		config.Slot0.kP = 0.11;
		config.Slot0.kI = 0;
		config.Slot0.kD = 0;
		beefyMotor.getConfigurator().apply(config);

		beefyMotor.setPosition(0);

		marioUnderwater = new Orchestra();
		marioUnderwater.addInstrument(beefyMotor);
		marioUnderwater.loadMusic("music/mario-underwater.chrp");
		marioUnderwater.play();

		duckOrchestra = new Orchestra();
		duckOrchestra.addInstrument(beefyMotor);
		duckOrchestra.loadMusic("music/duck.chrp");
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("climber/climber speed", beefyMotor.get());
		SmartDashboard.putNumber("climber/climber current", beefyMotor.getSupplyCurrent().getValueAsDouble());
		SmartDashboard.putNumber("climber/climber temp", beefyMotor.getDeviceTemp().getValueAsDouble());
	}

	public Command climbCommand() {
		return this.run(() -> {
				beefyMotor.setControl(new VelocityVoltage(100));
			})// ) // 		beefyMotor.getSupplyCurrent().getValue().compareTo(CURRENT_THRESHOLD) < 0 // 		beefyMotor.getPosition().getValue().compareTo(ROTATIONS_DOWN) > 0 && // 	() -> // .until(
			.andThen(() -> {
				beefyMotor.setControl(new VelocityVoltage(0));
			});
	}

	public Command descendCommand() {
		return run(() -> {
			beefyMotor.setControl(new VelocityVoltage(-100));
		})// ) // 		beefyMotor.getSupplyCurrent().getValue().compareTo(CURRENT_THRESHOLD) < 0 // 		beefyMotor.getPosition().getValue().compareTo(ROTATIONS_DOWN) > 0 && // 	() -> // .until(
		.andThen(() -> {
			beefyMotor.setControl(new VelocityVoltage(0));
		});
	}

	public Command playMusicCommand() {
		return runOnce(() -> duckOrchestra.play());
	}
}

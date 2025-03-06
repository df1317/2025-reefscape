/* ----------
 * Copywrite 2025 FRC team 1317 under AGPL-3.0
 * ----------- */

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanConstants;

public class ClimberSubsystem extends SubsystemBase {

	private final TalonFX beefyMotor;
	private final Orchestra marioUnderwater;
	private final Orchestra duckOrchestra;
	private static final Angle ROTATIONS_DOWN = Angle.ofBaseUnits(60, Degrees);
	private static final Current CURRENT_THRESHOLD = Current.ofBaseUnits(40, Amps);
	private final PositionVoltage positionVoltage = new PositionVoltage(0);

	public ClimberSubsystem() {
		beefyMotor = new TalonFX(CanConstants.beefyMotor);

		AudioConfigs audioConfigs = new AudioConfigs().withAllowMusicDurDisable(true);
		TalonFXConfiguration config = new TalonFXConfiguration().withAudio(audioConfigs);
		config.Slot0.kP = 0.11;
		config.Slot0.kI = 0.5;
		config.Slot0.kD = 0.0001;
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

	public Command climbCommand() {
		return run(() -> {
			Angle currentPosition = beefyMotor.getPosition().getValue();
			Current current = beefyMotor.getSupplyCurrent().getValue();

			if (currentPosition.compareTo(ROTATIONS_DOWN) > 0) {
				beefyMotor.setControl(positionVoltage.withPosition(-ROTATIONS_DOWN.baseUnitMagnitude())); // Go down
			} else if (current.compareTo(CURRENT_THRESHOLD) < 0) {
				beefyMotor.setControl(positionVoltage.withPosition(ROTATIONS_DOWN.baseUnitMagnitude())); // Go up
			} else {
				beefyMotor.setControl(positionVoltage.withPosition(currentPosition.baseUnitMagnitude())); // Hold position
			}
		});
	}

	public Command descendCommand() {
		return run(() -> {
			Angle currentPosition = beefyMotor.getPosition().getValue();
			Current current = beefyMotor.getSupplyCurrent().getValue();
			if (
				currentPosition.compareTo(Angle.ofBaseUnits(0, Degrees)) > 0 && current.compareTo(CURRENT_THRESHOLD) < 0
			) {
				beefyMotor.setControl(positionVoltage.withPosition(0)); // Go down
			} else {
				beefyMotor.setControl(positionVoltage.withPosition(currentPosition.baseUnitMagnitude())); // Hold position
			}
		});
	}

	public Command playMusicCommand() {
		return runOnce(() -> duckOrchestra.play());
	}
}

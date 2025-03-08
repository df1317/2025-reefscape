/* ----------
 * Copywrite 2025 FRC team 1317 under AGPL-3.0
 * ----------- */

package frc.robot.subsystems;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanConstants;
import java.util.function.DoubleSupplier;

public class ClimberSubsystem extends SubsystemBase {

	private final TalonFX beefyMotor;
	private final Orchestra marioUnderwater;
	private final Orchestra duckOrchestra;
	private final VoltageOut m_voltage = new VoltageOut(0).withEnableFOC(false);

	public ClimberSubsystem() {
		beefyMotor = new TalonFX(CanConstants.beefyMotor);

		HardwareLimitSwitchConfigs newHardwareLimitSwitch = new HardwareLimitSwitchConfigs()
			.withForwardLimitEnable(false)
			.withReverseLimitEnable(false)
			.withForwardLimitSource(ForwardLimitSourceValue.Disabled)
			.withReverseLimitSource(ReverseLimitSourceValue.Disabled);
		CurrentLimitsConfigs currentConfigs = new CurrentLimitsConfigs()
			.withStatorCurrentLimit(30)
			.withStatorCurrentLimitEnable(true);
		AudioConfigs audioConfigs = new AudioConfigs().withAllowMusicDurDisable(true);
		TalonFXConfiguration config = new TalonFXConfiguration()
			.withHardwareLimitSwitch(newHardwareLimitSwitch)
			.withAudio(audioConfigs)
			.withCurrentLimits(currentConfigs);

		config.Slot0.kP = 2;
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
		SmartDashboard.putNumber("climber/climber current", beefyMotor.getStatorCurrent().getValueAsDouble());
		SmartDashboard.putNumber("climber/climber temp", beefyMotor.getDeviceTemp().getValueAsDouble());
	}

	public Command climbCommand() {
		return run(() -> {
			beefyMotor.setControl(m_voltage.withOutput(-10).withEnableFOC(false));
		}).finallyDo(() -> {
			beefyMotor.setControl(m_voltage.withOutput(0).withEnableFOC(false));
		});
	}

	public Command descendCommand() {
		return run(() -> {
			beefyMotor.setControl(m_voltage.withOutput(10).withEnableFOC(false));
		}).finallyDo(() -> {
			beefyMotor.setControl(m_voltage.withOutput(0).withEnableFOC(false));
		});
	}

	public Command joyCommand(DoubleSupplier pos) {
		return run(() -> {
			beefyMotor.setControl(m_voltage.withOutput(pos.getAsDouble() * 10).withEnableFOC(false));
		}).finallyDo(() -> {
			beefyMotor.setControl(m_voltage.withOutput(0).withEnableFOC(false));
		});
	}

	public Command playMusicCommand() {
		return runOnce(() -> duckOrchestra.play());
	}
}

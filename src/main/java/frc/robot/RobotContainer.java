/* ----------
 * Copywrite 2025 FRC team 1317 under AGPL-3.0
 * ----------- */

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.libs.FieldConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ScoringSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;

/** ----------
 * RobotContainer Class
 * ---
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic
 * should actually be handled in the {@link Robot} periodic methods
 * (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and trigger mappings) should be declared here.
 * ---
 */
public class RobotContainer {

	/** ----------
	 * HID Initialization
	 * ------------ */
	private final CommandXboxController driverXbox = new CommandXboxController(0);
	private final CommandJoystick m_JoystickL = new CommandJoystick(1);
	private final CommandJoystick m_JoystickR = new CommandJoystick(2);

	/** ----------
	 * Subsystems
	 * ------------ */
	private final SwerveSubsystem drivebase = new SwerveSubsystem(
		new File(Filesystem.getDeployDirectory(), "swerve/neo")
	);
	private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
	private final ScoringSubsystem scoringSubsystem = new ScoringSubsystem();
	private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();

	/** ----------
	 * Swerve Drive Input Streams
	 * ------------ */

	/**
	 * Converts driver input into a field-relative ChassisSpeeds that is controlled
	 * by angular velocity.
	 */
	SwerveInputStream driveAngularVelocity = SwerveInputStream.of(
		drivebase.getSwerveDrive(),
		() -> driverXbox.getLeftY() * -1,
		() -> driverXbox.getLeftX() * -1
	)
		.withControllerRotationAxis(() -> driverXbox.getRightX() * -1)
		.deadband(OperatorConstants.DEADBAND)
		.scaleTranslation(0.8)
		.allianceRelativeControl(true);

	/**
	 * Clone's the angular velocity input stream and converts it to a fieldRelative
	 * input stream.
	 */
	SwerveInputStream driveDirectAngle = driveAngularVelocity
		.copy()
		.withControllerHeadingAxis(driverXbox::getRightX, driverXbox::getRightY)
		.headingWhile(true);

	/**
	 * Clone's the angular velocity input stream and converts it to a robotRelative
	 * input stream.
	 */
	SwerveInputStream driveRobotOriented = driveAngularVelocity
		.copy()
		.robotRelative(true)
		.allianceRelativeControl(false);

	SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(
		drivebase.getSwerveDrive(),
		() -> -driverXbox.getLeftY(),
		() -> -driverXbox.getLeftX()
	)
		.withControllerRotationAxis(() -> driverXbox.getRawAxis(2))
		.deadband(OperatorConstants.DEADBAND)
		.scaleTranslation(0.8)
		.allianceRelativeControl(true);
	// Derive the heading axis with math!
	SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard
		.copy()
		.withControllerHeadingAxis(
			() -> Math.sin(driverXbox.getRawAxis(2) * Math.PI) * (Math.PI * 2),
			() -> Math.cos(driverXbox.getRawAxis(2) * Math.PI) * (Math.PI * 2)
		)
		.headingWhile(true);

	/** ----------
	 * RobotContainer Root Class
	 * ---
	 * This is the root class for the robot. It is responsible for configuring the robot, its subsystems, and bindings.
	 * ---
	 */
	public RobotContainer() {
		configureBindings();
		DriverStation.silenceJoystickConnectionWarning(true);
	}

	/** ----------
	 * Configure the button bindings
	 * ---
	 * Use this method to define your button->command mappings. Buttons can be created by
	 * instantiating a {@link CommandButton} with a {@link Command} and then calling the
	 * various button-press functions on it.
	 * ---
	 */
	private void configureBindings() {
		Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

		drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

		/** ------------------------------------- *
		 * Xbox Swerve and Navigation bindings
		 * ---
		 * drivebase locking and fake vision reading
		 * zero gyro and print mode
		 * center modules and none
		 * zero elevator and none
		 * drive to pose (for sysid) and none
		 * ---
		 */

		driverXbox
			.x()
			.whileTrue(
				Commands.either(
					Commands.runOnce(drivebase::lock, drivebase).repeatedly(),
					Commands.runOnce(drivebase::addFakeVisionReading),
					DriverStation::isTest
				)
			);

		driverXbox
			.start()
			.onTrue(
				Commands.runOnce(drivebase::zeroGyro).andThen(
					Commands.runOnce(() ->
						System.out.println(DriverStation.isTest() ? "Test Mode: Reset Gyro" : "Other Mode: Reset Gyro")
					)
				)
			);

		driverXbox
			.back()
			.whileTrue(Commands.either(drivebase.centerModulesCommand(), Commands.none(), DriverStation::isTest));

		driverXbox
			.b()
			.whileTrue(Commands.either(elevatorSubsystem.zeroCommand(), Commands.none(), DriverStation::isTest));

		driverXbox
			.y()
			.onTrue(
				Commands.either(drivebase.driveToDistanceCommand(1.0, 0.2), Commands.none(), DriverStation::isTest)
			);

		/** -------------------------------------
		 * Xbox Scoring and Intake bindings
		 * ---
		 * duck song and drivebase locking
		 * intake and duck song
		 * eject and none
		 * ---
		 */

		driverXbox
			.leftBumper()
			.whileTrue(
				Commands.either(
					climberSubsystem.playMusicCommand(),
					Commands.runOnce(drivebase::lock, drivebase).repeatedly(),
					DriverStation::isTest
				)
			);

		driverXbox
			.rightBumper()
			.onTrue(
				Commands.either(
					scoringSubsystem.runIntakeCommand(),
					climberSubsystem.playMusicCommand(),
					DriverStation::isTest
				)
			);

		driverXbox
			.rightTrigger()
			.onTrue(Commands.either(scoringSubsystem.runEjectCommand(), Commands.none(), DriverStation::isTest));

		/** -------------------------------------
		 * Elevator position bindings
		 * ---
		 * positions are staggered so it is in an increasing u shape on the joystick
		 * demo command will be toggled via binding and will continuially run elevator positions
		 * manual control sets elevator position and speed based on joystick position
		 * ---
		 */

		m_JoystickL
			.trigger()
			.whileTrue(
				Commands.either(
					elevatorSubsystem.setSpeed(() -> m_JoystickL.getY() * -1),
					Commands.none(),
					DriverStation::isTest
				)
			);

		m_JoystickL
			.button(5)
			.onTrue(
				elevatorSubsystem
					.setPos(() -> FieldConstants.CoralStation.height)
					.alongWith(scoringSubsystem.tiltCommand(FieldConstants.CoralStation.pitch))
			);
		m_JoystickL
			.button(3)
			.onTrue(
				elevatorSubsystem
					.setPos(() -> FieldConstants.ReefHeight.L2.height)
					.alongWith(scoringSubsystem.tiltCommand(FieldConstants.ReefHeight.L2.pitch))
			);
		m_JoystickL
			.button(4)
			.onTrue(
				elevatorSubsystem
					.setPos(() -> FieldConstants.ReefHeight.L3.height)
					.alongWith(scoringSubsystem.tiltCommand(FieldConstants.ReefHeight.L3.pitch))
			);
		m_JoystickL
			.button(6)
			.onTrue(
				(elevatorSubsystem
						.setPos(() -> FieldConstants.ReefHeight.L4.height)
						.alongWith(scoringSubsystem.tiltCommand(FieldConstants.ReefHeight.L4.pitch))).andThen(
						scoringSubsystem.tiltCommand(144)
					)
			);

		m_JoystickL
			.button(7)
			.toggleOnTrue(
				Commands.either(
					Commands.repeatingSequence(
						elevatorSubsystem
							.setPos(() -> FieldConstants.CoralStation.height)
							.alongWith(scoringSubsystem.tiltCommand(FieldConstants.CoralStation.pitch)),
						Commands.waitSeconds(2),
						elevatorSubsystem
							.setPos(() -> FieldConstants.ReefHeight.L2.height)
							.alongWith(scoringSubsystem.tiltCommand(FieldConstants.ReefHeight.L2.pitch)),
						Commands.waitSeconds(2),
						elevatorSubsystem
							.setPos(() -> FieldConstants.ReefHeight.L3.height)
							.alongWith(scoringSubsystem.tiltCommand(FieldConstants.ReefHeight.L3.pitch)),
						Commands.waitSeconds(2),
						elevatorSubsystem
							.setPos(() -> FieldConstants.ReefHeight.L4.height)
							.alongWith(scoringSubsystem.tiltCommand(FieldConstants.ReefHeight.L4.pitch))
							.andThen(scoringSubsystem.tiltCommand(144)),
						Commands.waitSeconds(2)
					),
					Commands.none(),
					DriverStation::isTest
				)
			);

		/** -------------------------------------
		 * Test-mode specific climber bindings
		 * ---
		 * climb and descend
		 * ---
		 */

		m_JoystickL
			.button(8)
			.whileTrue(Commands.either(climberSubsystem.climbCommand(), Commands.none(), DriverStation::isTest));
		m_JoystickL
			.button(9)
			.whileTrue(Commands.either(climberSubsystem.descendCommand(), Commands.none(), DriverStation::isTest));

		/** -------------------------------------
		 * Test-mode specific tilt bindings
		 * ---
		 * tilt command and tilt nudge
		 * tilt sysid command
		 * ---
		 */

		m_JoystickL
			.button(10)
			.whileTrue(Commands.either(scoringSubsystem.tiltCommand(0), Commands.none(), DriverStation::isTest));
		m_JoystickL
			.button(11)
			.whileTrue(Commands.either(scoringSubsystem.tiltNudge(false), Commands.none(), DriverStation::isTest));
		m_JoystickL
			.button(12)
			.whileTrue(Commands.either(scoringSubsystem.tiltNudge(true), Commands.none(), DriverStation::isTest));

		/** -------------------------------------
		 * Test-mode specific sysid bindings
		 * ---
		 * sysid command for elevator, tilt, and drivebase
		 * ---
		 */

		m_JoystickL
			.button(2)
			.onTrue(
				Commands.either(scoringSubsystem.tiltSysIDCommand(4, 2, 2), Commands.none(), DriverStation::isTest)
			);

		m_JoystickR
			.button(9)
			.whileTrue(
				Commands.either(elevatorSubsystem.sysIDCommand(4, 2, 2), Commands.none(), DriverStation::isTest)
			);

		m_JoystickR
			.button(10)
			.whileTrue(
				Commands.either(scoringSubsystem.tiltSysIDCommand(4, 2, 2), Commands.none(), DriverStation::isTest)
			);
		m_JoystickR
			.button(11)
			.whileTrue(Commands.either(drivebase.sysIdAngleMotorCommand(), Commands.none(), DriverStation::isTest));
		m_JoystickR
			.button(12)
			.whileTrue(Commands.either(drivebase.sysIdDriveMotorCommand(), Commands.none(), DriverStation::isTest));
	}

	/** ----------
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 * ---
	 * @return the command to run in autonomous
	 * ---
	 */
	public Command getAutonomousCommand() {
		// An example command will be run in autonomous
		return drivebase.getAutonomousCommand("New Auto");
	}

	public void setMotorBrake(boolean brake) {
		drivebase.setMotorBrake(brake);
	}
}

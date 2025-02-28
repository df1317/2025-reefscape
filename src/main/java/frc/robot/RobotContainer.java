// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ScoringSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
public class RobotContainer {

	// Replace with CommandPS4Controller or CommandJoystick if needed
	private final CommandXboxController driverXbox = new CommandXboxController(0);
	private final CommandJoystick m_JoystickL = new CommandJoystick(1);
	// private final CommandJoystick m_JoystickR = new CommandJoystick(2);
	// The robot's subsystems and commands are defined here...
	private final SwerveSubsystem drivebase = new SwerveSubsystem(
		new File(Filesystem.getDeployDirectory(), "swerve/neo")
	);
	private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
	private final ScoringSubsystem scoringSubsystem = new ScoringSubsystem();

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

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		// Configure the trigger bindings
		configureBindings();
		DriverStation.silenceJoystickConnectionWarning(true);
		NamedCommands.registerCommand("test", Commands.print("I EXIST"));
	}

	private void configureBindings() {
		Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

		drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

		if (DriverStation.isTest()) {
			drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command
			// above!

			driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
			driverXbox.y().onTrue(drivebase.driveToDistanceCommand(1.0, (double) 0.2));
			driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
			driverXbox.back().whileTrue(drivebase.centerModulesCommand());
			driverXbox.leftBumper().onTrue(Commands.none());
			driverXbox.rightBumper().onTrue(Commands.none());
			// driverXbox.a().onTrue(Commands.runOnce(() -> System.out.println("Test Mode:
			// Drive SysID")));
			// driverXbox.a().whileTrue(drivebase.sysIdDriveMotorCommand());
			driverXbox.a().onTrue(Commands.runOnce(() -> System.out.println("test Mode: Reset Gyro")));
			driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));

			m_JoystickL.trigger().whileTrue(elevatorSubsystem.setSpeed(() -> m_JoystickL.getY() * -1));
			m_JoystickL.button(5).onTrue(elevatorSubsystem.setPos(() -> 0));
			m_JoystickL.button(3).onTrue(elevatorSubsystem.setPos(() -> 0.3));
			m_JoystickL.button(4).onTrue(elevatorSubsystem.setPos(() -> 0.6));
			m_JoystickL.button(6).onTrue(elevatorSubsystem.setPos(() -> 1.2));
			m_JoystickL.button(2).onTrue(elevatorSubsystem.sysIDCommand(4, 2, 2));
			m_JoystickL.button(7).toggleOnTrue(elevatorSubsystem.demo());
			// m_JoystickL.button(11)
			// .whileTrue(elevatorSubsystem.sysIDQuasistatic(Direction.kReverse, 3.0));
			// m_JoystickL.button(9).whileTrue(elevatorSubsystem.sysIDQuasistatic(Direction.kForward,
			// 3.0));

			// m_JoystickL.button(12)
			// .whileTrue(elevatorSubsystem.sysIDDynamic(Direction.kReverse, 1.0));
			// m_JoystickL.button(10).whileTrue(elevatorSubsystem.sysIDDynamic(Direction.kForward,
			// 0.5));

			m_JoystickL.button(11).whileTrue(scoringSubsystem.tiltCommand(0.5));
		} else {
			driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
			driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
			driverXbox
				.b()
				.whileTrue(drivebase.driveToPose(new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0))));
			driverXbox.start().whileTrue(Commands.none());
			driverXbox.back().whileTrue(Commands.none());
			driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
			driverXbox.rightBumper().onTrue(Commands.none());
			driverXbox.a().onTrue(Commands.runOnce(() -> System.out.println("Other Mode: Reset Gyro")));
		}
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		// An example command will be run in autonomous
		return drivebase.getAutonomousCommand("New Auto");
	}

	public void setMotorBrake(boolean brake) {
		drivebase.setMotorBrake(brake);
	}
}

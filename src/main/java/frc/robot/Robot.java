/* ----------
 * Copywrite 2025 FRC team 1317 under AGPL-3.0
 * ----------- */

package frc.robot;

import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/** ----------
 * Robot
 * ---
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the TimedRobot documentation. If you change the name of this class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 * ---
 */
public class Robot extends TimedRobot {

	private static Robot instance;
	private Command m_autonomousCommand;

	private RobotContainer m_robotContainer;

	private Timer disabledTimer;

	public Robot() {
		instance = this;
	}

	public static Robot getInstance() {
		return instance;
	}

	/** ----------
	 * RobotInit
	 * ---
	 * This function is run when the robot is first started up and should be used for any initialization code.
	 * ---
	 */
	@Override
	public void robotInit() {
		// Instantiate our RobotContainer.  This will perform all our button bindings, and put our
		// autonomous chooser on the dashboard.
		m_robotContainer = new RobotContainer();

		// create webserve for elastic dashboard layouts
		WebServer.start(5800, Filesystem.getDeployDirectory().getPath());

		// Create a timer to disable motor brake a few seconds after disable.  This will let the robot stop
		// immediately when disabled, but then also let it be pushed more
		disabledTimer = new Timer();

		if (isSimulation()) {
			DriverStation.silenceJoystickConnectionWarning(true);
		}
	}

	/** ----------
	 * RobotPeriodic
	 * ---
	 * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics that you want ran
	 * during disabled, autonomous, teleoperated and test.
	 *
	 * <p>This runs after the mode specific periodic functions, but before LiveWindow and
	 * SmartDashboard integrated updating.
	 *---
	 */
	@Override
	public void robotPeriodic() {
		/* ----------
		 * Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
		 * commands, running already-scheduled commands, removing finished or interrupted commands,
		 * and running subsystem periodic() methods.  This must be called from the robot's periodic
		 * block in order for anything in the Command-based framework to work.
		 */
		CommandScheduler.getInstance().run();

		// Put the current match time on the dashboard
		SmartDashboard.putNumber("misc/Match Time", DriverStation.getMatchTime());
	}

	/* ----------
	 * DisabledInit
	 * ---
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when the robot is disabled.
	 * ---
	 *  */
	@Override
	public void disabledInit() {
		m_robotContainer.setMotorBrake(true);
		disabledTimer.reset();
		disabledTimer.start();
	}

	/** ----------
	 * DisabledPeriodic
	 * ---
	 * This function is called periodically while the robot is in disabled mode.
	 * ---
	 */
	@Override
	public void disabledPeriodic() {
		if (disabledTimer.hasElapsed(Constants.DrivebaseConstants.WHEEL_LOCK_TIME)) {
			m_robotContainer.setMotorBrake(false);
			disabledTimer.stop();
			disabledTimer.reset();
		}
	}

	/** ----------
	 * AutonomousInit
	 * ---
	 * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
	 * ---
	 */
	@Override
	public void autonomousInit() {
		m_robotContainer.setMotorBrake(true);
		m_autonomousCommand = m_robotContainer.getAutonomousCommand();

		// schedule the autonomous command (example)
		if (m_autonomousCommand != null) {
			m_autonomousCommand.schedule();
		}
	}

	/** ----------
	 * TeleopInit
	 * ---
	 * This function is called at the beginning of operator control.
	 * ---
	 */
	@Override
	public void teleopInit() {
		/* ----------
		 * This makes sure that the autonomous stops running when
		 * teleop starts running. If you want the autonomous to
		 * continue until interrupted by another command, remove
		 * this line or comment it out.
		 */
		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		} else {
			CommandScheduler.getInstance().cancelAll();
		}
	}

	/** ----------
	 * TestInit
	 * ---
	 * This function is called periodically during test mode.
	 * ---
	 */
	@Override
	public void testInit() {
		// Cancels all running commands at the start of test mode.
		CommandScheduler.getInstance().cancelAll();
	}
}

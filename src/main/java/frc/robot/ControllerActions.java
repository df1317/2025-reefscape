package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ControllerActions {

    public final CommandXboxController m_XboxController = new CommandXboxController(0);
    public final CommandJoystick m_JoystickL = new CommandJoystick(1);
    public final CommandJoystick m_JoystickR = new CommandJoystick(2);
    private final double elevatorDeadzone = 0.1;

    public final Trigger scoreButton = new Trigger(
            m_JoystickL.button(1));
    public final Trigger intakeButton = new Trigger(
            m_JoystickR.button(1));
    public final Trigger manualElevatorButton = new Trigger(m_JoystickR.button(2));
    public final Trigger elevator0 = new Trigger(m_JoystickL.button(5).or(m_JoystickR.button(5)));
    public final Trigger elevator1 = new Trigger(m_JoystickL.button(3).or(m_JoystickR.button(3)));
    public final Trigger elevator2 = new Trigger(m_JoystickL.button(4).or(m_JoystickR.button(4)));
    public final Trigger elevator3 = new Trigger(m_JoystickL.button(6).or(m_JoystickR.button(6)));

    public double manualElevatorInput() {
        double pos = -m_JoystickR.getRawAxis(0);
        if (Math.abs(pos) < elevatorDeadzone) {
            return 0;
        } else {
            return pos;
        }
    }
}

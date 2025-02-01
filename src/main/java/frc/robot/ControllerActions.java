package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ControllerActions {

    public final CommandXboxController m_XboxController =
        new CommandXboxController(0);
    public final CommandJoystick m_JoystickL = new CommandJoystick(1);
    public final CommandJoystick m_JoystickR = new CommandJoystick(2);

    public final Trigger scoreButton = new Trigger(
        m_JoystickL.button(1).or(m_JoystickR.button(1))
    );
    public final Trigger intakeButton = new Trigger(
        m_JoystickL.button(3).or(m_JoystickR.button(3))
    );
}

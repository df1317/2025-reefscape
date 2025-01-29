package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ScoringSubsystem;

public class ScoringCommand extends Command {

    private ScoringSubsystem m_ScoringSubsystem;
    public boolean intake;
    private Timer outtakeTimer;

    public ScoringCommand(ScoringSubsystem scoringSub, boolean intake) {
        m_ScoringSubsystem = scoringSub;
        addRequirements(scoringSub);
        this.intake = intake;
        outtakeTimer = new Timer();
    }

    @Override
    public void initialize() {
        if (intake) {
            m_ScoringSubsystem.runIntake();
        } else {
            m_ScoringSubsystem.runOuttake();
            outtakeTimer.start();
        }
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        if (intake) {
            return m_ScoringSubsystem.coralSensor.get();
        } else {
            return outtakeTimer.hasElapsed(6);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_ScoringSubsystem.stopMotors();
        outtakeTimer.reset();
    }
}

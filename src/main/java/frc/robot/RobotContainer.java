// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.ScoringCommand;
import frc.robot.subsystems.ScoringSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here...

  private final ScoringSubsystem m_ScoringSubsystem = new ScoringSubsystem();
  private final ControllerActions m_ControllerActions = new ControllerActions();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
    ScoringCommand score = new ScoringCommand(m_ScoringSubsystem, false);
    m_ControllerActions.scoreButton.onTrue(score);

    ScoringCommand intake = new ScoringCommand(m_ScoringSubsystem, true);
    m_ControllerActions.intakeButton.onTrue(intake);
  }
}

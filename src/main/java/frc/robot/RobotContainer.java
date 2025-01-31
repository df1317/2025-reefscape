// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.ScoringSubsystem;
import frc.robot.commands.ScoringCommand;

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
  private final Buttons m_Buttons = new Buttons();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.

    ScoringCommand score = new ScoringCommand(m_ScoringSubsystem, false);
    m_Buttons.scoreButtonL.or(m_Buttons.scoreButtonR).onTrue(score);

    ScoringCommand intake = new ScoringCommand(m_ScoringSubsystem, true);
    m_Buttons.intakeButtonL.or(m_Buttons.intakeButtonR).onTrue(intake);
  }

}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.XRPDrivetrain;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class DriveCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final XRPDrivetrain m_drivetrain;
  private final CommandXboxController m_driverController;

  /**
   * Creates a new DriveCommand.
   *
   * @param drivetrain The drivetrain used by this command.
   * @param controller The xbox controller used by this command
   */
  public DriveCommand(XRPDrivetrain drivetrain, CommandXboxController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = drivetrain;
    m_driverController = controller;
    addRequirements(m_drivetrain);
  }

  @Override
  public void initialize() {
    // Called when the command is initially scheduled.
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.arcadeDrive(-m_driverController.getLeftY(), -m_driverController.getRightX());
  }


  @Override
  public void end(boolean interrupted) {
    // Called once the command ends or is interrupted.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

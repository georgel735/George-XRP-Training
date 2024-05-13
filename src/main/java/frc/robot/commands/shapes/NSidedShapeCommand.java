package frc.robot.commands.shapes;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.XRPDrivetrain;

//TODO: This command should have the drivetrain move in a triangle fashion
public class NSidedShapeCommand extends Command {
  private final XRPDrivetrain m_drivetrain;
  private final double m_sides;

  // use this boolean to tell the command when it should finish
  private boolean isFinished = false;

  public NSidedShapeCommand(XRPDrivetrain drivetrain, double sides) {
    m_drivetrain = drivetrain;
    m_sides = sides;
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(m_drivetrain);
  }

  @Override
  public void execute() {
    // Repeat the same motion once for each side
    for (int i = 0; i < /* insert something here */; i++) {

    }

    // Stop command
    isFinished = true;
  }

  @Override
  public boolean isFinished() {
    return isFinished;
  }

  @Override
  public void end(boolean interrupted) {
    m_drivetrain.arcadeDrive(0, 0);
  }
}

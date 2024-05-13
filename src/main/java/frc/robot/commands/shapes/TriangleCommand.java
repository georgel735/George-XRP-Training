package frc.robot.commands.shapes;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.XRPDrivetrain;

//TODO: This command should have the drivetrain move in a triangle fashion
public class TriangleCommand extends Command {
  private final XRPDrivetrain m_drivetrain;

  // use this boolean to tell the command when it should finish
  private boolean isFinished = false;

  public TriangleCommand(XRPDrivetrain drivetrain) {
    m_drivetrain = drivetrain;
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(m_drivetrain);
  }

  @Override
  public void execute() {
    // Repeat the same motions 3 times (for a triangle
    for (int i = 0; i < 3; i++) {
      // drive at half speed for 1 second
      m_drivetrain.driveForTime(0.5, 0.0, 1);

      // turn 120 degrees (360 / 3 = 120)
      m_drivetrain.rotateDegrees(120);
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

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.XRPDrivetrain;

public class SimpleAutoCommand extends SequentialCommandGroup {
    public SimpleAutoCommand(XRPDrivetrain drivetrain) {
        addCommands(new DriveForTimeCommand(drivetrain, 0.5, 1.0, 0.5),
        new DriveForTimeCommand(drivetrain, 0.5, 1.0, -0.5), new DriveForTimeCommand(drivetrain,0.75, 1.0, 25.0),
                new DriveForTimeCommand(drivetrain, 1.0, 1.0, 0.0));
    }
}

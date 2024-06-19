package frc.robot.commands;

import edu.wpi.first.wpilibj.xrp.XRPServo;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.XRPArmSlaysystem;


public class heeheehawhawCommand extends Command {
    private final XRPArmSlaysystem m_xrpArmSlaysystem;
    private final double m_angle;

    public heeheehawhawCommand(XRPArmSlaysystem xrpArmSlaysystem, double angle) {
        m_xrpArmSlaysystem = xrpArmSlaysystem;
        m_angle = angle;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(m_xrpArmSlaysystem);
    }

    @Override
    public void initialize() {
        m_xrpArmSlaysystem.setServoPosition(m_angle);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}

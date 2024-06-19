package frc.robot.subsystems;

import edu.wpi.first.wpilibj.xrp.XRPServo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class XRPArmSlaysystem extends SubsystemBase {
    private final XRPServo m_XRPServo = new XRPServo(4);
    public XRPArmSlaysystem() {}

    public void setServoPosition(double angle) {
        m_XRPServo.setAngle(angle);
    }

    public Command setServoPositionFactory(double angle) {
        return runOnce(() -> setServoPosition(angle));
    }
}

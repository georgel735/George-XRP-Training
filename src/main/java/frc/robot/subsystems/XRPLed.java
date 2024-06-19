package frc.robot.subsystems;

import edu.wpi.first.wpilibj.xrp.XRPOnBoardIO;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class XRPLed extends SubsystemBase {
    private final XRPOnBoardIO m_XRPLed = new XRPOnBoardIO();
    public XRPLed() {
        m_XRPLed.setLed(false);
    }

    public void setLedOn() {
        m_XRPLed.setLed(true);
    }
    public void setLedOff() {
        m_XRPLed.setLed(false);
    }
}

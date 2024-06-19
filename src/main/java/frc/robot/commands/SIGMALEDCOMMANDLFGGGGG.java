package frc.robot.commands;

import edu.wpi.first.wpilibj.xrp.XRPOnBoardIO;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.XRPLed;

public class SIGMALEDCOMMANDLFGGGGG extends Command {
    private final XRPLed m_led;
    private final boolean m_isOn;
    public SIGMALEDCOMMANDLFGGGGG(XRPLed led, boolean isOn) {
        m_led = led;
        m_isOn = isOn;
        addRequirements(m_led);
    }

    @Override
    public void initialize() {
        if (m_isOn) {
            m_led.setLedOn();
        } else {
            m_led.setLedOff();
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}

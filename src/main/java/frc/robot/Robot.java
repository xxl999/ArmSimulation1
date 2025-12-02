package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;

public class Robot extends TimedRobot {
    private final ArmSimulation m_arm = new ArmSimulation();
    private final Joystick m_joystick = new Joystick(Constants.kJoystickPort);

    public Robot() {}

    @Override
    public void simulationPeriodic() {
        m_arm.simulationPeriodic();
    }

    @Override
    public void teleopInit() {
        m_arm.loadPreferences();
    }

    @Override
    public void teleopPeriodic() {
        if (m_joystick.getTrigger()) {
            m_arm.reachSetpoint();
        } 
        else {
            m_arm.stop();
        }
    }

    @Override
    public void close() {
        m_arm.close();
        super.close();
    }

    @Override
    public void disabledInit() {
        m_arm.stop();
    }
}
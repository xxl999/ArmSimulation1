package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class ArmSimulation implements AutoCloseable {
    private double m_armKp = Constants.kDefaultArmKp;
    private double m_armSetpointDegrees = Constants.kDefaultArmSetpointDegrees;
    
    private final DCMotor m_armGearbox = DCMotor.getFalcon500(1);
    private final TalonFX m_motor = new TalonFX(Constants.kMotorPort);
    private final TalonFXSimState m_motorSim = m_motor.getSimState();
    
    // Falcon 500'ün kendi PID kontrolcüsünü kullan
    private final PositionVoltage m_positionControl = new PositionVoltage(0);
    private final VoltageOut m_voltageControl = new VoltageOut(0);
    
    private final SingleJointedArmSim m_armSim = new SingleJointedArmSim(
        m_armGearbox,
        Constants.kArmReduction,
        SingleJointedArmSim.estimateMOI(Constants.kArmLength, Constants.kArmMass),
        Constants.kArmLength,
        Constants.kMinAngleRads,
        Constants.kMaxAngleRads,
        true,
        0,
        Constants.kArmEncoderDistPerPulse,
        0.0
    );
    
    private final Mechanism2d m_mech2d = new Mechanism2d(60, 60);
    private final MechanismRoot2d m_armPivot = m_mech2d.getRoot("ArmPivot", 30, 30);
    private final MechanismLigament2d m_armTower = 
        m_armPivot.append(new MechanismLigament2d("ArmTower", 30, -90));
    private final MechanismLigament2d m_arm = 
        m_armPivot.append(
            new MechanismLigament2d(
                "Arm",
                20,
                Units.radiansToDegrees(m_armSim.getAngleRads()),
                6,
                new Color8Bit(Color.kBlue)));

    public ArmSimulation() {
        SmartDashboard.putData("Arm Sim", m_mech2d);
        m_armTower.setColor(new Color8Bit(Color.kBlue));
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        Slot0Configs slot0 = config.Slot0;
        slot0.kP = m_armKp;
        slot0.kI = 0.0;
        slot0.kD = 0.0;
        
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        m_motor.getConfigurator().apply(config);
        
        Preferences.initDouble(Constants.kArmPositionKey, m_armSetpointDegrees);
        Preferences.initDouble(Constants.kArmPKey, m_armKp);
    }

    /** Update the simulation model. */
    public void simulationPeriodic() {
        m_armSim.setInput(m_motorSim.getMotorVoltage());
        m_armSim.update(0.020);
        
        double positionRotations = m_armSim.getAngleRads() / (2 * Math.PI);
        m_motorSim.setRawRotorPosition(positionRotations * Constants.kArmReduction);
        m_motorSim.setRotorVelocity(m_armSim.getVelocityRadPerSec() / (2 * Math.PI));
        
        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(m_armSim.getCurrentDrawAmps()));
        
        m_arm.setAngle(Units.radiansToDegrees(m_armSim.getAngleRads()));
    }

    public void loadPreferences() {
        m_armSetpointDegrees = Preferences.getDouble(
            Constants.kArmPositionKey, m_armSetpointDegrees);
        
        if (m_armKp != Preferences.getDouble(Constants.kArmPKey, m_armKp)) {
            m_armKp = Preferences.getDouble(Constants.kArmPKey, m_armKp);
            
            Slot0Configs slot0 = new Slot0Configs();
            slot0.kP = m_armKp;
            slot0.kI = 0.0;
            slot0.kD = 0.0;
            m_motor.getConfigurator().apply(slot0);
        }
    }

    public void reachSetpoint() {
        double targetRads = Units.degreesToRadians(m_armSetpointDegrees);
        double targetRotations = targetRads / (2 * Math.PI) * Constants.kArmReduction;
        
        m_motor.setControl(m_positionControl.withPosition(targetRotations));
    }

    public void stop() {
        m_motor.setControl(m_voltageControl.withOutput(0.0));
    }

    @Override
    public void close() {
        m_mech2d.close();
        m_armPivot.close();
        m_arm.close();
        m_motor.close();
    }
}
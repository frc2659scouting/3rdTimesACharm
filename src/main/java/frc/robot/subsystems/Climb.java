package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climb extends SubsystemBase {
    
    private static final TalonFX m_telescopeMotor1 = new TalonFX(Constants.CLIMB_1);
    private static final TalonFX m_telescopeMotor2 = new TalonFX(Constants.CLIMB_2);
 
    public void setTele1(double pos) {
    //    m_telescopeMotor1.set(ControlMode.Position, calcRotats(pos));
        m_telescopeMotor1.set(ControlMode.Position, pos);
    }

    public void setTele2(double pos) {
    //    m_telescopeMotor2.set(ControlMode.Position, calcRotats(pos));
        m_telescopeMotor2.set(ControlMode.Position, pos);
    }
    
    public double getTele1Pos() {
        return m_telescopeMotor1.getSelectedSensorPosition();
    }

    public double getTele2Pos() {
        return m_telescopeMotor2.getSelectedSensorPosition();
    }

    public boolean tele1Extended() {
        return m_telescopeMotor1.getSelectedSensorPosition() > calcRotats(Constants.EXTENDED_CLIMB_POSITION);
    }

    public boolean tele2Extended() {
        return m_telescopeMotor2.getSelectedSensorPosition() > calcRotats(Constants.EXTENDED_CLIMB_POSITION);
    }

    private double calcRotats(double pos) {
        return pos * Constants.TELESCOPE_GEAR_RATIO;
    }
}

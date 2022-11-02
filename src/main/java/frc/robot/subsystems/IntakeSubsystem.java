package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

 public class IntakeSubsystem extends SubsystemBase{ 

    private static final Solenoid m_intakeSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
    private static final TalonFX m_intakeMotor = new TalonFX(Constants.INTAKE_MOTOR);

    public void extendIntake() {
        m_intakeSolenoid.set(true);
    }

    public void setIntake(double power) {
        extendIntake();
        m_intakeMotor.set(ControlMode.PercentOutput, power);
    }

    public void stopIntake() {
        m_intakeMotor.set(ControlMode.PercentOutput, 0.0);
    }

    public void retractIntake() {
        m_intakeSolenoid.set(false);
        m_intakeMotor.set(ControlMode.PercentOutput, 0.0);
    }

    @Override
    public void periodic() {
    }
}
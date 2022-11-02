package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;

public class HopperSubsystem extends SubsystemBase{
    
    private static final TalonFX m_tower = new TalonFX(Constants.HOPPER_MOTOR);
    private static final TalonFX m_centering = new TalonFX(Constants.BALL_CENTERING_MOTOR);
    private final static I2C.Port colorPort = I2C.Port.kOnboard;
    private static final ColorSensorV3 m_colorSensor = new ColorSensorV3(colorPort);
    private static String allianceColor = DriverStation.getAlliance().toString();
    private static double proxVal = 175;
    private static DigitalInput m_topSensor = new DigitalInput(1);
    public static boolean intaking;
    public static boolean fw; 
    private IntakeSubsystem m_intake = new IntakeSubsystem();
    private final ShooterSubsystem m_shoot = new ShooterSubsystem();
    private static boolean ballInCircCorrect = true;
    public final double hSpeed = 0.375; 

    public void gogoBallCenter(double power){
        m_centering.set(ControlMode.PercentOutput, -power);
    }

    public void gogoHopper(double speed){
        m_tower.set(ControlMode.PercentOutput, speed);
    }

    public void stopDaHopper() {
        m_tower.set(ControlMode.PercentOutput, 0.0);
        fw = false;
        stopCentering();
    }

    public void stopCentering() {
        m_centering.set(ControlMode.PercentOutput, 0.0);
    }

    public boolean ballInUpper() {
        return m_topSensor.get();
    }

    public boolean ballInLower() {
        return getProx() > proxVal;
    }

    public int getProx(){
        return m_colorSensor.getProximity();
    }

    public boolean blueBall() {
        return (getProx() > proxVal && m_colorSensor.getRed() < m_colorSensor.getBlue());
    }
    
    public boolean redBall() {
        return (getProx() > proxVal && m_colorSensor.getRed() > m_colorSensor.getBlue());
    }

    public boolean rightBall() {
        SmartDashboard.putString("alliance", allianceColor);
        if(allianceColor.equals("Blue")){
            return blueBall();
        } if (allianceColor.equals("Red")){
            return redBall();
        } else {
            return false;
        }
    }
    
    public void intakingLoop() {
        intaking = true;
            if (ballInLower() && ballInCircCorrect) { //make ballInLower
                stopDaHopper();
                m_shoot.stopShooter();
            } else {
                gogoHopper(hSpeed);
                m_shoot.garbageRemoval();
            }

            if (!ballInLower()) {
                gogoBallCenter(.5);
                m_intake.setIntake(.9);
            }
            
            if (rightBall() && ballInUpper() && ballInCircCorrect) {
                stopDaHopper();
                m_intake.retractIntake();
                m_shoot.stopShooter();
                intaking = false;
            } else if (!rightBall() && !ballInUpper() && ballInLower()) {
                ballInCircCorrect = true; //make false
            } else if (rightBall() && ballInUpper() && !ballInCircCorrect) {
                ballInCircCorrect = true;
            } else if (rightBall() && !ballInUpper()) {
                ballInCircCorrect = true;
            } else if (!rightBall() && ballInUpper() && ballInLower()) {
                m_intake.setIntake(-.75);
                gogoBallCenter(-.5);
                gogoHopper(-.375);
                Timer.delay(.1);
                stopDaHopper();
                Timer.delay(.3);
            }
    }

    public void reverseStuff() {
        gogoBallCenter(-.5);
        gogoHopper(-hSpeed);
    }

    @Override
    public void periodic() {
    }
    
}
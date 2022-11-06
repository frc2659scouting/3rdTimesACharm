// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Superstructure;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;

/*
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private final XboxController m_driveController = new XboxController(0);
  private final XboxController m_operatorController = new XboxController(1);
  

  private final PigeonIMU m_pig = new PigeonIMU(Constants.DRIVETRAIN_PIGEON_ID);
  private AutoModeSelector mAutoModeSelector = new AutoModeSelector();
  private static final ShooterSubsystem m_shoot = new ShooterSubsystem();
  private static final IntakeSubsystem m_intake = new IntakeSubsystem();
  private static final Superstructure m_super = new Superstructure();
  //private DrivetrainSubsystem mDriveTrain = new DrivetrainSubsystem();
  //private BackupAutoOption mBackupAutoOption = new BackupAutoOption();

  private double gyroRoll;  
  private double gyroPitch;
  private double gyroYaw;
  //private double startTime;
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private static HopperSubsystem m_hopper = new HopperSubsystem();
  public static boolean tracking = false;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  public static Color dColor;
  public static int prox;
  public static double ir;
  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    m_robotContainer.m_drivetrainSubsystem.resetEverything();
    
    mAutoModeSelector.updateModeCreator();    
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.

    CommandScheduler.getInstance().run();

    mAutoModeSelector.outputToSmartDashboard();

    //SmartDashboard.putNumber("Rotat", m_shoot.yaw);
    //SmartDashboard.putBoolean("Shooting", tracking);
    
    gyroRoll = m_pig.getRoll();
    gyroPitch = m_pig.getPitch();
    gyroYaw = m_pig.getYaw();
    SmartDashboard.putNumber("GyroRoll", gyroRoll);
    SmartDashboard.putNumber("GyroPitch", gyroPitch);
    SmartDashboard.putNumber("GyroYaw", gyroYaw);

    dColor = m_colorSensor.getColor();
    prox = m_colorSensor.getProximity();
    ir = m_colorSensor.getIR();
    
    SmartDashboard.putNumber("Red1", dColor.red);
    SmartDashboard.putNumber("Green", dColor.green);
    SmartDashboard.putNumber("Blue1", dColor.blue);
    SmartDashboard.putNumber("Proximity", prox);
    SmartDashboard.putBoolean("Ball In Upper", m_hopper.ballInUpper());
    SmartDashboard.putBoolean("Ball In Lower", m_hopper.ballInLower());
    SmartDashboard.putBoolean("Right Ball", m_hopper.rightBall());

  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    mAutoModeSelector.reset();
    mAutoModeSelector.updateModeCreator();
  }

  @Override
  public void disabledPeriodic() {
    // Update auto modes
    mAutoModeSelector.updateModeCreator();
    m_hopper.setAllianceColor();
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    //m_hopper.configureMotors(); //110522 
    DrivetrainSubsystem.zeroGyroscope(); //do not remove, very important
    m_autonomousCommand = m_robotContainer.getAutonomousCommand(); //uncomment to run command auto
 // startTime = Timer.getFPGATimestamp();
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    //startTime = Timer.getFPGATimestamp();

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
     // mBackupAutoOption.backupAutoOption(startTime);
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    m_hopper.setAllianceColor();
    m_robotContainer.m_drivetrainSubsystem.resetEverything();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    if (m_driveController.getBButton()) DrivetrainSubsystem.zeroGyroscope();
    if (m_driveController.getXButton()) m_robotContainer.m_drivetrainSubsystem.resetEverything();

    if (m_driveController.getLeftBumper()) {
      m_hopper.intakingLoop();
    } else if (m_driveController.getLeftBumperReleased()) {
      m_shoot.stopShooter();
      m_intake.retractIntake();
      m_hopper.stopDaHopper();
    } else if(m_driveController.getRightBumper()) {
      //m_shoot.track();
      m_shoot.wallShot();
    } else if (m_driveController.getRightBumperReleased()) {
      m_hopper.stopDaHopper();
      m_shoot.stopShooter();
      //m_shoot.noTrack();
    } else if (m_driveController.getRightTriggerAxis()>0.7)m_shoot.autoFire();
      else if (m_driveController.getRightTriggerAxis()<0.7 && m_driveController.getRightTriggerAxis()>0.15)m_shoot.stopShooter(); // THIS IS A JANK FIX!! essentially turns it off on a soft release - BVN 
    
    if(m_driveController.getStartButtonPressed())m_hopper.weAreRed();
    
    if(m_driveController.getRawButtonPressed(7))m_hopper.weAreBlue();
    


    
    if(m_operatorController.getLeftTriggerAxis() > 0.7){ //If we're climbing, then nothing else can be pressed
    m_super.climbControl(0);
      if (m_operatorController.getBButtonPressed()) {
        m_super.climbControl(1); //advance climb state
      }
      if(m_operatorController.getXButtonPressed()){
        m_super.climbControl(-1); //return climb state
      }
    }
    else{

    if (m_operatorController.getAButton()) {
      HopperSubsystem.intaking = false;
      m_intake.retractIntake();
      m_hopper.stopDaHopper();
      m_shoot.stopShooter();
      /*
    } else if (m_operatorController.getRightBumper()) {
      m_shoot.autoFire();
      m_shoot.track();
    } else if (m_operatorController.getRightBumperReleased()) {
      m_shoot.stopShooter();
      m_hopper.stopDaHopper();
      m_shoot.noTrack();
      */
    } else if (m_operatorController.getLeftBumper()) {
      m_hopper.reverseStuff();
      m_intake.setIntake(-.5);
    } else if (m_operatorController.getLeftBumperReleased()) {
      m_hopper.stopDaHopper();
      m_intake.retractIntake();
    } else if(m_operatorController.getRightBumper()) {
      m_shoot.wallShootPrepare();
      SmartDashboard.putNumber("key", m_operatorController.getRightTriggerAxis());
        if (m_operatorController.getRightTriggerAxis() > 0.9) m_shoot.wallShot();
    } else if (m_operatorController.getRightBumperReleased()) {
      m_shoot.stopShooter();
      m_hopper.stopDaHopper();
    }
  }

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {

    gyroRoll = m_pig.getRoll();
    gyroPitch = m_pig.getPitch();
    gyroYaw = m_pig.getYaw();
  //  SmartDashboard.putNumber("ClimbState", climbState);
    SmartDashboard.putNumber("GyroRoll", gyroRoll);
    SmartDashboard.putNumber("GyroPitch", gyroPitch);
    SmartDashboard.putNumber("GyroYaw", gyroYaw);


  }
  
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import java.util.List;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private AutoModeSelector mAutoModeSelector = new AutoModeSelector();
  private final XboxController m_controller = new XboxController(0);
  private static final ShooterSubsystem m_shoot = new ShooterSubsystem();
  private static final IntakeSubsystem m_intake = new IntakeSubsystem();
  private static final HopperSubsystem m_hopper = new HopperSubsystem();
  //private static double rotationSpoof = 0;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    //rotationSpoof =  m_shoot.yaw;
      m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
            m_drivetrainSubsystem,
            () -> -modifyAxis(m_controller.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> modifyAxis(m_controller.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> modifyAxis(m_controller.getRightX() * 0.75) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
      ));
      //SmartDashboard.putNumber("rotationControllerSpoof", m_shoot.yaw);
  }
    // Configure the button bindings

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  
  public Command getAutonomousCommand() {
    // 1. Create Trajectory Settings
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond, 
            AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                      .setKinematics(DrivetrainSubsystem.m_kinematics);

    // 2. Generate trajectory
       /*  List.of(
              new Translation2d(2.0, 0.0)),
      new Pose2d(2.0, 0.0, Rotation2d.fromDegrees(0.0)),
      trajectoryConfig);*/        

    // 3. Define PID controllers for tracking trajectory
      PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
      PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
      ProfiledPIDController thetaController = new ProfiledPIDController(
      AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
      thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // 4. Construct command to follow trajectory

    // 5. Add some init and wrap-up, and return everything
    // Select the auto mode from the SmartDash Sendable Chooser
    if(mAutoModeSelector.returnAutoMode().toString().equals("DO_NOTHING")){
      return new InstantCommand();
    }
    
    else if(mAutoModeSelector.returnAutoMode().toString().equals("SHOOT_DRIVE_BACK")) {
        Trajectory shootDriveTrajectory = TrajectoryGenerator.generateTrajectory(
          new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
          List.of(
            new Translation2d(-2.0 , 0.01)),
          new Pose2d(-1.8, 0.0, Rotation2d.fromDegrees(180 - 158.5)), 
          trajectoryConfig);
        //Shoot Drive Back Path
        SwerveControllerCommand swerveControllerCommandShootDrive = new SwerveControllerCommand(
          shootDriveTrajectory,
          m_drivetrainSubsystem::getPose,
          DrivetrainSubsystem.m_kinematics,
          xController,
          yController,
          thetaController,
          m_drivetrainSubsystem::setModuleStates,
          m_drivetrainSubsystem);
          
          return new SequentialCommandGroup(
            new InstantCommand(() -> m_shoot.autonShoot()),
            new InstantCommand(() -> m_drivetrainSubsystem.resetOdometry(shootDriveTrajectory.getInitialPose())),
            swerveControllerCommandShootDrive);

     }
    
    else if(mAutoModeSelector.returnAutoMode().toString().equals("RIGHT_SIDE_3_BALL")) {

        //shoot

      Trajectory trajectory = TrajectoryGenerator.generateTrajectory( //drives to edge of tarmac
      new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
      List.of(
        new Translation2d(-0.001, 0.001)),
      new Pose2d(-0.002, 0.002, Rotation2d.fromDegrees(0.0)), //rotation to paralell intake with outer tarmac line
      trajectoryConfig);      
    
    //intake down

      Trajectory trajectory1 = TrajectoryGenerator.generateTrajectory( //drives to 3rd ball
        new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
        List.of(
          new Translation2d(-1.42, 0.58)), //-0.5, 0.035
        new Pose2d(-2.1, 0.65, Rotation2d.fromDegrees(158.5)), 
        trajectoryConfig);

        Trajectory trajectory2 = TrajectoryGenerator.generateTrajectory( //drives to edge of tarmac
      new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
      List.of(
        new Translation2d(0.001, -0.001)),
      new Pose2d(0.002, -0.002, Rotation2d.fromDegrees(0.0)), //rotation to paralell intake with outer tarmac line
      trajectoryConfig);      
    
    //intake down

      Trajectory trajectory3 = TrajectoryGenerator.generateTrajectory( //drives to 3rd ball
        new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
        List.of(
          new Translation2d(1.42, -0.58)), //-0.5, 0.035
        new Pose2d(2.1, -0.65, Rotation2d.fromDegrees(-158.5)), 
        trajectoryConfig);
        
      SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
              trajectory,
              m_drivetrainSubsystem::getPose,
              DrivetrainSubsystem.m_kinematics,
              xController,
              yController,
              thetaController,
              m_drivetrainSubsystem::setModuleStates,
              m_drivetrainSubsystem);

      SwerveControllerCommand swerveControllerCommand1 = new SwerveControllerCommand(
              trajectory1,
              m_drivetrainSubsystem::getPose,
              DrivetrainSubsystem.m_kinematics,
              xController,
              yController,
              thetaController,
              m_drivetrainSubsystem::setModuleStates,
              m_drivetrainSubsystem);  
              
              SwerveControllerCommand swerveControllerCommand2 = new SwerveControllerCommand(
                trajectory2,
                m_drivetrainSubsystem::getPose,
                DrivetrainSubsystem.m_kinematics,
                xController,
                yController,
                thetaController,
                m_drivetrainSubsystem::setModuleStates,
                m_drivetrainSubsystem);
  
        SwerveControllerCommand swerveControllerCommand3 = new SwerveControllerCommand(
                trajectory3,
                m_drivetrainSubsystem::getPose,
                DrivetrainSubsystem.m_kinematics,
                xController,
                yController,
                thetaController,
                m_drivetrainSubsystem::setModuleStates,
                m_drivetrainSubsystem);

     return new SequentialCommandGroup(
      new InstantCommand(() -> m_shoot.autonShoot()),
            new InstantCommand(() -> m_drivetrainSubsystem.resetOdometry(trajectory.getInitialPose())),
             swerveControllerCommand,
            new InstantCommand(() -> m_drivetrainSubsystem.resetOdometry(trajectory1.getInitialPose())),
             swerveControllerCommand1,
             new InstantCommand(() -> m_intake.setIntake(.75)),
             new InstantCommand(() -> m_hopper.gogoHopper(.5)),
            new InstantCommand(() -> Timer.delay(.3)),
            new InstantCommand(() -> m_hopper.stopDaHopper()),
             new InstantCommand(() -> m_intake.retractIntake()),
             new InstantCommand(() -> m_drivetrainSubsystem.resetOdometry(trajectory2.getInitialPose())),
             swerveControllerCommand2,
             new InstantCommand(() -> m_drivetrainSubsystem.resetOdometry(trajectory3.getInitialPose())),
             swerveControllerCommand3,
             new InstantCommand(() -> m_shoot.autonShoot()),
            new InstantCommand(() -> m_drivetrainSubsystem.stopModules()));   
            // return new InstantCommand();
    } else if(mAutoModeSelector.returnAutoMode().toString().equals("LEFT_SIDE_3_BALL")) {

      //10-26-22 OWEN N.

    Trajectory trajectory = TrajectoryGenerator.generateTrajectory( //drives to edge of tarmac
    new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
    List.of(
      new Translation2d(-0.1, 0.01)),
    new Pose2d(-0.2, 0.02, Rotation2d.fromDegrees(0.0)), //rotation to paralell intake with outer tarmac line
    trajectoryConfig);      
  
  //intake down

    Trajectory trajectory1 = TrajectoryGenerator.generateTrajectory( //drives to 3rd ball
      new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
      List.of(
        new Translation2d(-0.7, -0.48)), //-0.5, 0.035
      new Pose2d(-2.7, -1.0, Rotation2d.fromDegrees(179.0)), 
      trajectoryConfig);

      Trajectory trajectory2 = TrajectoryGenerator.generateTrajectory( 
    new Pose2d(-2.7, -1.0, new Rotation2d(179.0)),
    List.of(
      new Translation2d(-1.0, -.4)), //(0.7, 0.48)
    new Pose2d(0.001, 0.001, Rotation2d.fromDegrees(0.0)), //2.4, 0.85
    trajectoryConfig);      

    Trajectory trajectory3 = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
      List.of(
        new Translation2d(0.1, -0.01)), //-0.5, 0.035
      new Pose2d(0.02, -0.02, Rotation2d.fromDegrees(0.0)), 
      trajectoryConfig);
      
    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
            trajectory,
            m_drivetrainSubsystem::getPose,
            DrivetrainSubsystem.m_kinematics,
            xController,
            yController,
            thetaController,
            m_drivetrainSubsystem::setModuleStates,
            m_drivetrainSubsystem);

    SwerveControllerCommand swerveControllerCommand1 = new SwerveControllerCommand(
            trajectory1,
            m_drivetrainSubsystem::getPose,
            DrivetrainSubsystem.m_kinematics,
            xController,
            yController,
            thetaController,
            m_drivetrainSubsystem::setModuleStates,
            m_drivetrainSubsystem);  
            
            SwerveControllerCommand swerveControllerCommand2 = new SwerveControllerCommand(
              trajectory2,
              m_drivetrainSubsystem::getPose,
              DrivetrainSubsystem.m_kinematics,
              xController,
              yController,
              thetaController,
              m_drivetrainSubsystem::setModuleStates,
              m_drivetrainSubsystem);

      SwerveControllerCommand swerveControllerCommand3 = new SwerveControllerCommand(
              trajectory3,
              m_drivetrainSubsystem::getPose,
              DrivetrainSubsystem.m_kinematics,
              xController,
              yController,
              thetaController,
              m_drivetrainSubsystem::setModuleStates,
              m_drivetrainSubsystem);

   return new SequentialCommandGroup(
    new InstantCommand(() -> m_shoot.autonShoot()),
    new InstantCommand(() -> m_drivetrainSubsystem.resetOdometry(trajectory.getInitialPose())),
        swerveControllerCommand,
        new InstantCommand(() -> m_intake.setIntake(.75)),
    new InstantCommand(() -> m_drivetrainSubsystem.resetOdometry(trajectory1.getInitialPose())),
        swerveControllerCommand1,
        new InstantCommand(() -> m_hopper.gogoHopper(.375)),
        new InstantCommand(() -> Timer.delay(0.5)),
        new InstantCommand(() -> m_hopper.stopDaHopper()),
        new InstantCommand(() -> m_intake.retractIntake()),
    new InstantCommand(() -> m_drivetrainSubsystem.resetOdometry(trajectory2.getInitialPose())),
        swerveControllerCommand2,
    //new InstantCommand(() -> m_drivetrainSubsystem.resetOdometry(trajectory3.getInitialPose())),
        //swerveControllerCommand3,
        new InstantCommand(() -> m_shoot.autonShoot()),
    new InstantCommand(() -> m_drivetrainSubsystem.stopModules()));   
    
  }  else if(mAutoModeSelector.returnAutoMode().toString().equals("RIGHT_SIDE_4_BALL")) {

      //shoot

      Trajectory rightSide1 = TrajectoryGenerator.generateTrajectory( //drives to edge of tarmac
        new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
        List.of(
          new Translation2d(-0.001, 0.001)),
        new Pose2d(-0.002, 0.002, Rotation2d.fromDegrees(0.0)), //rotation to paralell intake with outer tarmac line
        trajectoryConfig);      
      
      //intake down

        Trajectory rightSide2 = TrajectoryGenerator.generateTrajectory( //drives to 3rd ball
          new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
          List.of(
            new Translation2d(-1.42, 0.58)), //-0.5, 0.035
          new Pose2d(-2.1, 0.65, Rotation2d.fromDegrees(158.5)), 
          trajectoryConfig);  

        Trajectory rightSide3 = TrajectoryGenerator.generateTrajectory( //drives to 4th ball
          new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
          List.of(
            new Translation2d(-0.5, 0.5)),
          new Pose2d(-1.5, 1.3, Rotation2d.fromDegrees(93.349)), 
          trajectoryConfig);        
      
      
        Trajectory rightSide4 = TrajectoryGenerator.generateTrajectory( //drive to optimal shooting position
        new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
        List.of(
          new Translation2d(-1.08, 0.7)),
        new Pose2d(-2.16, 1.42, Rotation2d.fromDegrees(93.349)), 
        trajectoryConfig);     
        
        Trajectory rightSide5 = TrajectoryGenerator.generateTrajectory( //drive to strategic teleop starting position
        new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
        List.of(
          new Translation2d(0.1, 0.0)),
        new Pose2d(0.2, 0.0, Rotation2d.fromDegrees(0.0)), //rotation so that intake is away from driver and paralell to driver wall
        trajectoryConfig); 
        
        SwerveControllerCommand rightSideMulti1 = new SwerveControllerCommand(
            rightSide1,
            m_drivetrainSubsystem::getPose,
            DrivetrainSubsystem.m_kinematics,
            xController,
            yController,
            thetaController,
            m_drivetrainSubsystem::setModuleStates,
            m_drivetrainSubsystem);      
      
        SwerveControllerCommand rightSideMulti2 = new SwerveControllerCommand(
            rightSide2,
            m_drivetrainSubsystem::getPose,
            DrivetrainSubsystem.m_kinematics,
            xController,
            yController,
            thetaController,
            m_drivetrainSubsystem::setModuleStates,
            m_drivetrainSubsystem);    
      
        SwerveControllerCommand rightSideMulti3 = new SwerveControllerCommand(
            rightSide3,
            m_drivetrainSubsystem::getPose,
            DrivetrainSubsystem.m_kinematics,
            xController,
            yController,
            thetaController,
            m_drivetrainSubsystem::setModuleStates,
            m_drivetrainSubsystem);      
      
        SwerveControllerCommand rightSideMulti4 = new SwerveControllerCommand(
            rightSide4,
            m_drivetrainSubsystem::getPose,
            DrivetrainSubsystem.m_kinematics,
            xController,
            yController,
            thetaController,
            m_drivetrainSubsystem::setModuleStates,
            m_drivetrainSubsystem);   
            
        SwerveControllerCommand rightSideMulti5 = new SwerveControllerCommand(
            rightSide5,
            m_drivetrainSubsystem::getPose,
            DrivetrainSubsystem.m_kinematics,
            xController,
            yController,
            thetaController,
            m_drivetrainSubsystem::setModuleStates,
            m_drivetrainSubsystem); 
     
        return new SequentialCommandGroup(
          new InstantCommand(() -> m_shoot.autonShoot()),
             new InstantCommand(() -> m_drivetrainSubsystem.resetOdometry(rightSide1.getInitialPose())),
              rightSideMulti1,
             new InstantCommand(() -> m_drivetrainSubsystem.resetOdometry(rightSide2.getInitialPose())),
              rightSideMulti2,
              new InstantCommand(() -> m_intake.setIntake(.75)),
            new InstantCommand(() -> m_hopper.gogoHopper(.5)),
            new InstantCommand(() -> Timer.delay(.3)),
            new InstantCommand(() -> m_hopper.stopDaHopper()),
             new InstantCommand(() -> m_drivetrainSubsystem.resetOdometry(rightSide3.getInitialPose())),
             rightSideMulti3,
             new InstantCommand(() -> Timer.delay(.1)),
            new InstantCommand(() -> m_intake.retractIntake()),
             new InstantCommand(() -> m_drivetrainSubsystem.resetOdometry(rightSide4.getInitialPose())),
             rightSideMulti4,
             new InstantCommand(() -> m_shoot.autonShoot()),
             new InstantCommand(() -> m_drivetrainSubsystem.resetOdometry(rightSide4.getInitialPose())),
             rightSideMulti5,
             new InstantCommand(() -> m_drivetrainSubsystem.stopModules()));   
     } else if(mAutoModeSelector.returnAutoMode().toString().equals("DISCO")) {
      Trajectory shootDriveTrajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
        List.of(
          new Translation2d(0.0 , 1.0),
          new Translation2d(-1.0, 1.0),
          new Translation2d(-1.0, 0.0)),
        new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(280.0)), 
        trajectoryConfig);

      SwerveControllerCommand swerveControllerCommandShootDrive = new SwerveControllerCommand(
        shootDriveTrajectory,
        m_drivetrainSubsystem::getPose,
        DrivetrainSubsystem.m_kinematics,
        xController,
        yController,
        thetaController,
        m_drivetrainSubsystem::setModuleStates,
        m_drivetrainSubsystem);
        
    return new SequentialCommandGroup();
    } else if(mAutoModeSelector.returnAutoMode().toString().equals("JUST_SHOOT")) {
      return new SequentialCommandGroup(
        new InstantCommand(() -> m_shoot.autonShoot()));
    } else {
       return new InstantCommand();
     }
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.08);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
  /**********************custom auto thing****************************/
  public Boolean superSpicyCustomAuto(double xChange, double yChange, double angleChange){
    double yGain = 1;
    double xGain = 1;
    double zGain = 1;
    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
      m_drivetrainSubsystem,
      () -> -modifyAxis(yGain*(yChange - m_drivetrainSubsystem.getPose().getY())) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
      () -> modifyAxis(xGain*(xChange - m_drivetrainSubsystem.getPose().getX())) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
      () -> modifyAxis(zGain*(angleChange - m_drivetrainSubsystem.getRotation().getDegrees())) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
     ));
    if (m_drivetrainSubsystem.getPose().getY() - yChange < 0.01 ){
      if( m_drivetrainSubsystem.getPose().getX() - xChange < 0.01 ){
        if(angleChange - m_drivetrainSubsystem.getRotation().getDegrees() < 1.0){
          return true;          
        }
      }
    } 
     return false;
  }
   /**********************custom auto thing****************************/
   public void resetTheOdo(){
    m_drivetrainSubsystem.resetOdometry(new Pose2d(0,0,new Rotation2d()));
   }
  
}

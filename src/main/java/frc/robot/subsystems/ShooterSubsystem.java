package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Timer;

public class ShooterSubsystem extends SubsystemBase {
    private static final TalonFX m_shooterMotor = new TalonFX(Constants.FLYWHEEL_MOTOR);
    private static final TalonFX m_hoodMotor = new TalonFX(Constants.HOOD_MOTOR);

    //private final PigeonIMU m_pig = new PigeonIMU(Constants.DRIVETRAIN_PIGEON_ID);
//UNCOMMENT    public final static DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem(); //Could be causing issue
    //private final static XboxController m_controller = new XboxController(0);

    private static double flywheelShotSpeed = 8269.0; 
    private double flywheelDeadband = 63; //set back to 100 - 200 used for testing
    private static final HopperSubsystem m_hopper = new HopperSubsystem();
    /*
    public static PhotonCamera camera = new PhotonCamera("Eye of the Beholder");
    public static PhotonPipelineResult result = new PhotonPipelineResult();
    public static PhotonTrackedTarget target = new PhotonTrackedTarget();
    public static double yaw;
    public static double pitch;
    public static double rotat;
    private static boolean tracking = false;
    private static boolean spit = false;
    private static boolean spitStart = true;
    private static double flyWheelSpeedTarget;
    private static double goalDistance;
    private static double entryHeight = 2; //ft
    private static double staticShotVelocity;
    private static double movingShotVelocity;
    private static double entryVelocity;
    private static double entryAngle = -30 * Math.PI / 180; // was 30
    private static final double goalHeight = 82 / 12;
    private static double horizontalShotVelocity;
    private static double verticalShotVelocity;
    private static double hoodAngleTarget;
    private static double goalAngle;
    private static double airResComp; //air resistance compensation factor
    private static double gravity = 16.2; //Feet per second^2
    private static double movingXOffset;
    private static double yawComp = 0; //yaw compensation for moving shot
    private static double hoodComp = 0;//copmensation for moving shot
    private static double shotSpeedComp = 0; //compensation for moving shot
    private static double xVel = 0;
    private static double yVel = 0;
    private static double xVelGoal = 0;
    private static double yVelGoal = 0;
    private static double timeToGoal = 0;
    private static double botAngle = 0;
    private static double spitTime = 0;
    */
    

    public double getShooterSpeed() {
        SmartDashboard.putNumber("Flywheel Speed", m_shooterMotor.getSelectedSensorVelocity());
        return m_shooterMotor.getSelectedSensorVelocity();
    }
    
    public static void setShooter() {
        m_shooterMotor.set(ControlMode.Velocity, flywheelShotSpeed); //
        SmartDashboard.putNumber("Flywheel Speed", m_shooterMotor.getSelectedSensorVelocity());
    }

    public void wallShootPrepare() {
        setShooter();
        /*
        m_hoodMotor.setInverted(true);
        m_hoodMotor.set(ControlMode.Position, -200);  // 10-26-22 slightly less than 0 - OWEN TUNE THIS
        */
    }

    public void wallShot() {
        if (reachedDesiredSpeed()) {
            m_hopper.gogoHopper(.4); //10-26-22 BVN ramping this up due to ball feed issues
        } else {
            //flywheelShotSpeed = 8000; //10-26-22 BVN this is already getting set in wall-shot prepare
            setShooter(); //10-26-22 BVN needs to be here so we still spit
            m_hopper.stopDaHopper();
        }
    }

    public void autonShoot() {
        wallShootPrepare();
        Timer.delay(1.5); //1
        m_hopper.gogoHopper(.375);
        Timer.delay(.2); //.1
        m_hopper.stopDaHopper();
        Timer.delay(1); //1
        m_hopper.gogoHopper(.375);
        Timer.delay(1);
        m_hopper.stopDaHopper();
        stopShooter();
    }

    public void stopShooter() {
        //camera.setLED(VisionLEDMode.kOff);
        SmartDashboard.putNumber("Flywheel Speed", m_shooterMotor.getSelectedSensorVelocity());
        m_shooterMotor.set(ControlMode.PercentOutput, 0.0);
        //tracking = false;
        //spit = true;
    }

    public void garbageRemoval() {
        SmartDashboard.putNumber("Flywheel Speed", m_shooterMotor.getSelectedSensorVelocity());
        m_shooterMotor.set(ControlMode.PercentOutput, .375);
    }

    public void setHood(double angle) {
        m_hoodMotor.set(ControlMode.Position, angle);
    }

    /*
    public void fire(boolean wall) {
        camera.setLED(VisionLEDMode.kOn);
        if (reachedDesiredSpeed() && yaw != 0 && -5 < yaw && yaw < 5 && !wall) { //YAW TO 5 from 0.5
            m_hopper.gogoHopper(m_hopper.hSpeed*.8);
            m_hopper.gogoBallCenter(.25);
        } else if(!wall) {
            m_hopper.stopDaHopper();
        } else if(wall && reachedDesiredSpeed()) {
            m_hopper.gogoHopper(m_hopper.hSpeed*.8);
            m_hopper.gogoBallCenter(.25);
        }
    }
    */

    public boolean reachedDesiredSpeed() {
        if(m_shooterMotor.getSelectedSensorVelocity() < flywheelShotSpeed + flywheelDeadband && m_shooterMotor.getSelectedSensorVelocity() > flywheelShotSpeed - flywheelDeadband  ){ //setHalfspeed cause may take too long to get to speed 
            SmartDashboard.putNumber("Flywheel Speed", m_shooterMotor.getSelectedSensorVelocity());
            return true;
        }else{
            return false;
        }
    }

    /*
    public void track() {
        tracking = true;
        if (yaw != 0) {
            rotat = yaw/15; //tune dis
        } else {
            rotat = 0;
        }
    }

    public static double returnRotat(){
       // track();
        return yaw;
    }
    private static void getShotWhileMoving() {
        camera.setLED(VisionLEDMode.kOn);
        result = camera.getLatestResult();
        target = result.getBestTarget(); 
            if(target != null) {
                pitch = target.getPitch();
            } else {
                pitch = 0;
                yaw = 0;
                yawComp = 0;
                return;
            }
        //m_drivetrainSubsystem.m_chassisSpeeds;
        
        botAngle = DrivetrainSubsystem.headingDegrees;
        xVel = -m_controller.getLeftX() * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND;    
        yVel = -m_controller.getLeftY() * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND;
//() -> modifyAxis(m_controller.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND
        xVelGoal = xVel * Math.cos(yaw * Math.PI / 180 + botAngle) +  yVel * Math.sin(yaw * Math.PI / 180+botAngle);
        yVelGoal = yVel * Math.cos(yaw * Math.PI / 180 + botAngle) +  xVel * Math.sin(yaw * Math.PI / 180 + botAngle);

        goalAngle = pitch + 40;
        goalAngle = goalAngle * Math.PI / 180;

        goalDistance = goalHeight / Math.tan(goalAngle);
        SmartDashboard.putNumber("Goal Dist", goalDistance);

        // airResComp = (goalDistance*goalDistance)*.000553 + (goalDistance*.00798) + 1.07; //CD (Coefficient of Drag) = .5 !!This is included as a linearized factor instead of second order, directly in flywheel speed setting

//             horizontalShotVelocity = Math.sqrt(Math.pow(goalDistance, 2)*gravity/(2*(goalHeight + entryHeight + Math.tan(entryAngle)*goalDistance)));
        horizontalShotVelocity = Math.pow(Math.pow(goalDistance, 2) * gravity / (2 * (goalHeight + entryHeight + Math.tan(-entryAngle)*goalDistance)), .5);
        //SmartDashboard.putNumber("horizontalShotVelocity", horizontalShotVelocity);
        entryVelocity = horizontalShotVelocity/Math.cos(entryAngle);
        //SmartDashboard.putNumber("entryVelocity", entryVelocity);

        hoodAngleTarget = Math.atan(Math.sqrt(Math.pow(Math.sin(entryAngle), 2) + (2*gravity*(goalHeight + entryHeight)/Math.pow(entryVelocity, 2)))/Math.pow(Math.cos(entryAngle), 2));
        staticShotVelocity = horizontalShotVelocity/Math.cos(hoodAngleTarget); //basically uses the hood angle target to calc the shot speed withouth wind resistance
        //SmartDashboard.putNumber("staticShotVelocity", staticShotVelocity);

        hoodAngleTarget = hoodAngleTarget * 180 / Math.PI;
        //SmartDashboard.putNumber("hoodAngleTarget", hoodAngleTarget);

        verticalShotVelocity = Math.pow(Math.pow(staticShotVelocity, 2)-Math.pow(horizontalShotVelocity, 2), 0.5);

        //flyWheelSpeedTarget = (staticShotVelocity*12*60)/(4*Math.PI);
        timeToGoal = goalDistance / horizontalShotVelocity; //ball airtime

        movingXOffset = timeToGoal * xVelGoal; //ball airtime x tangential speed (relative to goal)

        yawComp = Math.atan(movingXOffset/goalDistance)*180 / Math.PI; //ground relative (z axis) compensation angle for moving shot
        horizontalShotVelocity -= yVelGoal; //subtract goal relative speed
        movingShotVelocity = Math.pow(Math.pow(horizontalShotVelocity, 2)+Math.pow(verticalShotVelocity, 2), 0.5);
        hoodComp = Math.atan(verticalShotVelocity / horizontalShotVelocity)* 180 / Math.PI - hoodAngleTarget;
        flyWheelSpeedTarget = (movingShotVelocity*12*60)/(4*Math.PI) * 8.8; //OWEN TUNE THIS
        flywheelShotSpeed = flyWheelSpeedTarget * (.95+.0025*goalDistance); //Air resistance compensation **102622 was 1.01+.00209*goalDistance 
        SmartDashboard.putNumber("yawComp", yawComp);
        SmartDashboard.putNumber("goal tangent velocity", xVelGoal);
        SmartDashboard.putNumber("yGoalRelative", yVelGoal);
        SmartDashboard.putNumber("Hood Angle Target with comp", hoodAngleTarget + hoodComp);
        SmartDashboard.putNumber("flywheelShotSpeed", flywheelShotSpeed);
        SmartDashboard.putNumber("vert shot velocity", verticalShotVelocity);
        setShooter();
        // setHood();
        m_hoodMotor.set(ControlMode.Position, ((90 - hoodAngleTarget - hoodComp + 22 )/(53 - 10)*-15000)); //sets hood angle by motor position OWEN TUNE THE ANGLE - CURRENTLY 25, higher is aim lower

        }

     //   private static void setHood() {
     //       m_hoodMotor.set(ControlMode.Position, ((90 - hoodAngleTarget - 10)/(53 - 10)*15000)); //sets hood angle by motor position
     //   }

        public void autoFire() {
            m_hoodMotor.setInverted(false);
            camera.setLED(VisionLEDMode.kOn);
            //getShot();
            tracking = true;
            getShotWhileMoving();
            if (reachedDesiredSpeed() && yaw != 0 && -5 < yaw && yaw < 5) {
                m_hopper.gogoHopper(m_hopper.hSpeed); //was 50%, but that was on top of a 37.5% speed, so was only activating 18%
                m_hopper.gogoBallCenter(.25);
            }
            else{
                if(!spit) m_hopper.stopDaHopper();
            }
        }

    public void noTrack() {
        rotat = 0;
        yaw = 0;
        yawComp = 0;
        tracking = false;
    }
    */

    @Override
    public void periodic() {
        /*
        result = camera.getLatestResult();
        target = result.getBestTarget(); 
            if(target != null && tracking) {
                if(target.getYaw() + yawComp > 10) yaw = 10;
                else if(target.getYaw() + yawComp < -10) yaw = -10;
                else yaw = (target.getYaw() + yawComp);
            } else {
                yaw = 0;
                yawComp = 0;
            }
            */  
    }
    
/*
    private static void getShot() {
        yawComp = 0;
        camera.setLED(VisionLEDMode.kOn);
        result = camera.getLatestResult();
        target = result.getBestTarget(); 
            if(target != null) {
                pitch = target.getPitch();
            } else {
                pitch = 0;
                return;
            }
            
            goalAngle = pitch + 40;
            goalAngle = goalAngle * Math.PI / 180;

            goalDistance = goalHeight / Math.tan(goalAngle);
            SmartDashboard.putNumber("Goal Dist", goalDistance);

            airResComp = (goalDistance*goalDistance)*.000553 + (goalDistance*.00798) + 1.07; //CD (Coefficient of Drag) = .5

//             horizontalShotVelocity = Math.sqrt(Math.pow(goalDistance, 2)*gravity/(2*(goalHeight + entryHeight + Math.tan(entryAngle)*goalDistance)));
            horizontalShotVelocity = Math.pow(Math.pow(goalDistance, 2) * gravity / (2 * (goalHeight + entryHeight + Math.tan(-entryAngle)*goalDistance)), .5);
            SmartDashboard.putNumber("horizontalShotVelocity", horizontalShotVelocity);
            entryVelocity = horizontalShotVelocity/Math.cos(entryAngle);
            SmartDashboard.putNumber("entryVelocity", entryVelocity);
            hoodAngleTarget = Math.atan(Math.sqrt(Math.pow(Math.sin(entryAngle), 2) + (2*gravity*(goalHeight + entryHeight)/Math.pow(entryVelocity, 2)))/Math.pow(Math.cos(entryAngle), 2));
            staticShotVelocity = horizontalShotVelocity/Math.cos(hoodAngleTarget);
            SmartDashboard.putNumber("staticShotVelocity", staticShotVelocity);
            hoodAngleTarget =  hoodAngleTarget * 180 / Math.PI;
            SmartDashboard.putNumber("hoodAngleTarget", hoodAngleTarget);
            
            //flyWheelSpeedTarget = (staticShotVelocity*12*60)/(4*Math.PI);
            flyWheelSpeedTarget = (staticShotVelocity*12*60)/(4*Math.PI) * (1.01+.00209*goalDistance); //Air resistance compensation
            flywheelShotSpeed = flyWheelSpeedTarget * 9;
            SmartDashboard.putNumber("flywheelShotSpeed", flywheelShotSpeed);
            setShooter();
           // setHood();
           m_hoodMotor.set(ControlMode.Position, ((90 - hoodAngleTarget - 5 )/(53 - 10)*-15000)); //sets hood angle by motor position
        }
        */
}

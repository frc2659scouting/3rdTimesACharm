// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.subsystems.DrivetrainSubsystem;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.737; //29in. DONE Measure and set trackwidth
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.736; //29in. DONE Measure and set wheelbase

    public static final int DRIVETRAIN_PIGEON_ID = 13; // DONE Set Pigeon ID

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 4; // DONE Set front left module drive motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 5; // DONE Set front left module steer motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 6; // DONE Set front left steer encoder ID
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = Math.toRadians(0.0  + 62); // DONE Measure and set front left steer offset

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 10; // DONE Set front right drive motor ID
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 11; // DONE Set front right steer motor ID
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 12; // DONE Set front right steer encoder ID
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = Math.toRadians(27.6 ); // DONE Measure and set front right steer offset

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 1; // DONE Set back left drive motor ID
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 2; // DONE Set back left steer motor ID
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 3; // DONE Set back left steer encoder ID
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = Math.toRadians(0.0 - 2.3 - 1.4) ; // DONE Measure and set back left steer offset

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 7; // DONE Set back right drive motor ID
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 8; // DONE Set back right steer motor ID
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 9; // DONE Set back right steer encoder ID
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = Math.toRadians(278.2); // DONE Measure and set back right steer offset

    public static final int HOPPER_MOTOR = 14;
    public static final int BALL_CENTERING_MOTOR = 18;
    public static final int INTAKE_MOTOR = 19;
    public static final int FLYWHEEL_MOTOR = 20;
    public static final int HOOD_MOTOR = 21;
    public static final double _GEAR_RATIO = (99/1); 
    public static final int CLIMB_1 = 25;
    public static final int CLIMB_2 = 41;

    public static final double EXTENDED_CLIMB_POSITION = 69;
    public static final double CLIMB_ROLL_POS_1 = 69;
    public static final double CLIMB_ROLL_POS_2 = 420;
    public static final double TELESCOPE_GEAR_RATIO = Math.PI*((6.9/42.0)*(42.0/6.9));

    public static final class AutoConstants{

        public static final double kMaxSpeedMetersPerSecond = DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND / 2; //change this?
        public static final double kMaxAngularSpeedRadiansPerSecond = 
        DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * 4;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3; //try 3
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI * .8;//this is good

        public static final double kPXController = 4.5;// 1, 3, maybe this needs to be wayyy higher? we shall see carlos barlos jr. the 3rd
        public static final double kPYController = 4.5; // 1, 3
        public static final double kPThetaController = 20.5; // 3 okay, 1.5 bad, 6 still too low...    I think, 10 too low
        
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = 
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
    }    

}


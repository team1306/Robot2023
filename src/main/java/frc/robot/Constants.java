package frc.robot;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.util.Units;

/**
 * A class for holding constant values in a single editable spot. What goes in this file: -Robot Motorcontroller ports
 * and IDS -Robot phsyical attributes -Field attributes
 * 
 * Things that don't go in this file: -Subsystem-specific PID values -Subsystem-specific sensor thresholds
 * 
 * Constants is interface as interface fields are public, static, and final by default
 * 
 */
@SuppressWarnings("unused")
public interface Constants {
    // ============ CAN IDS for Testing w/ Turbo =============
    // int LEFT_DRIVETRAIN_TALON = 1;
    // int RIGHT_DRIVETRAIN_TALON = 2;
    // int LEFT_DRIVETRAIN_VICTOR = 3;
    // int RIGHT_DRIVETRAIN_VICTOR = 4;

    // ============ Current Year CAN IDs ====================
    // drivetrain IDs
    int SPARK_FAR_LEFT = 1;
    int SPARK_NEAR_LEFT = 2;
    int SPARK_NEAR_RIGHT = 3;
    int SPARK_FAR_RIGHT = 4;
    // grabber ID
    int SPARK_CENTER = 5;
    // intake IDs
    int TALON_FAR_RIGHT = 6;
    int TALON_NEAR_RIGHT = 7;
    int TALON_NEAR_LEFT = 8;
    int TALON_FAR_LEFT = 9;


    // double TRACK_WIDTH_METERS = .60;
    // double MAX_SPEED_MPS = 4.67;
    // double MAX_ACCELERATION_MPSS = 3;
    double WHEEL_CIRCUMFERENCE = Units.inchesToMeters(6) * Math.PI;

    // double MAX_DIFF_ACCELERATION = 0;

    double TIME_TO_FULL_SPEED = .25;
    double TIME_PER_LOOP = .02;
}

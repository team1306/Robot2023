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
    int LEFT_DRIVETRAIN_TALON = 1;
    int RIGHT_DRIVETRAIN_TALON = 2;
    int LEFT_DRIVETRAIN_VICTOR = 3;
    int RIGHT_DRIVETRAIN_VICTOR = 4;

    // double TRACK_WIDTH_METERS = .60;
    // double MAX_SPEED_MPS = 4.67;
    // double MAX_ACCELERATION_MPSS = 3;
    double WHEEL_CIRCUMFERENCE = Units.inchesToMeters(6) * Math.PI;

    // double MAX_DIFF_ACCELERATION = 0;

    double TIME_TO_FULL_SPEED = .25;
    double TIME_PER_LOOP = .02;
}

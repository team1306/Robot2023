package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.MotorUtils;

/**
 * Controls the elevator portion for the robot, using a PID controller to extend to specific positions, with a manual
 * controller where the speed is bound to some modification of the input from a controller
 */
public class Elevator extends SubsystemBase {

    // stores useful constants related to the elevator subsystem
    public interface ElevatorConstants {
        // TODO find proper values for all constants
        // PID gains
        double KP = 0;
        double KI = 0;
        double KD = 0;
        // trapezoidal profile configurations (limits on PID controller)
        double MaxVelocityRadPerSec = 0;
        double MaxAccelRadPerSec2 = 0;
        // software bounds of rotation for the arm's motor.
        double MinAngle = 0;
        double MaxAngle = 0;
    }

    private WPI_TalonFX elevatorMotor = MotorUtils.initWPITalonFX(Constants.TALON_CENTER_ARM);

    private ProfiledPIDController controller = new ProfiledPIDController(
        ElevatorConstants.KP,
        ElevatorConstants.KI,
        ElevatorConstants.KD,
        new TrapezoidProfile.Constraints(
            ElevatorConstants.MaxVelocityRadPerSec,
            ElevatorConstants.MaxAccelRadPerSec2
        )
    );

    public Elevator() {
        // reset encoder and controller
        elevatorMotor.setSelectedSensorPosition(0);
        controller.reset(elevatorMotor.getSelectedSensorPosition());
    }


    // TODO make control methods
}

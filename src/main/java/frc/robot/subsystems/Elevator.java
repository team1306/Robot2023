package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
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
        double KP = 0.2;
        double KI = 0;
        double KD = 0;
        // trapezoidal profile configurations (limits on PID controller)
        double MaxVelocity = 3;
        double MaxAccel = 0.5;
        // software bounds of rotation for the arm's motor.
        double MinAngle = 0;
        double MaxAngle = 0;
    }

    // TODO find proper values
    private SlewRateLimiter filter = new SlewRateLimiter(0.5);
    private State goal = new State(0, 0);
    private WPI_TalonFX elevatorMotor = MotorUtils.initWPITalonFX(Constants.TALON_CENTER_ARM);

    private ProfiledPIDController controller = new ProfiledPIDController(
        ElevatorConstants.KP,
        ElevatorConstants.KI,
        ElevatorConstants.KD,
        new TrapezoidProfile.Constraints(ElevatorConstants.MaxVelocity, ElevatorConstants.MaxAccel)
    );

    // mode the subsystem is in
    private boolean isClosedLoop;

    public Elevator() {
        // reset encoder and controller
        elevatorMotor.setSelectedSensorPosition(0);
        controller.reset(elevatorMotor.getSelectedSensorPosition());
    }

    public void automate() {
        isClosedLoop = true;
        controller.reset(elevatorMotor.getSelectedSensorPosition());
    }

    public void manualize() {
        isClosedLoop = false;
        controller.setGoal(new State());
    }

    /**
     * return the position of the elevator
     * 
     * @return the sensor position (or some modification of it)
     */
    private double getPosition() {
        return elevatorMotor.getSelectedSensorPosition();
    }

    /**
     * manually control the speed of the elevator with a given output value
     * 
     * @param output desired output for the elevator
     */
    public void runElevator(double output) {
        if (!isClosedLoop) {
            elevatorMotor.set(ControlMode.PercentOutput, filter.calculate(output));
        }
    }

    /**
     * change the goal state of the elevator
     * 
     * @param goal new goal state
     */
    public void setGoal(State newGoal) {
        this.goal = newGoal;
    }

    @Override
    public void periodic() {
        if (isClosedLoop) {
            // if there isn't a goal for some reason, then set it to the default position
            if (goal == null) {
                controller.setGoal(new State(0, 0));
            } else {
                controller.setGoal(
                    // clamp the goal position to our allowed amount
                    new State(
                        MathUtil.clamp(
                            goal.position,
                            ElevatorConstants.MinAngle,
                            ElevatorConstants.MaxAngle
                        ),
                        goal.velocity
                    )
                );
            }
            elevatorMotor.setVoltage(controller.calculate(getPosition(), controller.getGoal()));
        }
    }
}

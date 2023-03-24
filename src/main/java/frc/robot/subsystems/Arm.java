package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.MotorUtils;

/**
 * Controls the arm portion for the robot, using a PID controller to rotate to specific positions, with a manual
 * controller where the speed is bound to some modification of the input from an xbox controller
 */
public class Arm extends SubsystemBase {

    // non-manual(true) vs manual control
    private boolean isClosedLoop;

    // stores useful constants related to the arm subsystem
    public interface ArmConstants {
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

    // initalize our electronic components
    private CANSparkMax armMotor = MotorUtils.initSparkMax(Constants.SPARK_CENTER);
    // get the relative encoder for the sparkMax
    private RelativeEncoder armEncoder = armMotor.getEncoder();
    // initalize our PID controller for the motor position (wanna see difference from normal PIDController)
    private ProfiledPIDController controller = new ProfiledPIDController(
        ArmConstants.KP,
        ArmConstants.KI,
        ArmConstants.KD,
        new TrapezoidProfile.Constraints(
            ArmConstants.MaxVelocityRadPerSec,
            ArmConstants.MaxAccelRadPerSec2
        )
    );
    // set a goal position/velocity (rotation/angular velocity)
    private State goal = new State(0, 0); // TODO probably find better base state

    // limiter during manual control
    private SlewRateLimiter filter = new SlewRateLimiter(0.5); // todo find better rate limit

    public Arm() {
        // start by reseting in our base state
        resetEncoder();
    }

    // TODO make manual control method
    /**
     * zeroes the values of the motor's encoder
     */
    public void resetEncoder() {
        armEncoder.setPosition(0);
    }


    public void automate() {
        isClosedLoop = true;
        controller.reset(armEncoder.getPosition());
    }

    public void manualize() {
        isClosedLoop = false;
        controller.setGoal(new State());
    }

    /**
     * change the goal state of the arm
     * 
     * @param goal new goal state
     */
    public void setGoal(State goal) {
        this.goal = goal;
    }

    public void runArm(double output) {
        if (!isClosedLoop) {
            // set to a limited output TODO maybe make slightly differnt
            armMotor.set(filter.calculate(output) / 3);
        }
    }

    @Override
    public void periodic() {
        // run the pid controller when in closed loop
        if (isClosedLoop) {
            if (goal == null)
                controller.setGoal(new State(0, 0));
            else
                controller.setGoal(
                    // clamp the goal position to our allowed amount
                    new State(
                        MathUtil.clamp(goal.position, ArmConstants.MinAngle, ArmConstants.MaxAngle),
                        goal.velocity
                    )
                );
            // use the resulting output to run the motor
            armMotor
                .setVoltage(controller.calculate(armEncoder.getPosition(), controller.getGoal()));
        }
    }


}

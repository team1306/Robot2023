package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.MotorUtils;

public class Intake extends SubsystemBase {
    // our electronic components
    private WPI_TalonSRX left, right;
    private DoubleSolenoid dsol;

    // whether the intake is running (both initalized to false)
    private boolean running = false;
    // output of the motor when the intake is running
    private final double speed = 0.6;

    public Intake() {
        // TODO replace with correct CAN IDs
        left = MotorUtils.initWPITalonSRX(Constants.TALON_FAR_LEFT);
        right = MotorUtils.initWPITalonSRX(Constants.TALON_FAR_RIGHT);
        // probably should be inverted to produce net inward force
        right.setInverted(true);
        right.follow(left);

        // dsol = new DoubleSolenoid(
        // PneumaticsModuleType.REVPH,
        // Constants.INTAKE_EXTEND,
        // Constants.INTAKE_RETRACT
        // );

    }

    /**
     * toggle the solenoid: if retracted, extend the intake arms, otherwise retract them
     */
    public void setPnum(boolean state) {
        dsol.set(state ? Value.kForward : Value.kReverse);
    }

    /**
     * toggle whether the motors are running
     *
     * @param running
     */
    public void setRun(boolean state) {
        this.running = state;
    }

    @Override
    public void periodic() {
        // if running then spin the motors
        if (this.running) {
            // TODO find a good value
            left.set(ControlMode.PercentOutput, speed);
        } else {
            left.set(ControlMode.PercentOutput, 0);
        }
    }
}

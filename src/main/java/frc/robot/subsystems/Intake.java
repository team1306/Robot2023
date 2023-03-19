package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.MotorUtils;

public class Intake extends SubsystemBase {
    // our electronic components
    private WPI_TalonSRX left, right;
    private Solenoid solenoid;
    // whether the intake is running (both initalized to false)
    private boolean running = false;
    // output of the motor when the intake is running
    private final double speed = 0.25;

    public Intake() {
        // TODO replace with correct CAN IDs
        left = MotorUtils.initWPITalonSRX(Constants.TALON_FAR_LEFT);
        right = MotorUtils.initWPITalonSRX(Constants.TALON_FAR_RIGHT);
        // probably should be inverted to produce net inward force
        right.setInverted(true);
        right.follow(left);

        // TODO find correct channel ID
        solenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.INTAKE);
    }

    /**
     * toggle the solenoid: if retracted, extend the intake arms, otherwise retract them
     */
    public void toggleDeploy() {
        solenoid.toggle();
    }

    /**
     * toggle whether the motors are running
     * 
     * @param running
     */
    public void toggleRun(boolean running) {
        this.running = running;
    }

    @Override
    public void periodic() {
        // if running then spin the motors
        if (this.running) {
            // TODO find a good value
            left.set(ControlMode.PercentOutput, speed);
        }
    }
}

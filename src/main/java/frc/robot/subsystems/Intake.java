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
    private Solenoid leftCylinder, rightCylinder;
    // whether the intake is running/deployed (both initalized to false)
    private boolean running, lDeploy, rDeploy;

    public Intake() {
        // TODO replace with correct CAN IDs
        left = MotorUtils.initWPITalonSRX(Constants.TALON_FAR_LEFT);
        right = MotorUtils.initWPITalonSRX(Constants.TALON_FAR_RIGHT);
        // probably should be inverted to produce net inward force
        right.setInverted(true);
        right.follow(left);

        // TODO make sure ID of hub is 1 and find correct channel
        leftCylinder = new Solenoid(PneumaticsModuleType.REVPH, 0);
    }

    public void toggleDeploy(boolean lExtend, boolean rExtend) {
        // if they're the same, do nothing and exit early
        if (lDeploy == lExtend)
            return;
        if (rDeploy == rExtend)
            return;
        // if they're different than retract/extend the correct side
        lDeploy = lExtend;
        rDeploy = rExtend;

        leftCylinder.set(lExtend);
        rightCylinder.set(rExtend);
    }

    public void toggleRun(boolean running) {
        this.running = running;
    }

    @Override
    public void periodic() {
        // if running then spin the motors
        if (this.running) {
            // TODO find a good value (or use voltage instead of percentage)
            left.set(ControlMode.PercentOutput, 0.4);
        }
    }
}

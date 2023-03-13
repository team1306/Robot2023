/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.TestDriveCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utils.Controller;
import frc.robot.utils.UserAnalog;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    private final boolean RUN_AUTO = false;

    // The robot's subsystems and commands are defined here...
    // Subsystems
    private DriveTrain driveTrain;
    private Arm arm;

    // Commands
    private Command autoCommand;
    private Command driveCommand;
    private Command armCommand;

    // inputs for drive train
    private UserAnalog backwardsTurbo;
    private UserAnalog forwardTurbo;
    private UserAnalog rotationDriveTrain;

    // inputs for arm
    private UserAnalog armInput;

    // shuffleboard input
    public static UserAnalog maxSpeed, maxRotation;

    // gyro
    public static final AHRS navx = new AHRS();

    // https://www.baeldung.com/java-static-instance-initializer-blocks
    static {
        double defaultSpd = 0.3, defaultRot = 0.3;
        String spdKey = "Max Speed", rotKey = "Max Rotation";
        // creates boxes in shuffleboard
        SmartDashboard.putNumber(spdKey, defaultSpd);
        SmartDashboard.putNumber(rotKey, defaultRot);
        // create listeners
        maxSpeed = () -> SmartDashboard.getNumber(spdKey, defaultSpd);
        maxRotation = () -> SmartDashboard.getNumber(rotKey, defaultRot);
    }

    // The robot's inputs that it recieves from the controller are defined here

    /**
     * The container for the robot. Contains subsystems, OI devices.
     */
    public RobotContainer() {
        // initalize controller (sets correct usb ports)
        Controller.init();
        // bind buttons
        configureButtonBindings();
        // create subsytems 
        driveTrain = new DriveTrain();
        arm = new Arm();

        // create commands
        driveCommand = new DriveCommand(
            driveTrain,
            backwardsTurbo,
            forwardTurbo,
            rotationDriveTrain
        );
        armCommand = new ArmCommand(arm, armInput);

        // hold B button to autobalance (attempt to get to )
        Controller.bindCommand(
            Controller.PRIMARY,
            Controller.BUTTON_B,
            // TODO maybe make pitch instead for new robot
            new BalanceCommand(navx.getRoll(), driveTrain)
        );

        // click X button to use test mode (control each side of robot)
        Controller.bindCommand(
            Controller.PRIMARY,
            Controller.BUTTON_X,
            // left trigger -> left, right trigger -> right side
            new TestDriveCommand(driveTrain, backwardsTurbo, forwardTurbo)
        );

        driveTrain.setDefaultCommand(driveCommand);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by instantiating a
     * {@link GenericHID} or one of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}),
     * and then passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // forwards/backwards input
        backwardsTurbo = Controller.simpleAxis(Controller.PRIMARY, Controller.AXIS_LTRIGGER);
        forwardTurbo = Controller.simpleAxis(Controller.PRIMARY, Controller.AXIS_RTRIGGER);
        rotationDriveTrain = Controller.simpleAxis(Controller.PRIMARY, Controller.AXIS_LX);
        // arm input
        armInput = Controller.simpleAxis(Controller.PRIMARY, Controller.AXIS_RY);
    }

    /**
     * called when autonomous is started should create all commands that are used in auto
     */
    public void startAuto() {
        autoCommand = getAutonomousCommand();
        if (RUN_AUTO) {
            driveCommand.cancel();
            autoCommand.schedule();
        }
        // System.out.println("startAuto() IS RUNNING.");
    }

    /**
     * start off teleop period by cancelling autonomous command and switching the drivetrain command to the user driving
     * command
     * 
     */
    public void startTeleop() {
        // This makes sure that the autonomous stops running when teleop starts running.
        // If you want the autonomous to
        // continue until interrupted by another command, remove this line or comment it
        // out.
        if (RUN_AUTO)
            autoCommand.cancel();

        driveCommand.schedule();
        armCommand.schedule();
    }

    public Command getAutonomousCommand() {
        // no auto command just yet
        return null;
    }

}

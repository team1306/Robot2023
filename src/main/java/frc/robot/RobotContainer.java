/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.ResourceBundle.Control;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.ElevatorCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Intake;
import frc.robot.utils.Controller;
import frc.robot.utils.UserAnalog;
import frc.robot.utils.UserDigital;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    // whether or not to run the autonomous command
    private final boolean RUN_AUTO = false;
    // toggle for things only done during testing vs in competition
    private final boolean DevMode = true;

    // The robot's subsystems and commands are defined here...
    // Subsystems
    private DriveTrain driveTrain;
    private Arm arm;
    private Elevator elevator;
    private Intake intake;
    private Grabber grabber;

    // Commands
    private Command autoCommand, driveCommand, elevatorCommand, intakeCommand;

    // inputs for drive train
    private UserAnalog backwardsTurbo;
    private UserAnalog forwardTurbo;
    private UserAnalog rotationDriveTrain;

    // inputs for arm
    private UserAnalog elevatorInput, armInput;

    // inputs for intake
    private UserDigital toggleWheels, togglePnum;

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
        elevator = new Elevator();
        intake = new Intake();
        grabber = new Grabber();

        // create commands
        // drive command
        driveCommand = new DriveCommand(
            driveTrain,
            backwardsTurbo,
            forwardTurbo,
            rotationDriveTrain
        );
        driveTrain.setDefaultCommand(driveCommand);

        // elevator
        elevatorCommand = new ElevatorCommand(elevator, elevatorInput);

        // intake

        // grabber

        // autobalance command
        // hold B button to autobalance (attempt to get to level state -- start on balance beam)
        Controller.bindCommand(
            Controller.PRIMARY,
            Controller.BUTTON_B,
            // since RIO lying face up, it should use the pitch value
            // using current measurement as baseline (flat) value to compare to
            new BalanceCommand(DriveTrain.gyro.getPitch(), driveTrain)
        );

        if (DevMode) {
            // testdrive command, don't use in competition
            // click X button to use test mode (control each side of robot)
            Controller.bindCommand(
                Controller.PRIMARY,
                Controller.BUTTON_X,
                // left trigger -> left, right trigger -> right side
                driveTrain.testDrive(backwardsTurbo, forwardTurbo)
            );
        }


    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by instantiating a
     * {@link GenericHID} or one of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}),
     * and then passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // drivetrain inputs
        backwardsTurbo = Controller.simpleAxis(Controller.PRIMARY, Controller.AXIS_LTRIGGER);
        forwardTurbo = Controller.simpleAxis(Controller.PRIMARY, Controller.AXIS_RTRIGGER);
        rotationDriveTrain = Controller.simpleAxis(Controller.PRIMARY, Controller.AXIS_LX);

        // intake inputs TODO work in progress, gotta find actual ones with Chris
        togglePnum = Controller.simpleButton(Controller.PRIMARY, Controller.BUTTON_LBUMPER);
        toggleWheels = Controller.simpleButton(Controller.PRIMARY, Controller.BUTTON_RBUMPER);

        // arm input
        elevatorInput = Controller.simpleAxis(Controller.SECONDARY, Controller.AXIS_RY);

    }

    /**
     * called when autonomous is started should create all commands that are used in auto
     */
    public void startAuto() {
        autoCommand = getAutonomousCommand();
        if (RUN_AUTO) {
            // driveCommand.cancel();
            autoCommand.schedule();
        }
    }

    /**
     * start off teleop period by cancelling autonomous command and switching the drivetrain command to the user driving
     * command
     */
    public void startTeleop() {
        // This makes sure that the autonomous stops running when teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove this or comment it out.
        if (RUN_AUTO) {
            autoCommand.cancel();
        }

    }

    public Command getAutonomousCommand() {
        // TODO implement proper command sequence
        // placeholder example: just drive forwards for 3 seconds
        return Commands.run(() -> driveTrain.arcadeDrive(1, 0)).withTimeout(3);
    }
}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private boolean RUN_AUTO = true;
    // toggle for things only done during testing vs in competition
    private final boolean DevMode = false;

    // The robot's subsystems and commands are defined here...
    // Subsystems
    private DriveTrain driveTrain;
    private Arm arm;
    private Elevator elevator;
    private Intake intake;
    private Grabber grabber;

    // Commands
    private Command autoCommand, elevatorCommand, intakeCommand;
    private DriveCommand driveCommand;

    // inputs for drive train
    private UserAnalog backwardsTurbo;
    private UserAnalog forwardTurbo;
    private UserAnalog rotationDriveTrain;

    private UserDigital turbo;
    // inputs for arm
    private UserAnalog elevatorInput, armInput;

    // inputs for intake
    private UserDigital toggleWheels, togglePnum;

    // inputs for grabber
    private UserDigital toggleGrabber;


    private SendableChooser<Command> autos = new SendableChooser<>();

    // The robot's inputs that it recieves from the controller are defined here

    /**
     * The container for the robot. Contains subsystems, OI devices.
     */
    public RobotContainer() {
        // initalize controller (sets correct usb ports)
        Controller.init();
        // bind buttons
        configureButtonBindings();
        // create auto bindings

        // create subsytems
        driveTrain = new DriveTrain();
        // arm = new Arm();
        // elevator = new Elevator();
        // intake = new Intake();
        // grabber = new Grabber();

        // create commands
        // drive command
        driveCommand = new DriveCommand(
            driveTrain,
            backwardsTurbo,
            forwardTurbo,
            rotationDriveTrain
        );
        driveTrain.setDefaultCommand(driveCommand);

        // turbo button
        Controller.asTrigger(turbo).debounce(0.05).toggleOnTrue(Commands.startEnd(() -> {
            driveCommand.turbo = false;
            SmartDashboard.putBoolean("isTurbo", false);
        }, () -> {
            driveCommand.turbo = true;
            SmartDashboard.putBoolean("isTurbo", true);
        }));

        // elevator
        // elevatorCommand = new ElevatorCommand(elevator, elevatorInput);

        // intake
        // if we want independent toggle control: make toggleDeploy/toggleRun take a parameter true/false
        // var depTrigger = Controller.asTrigger(togglePnum).debounce(0.05);
        // // toggling toggles pneumatics,
        // depTrigger.toggleOnTrue(
        // Commands.startEnd(() -> intake.setPnum(true), () -> intake.setPnum(false), intake)
        // );

        // arm
        // var armcom = new ArmCommand(arm, armInput);
        // var runTrigger = Controller.asTrigger(toggleWheels).debounce(0.05);
        // // toggling intake runner
        // runTrigger.toggleOnTrue(
        // Commands.startEnd(() -> intake.setRun(true), () -> intake.setRun(false), intake)
        // );

        // grabber
        // var grabberToggle = Controller.asTrigger(toggleGrabber).debounce(0.05);
        // // toggling intake runner
        // grabberToggle.toggleOnTrue(
        // Commands.startEnd(() -> grabber.set(true), () -> grabber.set(false), grabber)
        // );

        // autobalance command
        // hold B button to autobalance (attempt to get to level state -- start on balance beam)
        // Controller.bindCommand(
        // Controller.PRIMARY,
        // Controller.BUTTON_B,
        // // since RIO lying face up, it should use the pitch value
        // // using current measurement as baseline (flat) value to compare to
        // new BalanceCommand(DriveTrain.gyro.getRoll(), driveTrain)
        // );

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

        // auto choices

        var nothing = Commands.none();
        var forward = Commands
            .startEnd(
                () -> driveTrain.arcadeDrive(0.5, 0),
                () -> driveTrain.arcadeDrive(0, 0),
                driveTrain
            )
            .withTimeout(2);
        var backward = Commands
            .startEnd(
                () -> driveTrain.arcadeDrive(-0.5, 0),
                () -> driveTrain.arcadeDrive(0, 0),
                driveTrain
            )
            .withTimeout(2);
        var balance = Commands
            .startEnd(
                () -> driveTrain.arcadeDrive(0.3, 0),
                () -> driveTrain.arcadeDrive(0, 0),
                driveTrain
            )
            .withTimeout(2)
            .andThen(new BalanceCommand(0, driveTrain));

        autos.setDefaultOption("do nothing", nothing);
        autos.addOption("driveForward", forward);
        autos.addOption("driveBackward", backward);
        autos.addOption("forward balance", balance);

        SmartDashboard.putData(autos);


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

        turbo = Controller.simpleButton(Controller.PRIMARY, Controller.BUTTON_X);
        // intake inputs TODO work in progress, gotta find actual ones with Chris

        // togglePnum = Controller.simpleButton(Controller.PRIMARY, Controller.BUTTON_Y);
        // toggleWheels = Controller.simpleButton(Controller.PRIMARY, Controller.BUTTON_A);

        // elevator input
        // elevatorInput = Controller.simpleAxis(Controller.SECONDARY, Controller.AXIS_RY);

        // arm input
        // armInput = Controller.simpleAxis(Controller.SECONDARY, Controller.AXIS_LY);

        // grabber input
        // toggleGrabber = Controller.simpleButton(Controller.SECONDARY, Controller.BUTTON_B);
    }

    /**
     * called when autonomous is started should create all commands that are used in auto
     */
    public void startAuto() {
        autoCommand = getAutonomousCommand();
        if (RUN_AUTO && autoCommand != null) {
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
        if (RUN_AUTO && autoCommand != null) {
            autoCommand.cancel();
        }

    }

    public Command getAutonomousCommand() {
        return autos.getSelected();
    }
}

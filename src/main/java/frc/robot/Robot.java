
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.vision.VisionThread;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.apriltag.AprilTagDetector.Config;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the TimedRobot documentation. If you change the name of this class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */
public class Robot extends TimedRobot {

    public static DriveTrain driveTrain;
    private RobotContainer m_robotContainer = new RobotContainer();

    // messing around w/ apriltag detector & video stuff
    private AprilTagDetector detector = new AprilTagDetector();

    private CvSource out, out2;
    private VisionThread aprilThread, coneThread;

    /**
     * This function is run when the robot is first started up and should be used for any initialization code.
     */
    @Override
    public void robotInit() {
        detector.addFamily("tag16h5");
        var config = new Config();
        config.numThreads = 4;
        config.quadDecimate = 1;
        config.quadSigma = 0;
        config.refineEdges = true;
        detector.setConfig(config);

        // get camera input
        var cam = CameraServer.startAutomaticCapture();
        cam.setResolution(640, 480);

        // output video stream, one for color & markings, one for black & white
        out = CameraServer.putVideo("out", 480, 270);

        // seperate thread for apriltag detection
        aprilThread = new VisionThread(cam, mat -> {
            // create matrix for modifications, and copy input from camera
            Imgproc.cvtColor(mat, mat, Imgproc.COLOR_BGR2GRAY);
            // detect tags
            var dets = detector.detect(mat);
            for (var det : dets) {
                // only valid tags from 1 to 8
                if (det.getId() > 8 || det.getId() < 1)
                    continue;
                // run pose estimator
                // ...
            }
            // output frame to camera server in shuffleboard
            out.putFrame(mat);
        }, a -> {});
        aprilThread.setDaemon(true);
        aprilThread.start();
    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for items like diagnostics that you want
     * ran during disabled, autonomous, teleoperated and test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow and SmartDashboard integrated
     * updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled
        // commands, running already- scheduled commands, removing finished or
        // interrupted commands,
        // and running subsystem periodic() methods. This must be called from the
        // robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
        // use a max speed/rotation provided in shuffleboard
    }

    /**
     * This function is called once each time the robot enters Disabled mode.
     */
    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    /**
     * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        // m_robotContainer.startAuto();
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void teleopInit() {
        m_robotContainer.startTeleop();
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {

    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {}

    @Override
    public void simulationPeriodic() {
        CommandScheduler.getInstance().run();
    }
}
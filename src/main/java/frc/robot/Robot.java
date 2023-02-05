
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
        // configure apriltag detector
        detector.addFamily("tag16h5");
        var config = new Config();
        config.numThreads = 4;
        config.quadDecimate = 1;
        config.quadSigma = 0;
        config.refineEdges = true;
        detector.setConfig(config);

        // get camera input
        var cam = CameraServer.startAutomaticCapture();
        cam.setResolution(480, 270);

        // output video stream, one for color & markings, one for black & white
        out = CameraServer.putVideo("out", 480, 270);
        out2 = CameraServer.putVideo("bw", 480, 270);

        // seperate thread for apriltag detection
        aprilThread = new VisionThread(cam, mat -> {
            // create matrix for modifications, and copy input from camera
            Mat tempMat = Mat.zeros(mat.size(), mat.type());
            mat.copyTo(tempMat);
            // convert to grayscale and split by brightness: => 120 -> white and < 120 -> black
            Imgproc.cvtColor(tempMat, tempMat, Imgproc.COLOR_BGR2GRAY);
            Imgproc.threshold(tempMat, tempMat, 120, 255, Imgproc.THRESH_BINARY);
            // detect tags
            var dets = detector.detect(tempMat);
            for (var det : dets) {
                // only valid tags from 1 to 8
                if (det.getId() > 8 || det.getId() < 1)
                    continue;
                // draw center marking
                Imgproc.drawMarker(
                    mat,
                    new Point(det.getCenterX(), det.getCenterY()),
                    new Scalar(0, 255, 0)
                );
                // corners are stored as x1, y1, x2, y2, etc.
                for (int i = 0; i < det.getCorners().length / 2; i++) {
                    // draw corner markings
                    Imgproc.drawMarker(
                        mat,
                        new Point(det.getCornerX(i), det.getCornerY(i)),
                        new Scalar(0, 0, 255)
                    );
                }
            }
            // output frames to camera server in shuffleboard
            out.putFrame(mat);
            out2.putFrame(tempMat);
        }, a -> {});
        // aprilThread.start();

        // seperate thread for cone detection (primarily for the fun lol)
        coneThread = new VisionThread(cam, mat -> {
            // look for yellow pixels in picture
            var mat2 = mat.clone();
            Core.inRange(mat, new Scalar(0, 75, 75), new Scalar(75, 255, 255), mat);
            Imgproc.GaussianBlur(mat, mat, new Size(41, 41), 0);
            Imgproc.threshold(mat, mat, 120, 255, Imgproc.THRESH_BINARY);
            var moments = Imgproc.moments(mat);
            var x = moments.get_m10() / moments.get_m00();
            var y = moments.get_m01() / moments.get_m00();
            Imgproc.circle(mat2, new Point(x, y), 5, new Scalar(0, 255, 0), -1);
            // Core.inRange(mat, new Scalar(138, 105, 0), new Scalar(255, 250, 101), mat);
            out.putFrame(mat);
            out2.putFrame(mat2);
        }, a -> {});
        // coneThread.start();
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
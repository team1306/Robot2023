package frc.robot.commands;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class VisionCommand extends CommandBase {
    private final CvSink cvSink = new CvSink("apriltag detector runner");
    private Mat m = new Mat();

    public VisionCommand(VideoSource source) {
        cvSink.setSource(source);
    }

    @Override
    public void execute() {
        Mat m = new Mat();
        var time = cvSink.grabFrame(m);
        if (time == 0)
            System.out.println("uh oh");
        // might be different
        Core.rotate(m, m, Core.ROTATE_90_COUNTERCLOCKWISE);
        Imgproc.cvtColor(m, m, Imgproc.COLOR_BGR2GRAY);
        Imgproc.threshold(m, m, 120, 255, Imgproc.THRESH_BINARY);
    }

}

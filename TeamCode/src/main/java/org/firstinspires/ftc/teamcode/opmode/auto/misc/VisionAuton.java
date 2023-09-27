package org.firstinspires.ftc.teamcode.opmode.auto.misc;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystems.vision.ff.Vision;
import org.firstinspires.ftc.teamcode.util.misc.Util;
import org.firstinspires.ftc.teamcode.util.teleop.MatchOpMode;

import java.util.logging.Level;
@Autonomous(name = "Blue Carousel", group = "BLUE")
public class VisionAuton extends MatchOpMode {
public static double startPoseX = 0;
public static double startPoseY = 0;
public static double startPoseHeading = 0;

// Motor
    private Vision vision;

@Override
public void robotInit() {

    vision = new Vision(hardwareMap,  telemetry);
}

@Override
public void disabledPeriodic() {
    Util.logger(this, telemetry, Level.INFO, "Current Position", vision.getCurrentPosition());
    vision.periodic();
}

@Override
public void matchStart() {
    schedule(
//            new SelectCommand(new HashMap<Object, Command>() {{
//                put(TeamMarkerPipeline.Position.LEFT, new SequentialCommandGroup(
//                );
//                put(TeamMarkerPipeline.Position.MIDDLE, new SequentialCommandGroup(
//                );
//                put(TeamMarkerPipeline.Position.RIGHT, new SequentialCommandGroup(
//                );
//            }}, vision::getCurrentPosition)
    );
    }
}
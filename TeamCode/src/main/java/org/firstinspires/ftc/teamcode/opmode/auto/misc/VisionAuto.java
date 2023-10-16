package org.firstinspires.ftc.teamcode.opmode.auto.misc;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystems.vision.ff.Vision;
import org.firstinspires.ftc.teamcode.util.teleop.MatchOpMode;
@Autonomous
public class VisionAuto extends MatchOpMode {
    private Vision vision;

@Override
public void robotInit() {
    vision = new Vision(hardwareMap,  telemetry);
}

@Override
public void disabledPeriodic() {
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
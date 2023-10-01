package org.firstinspires.ftc.teamcode.subsystems.vision.ff;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.misc.Util;

import java.util.logging.Level;


public class Vision extends SubsystemBase {
    private Telemetry telemetry;
    private final FFRectDetector duckDetector;
    private TeamMarkerPipeline.Position currentPos;

    public Vision(HardwareMap hw, Telemetry tl) {
        duckDetector = new FFRectDetector(hw, tl);
        duckDetector.init();

        duckDetector.setLeftRectangle(0.12, 0.25);
        duckDetector.setCenterRectangle(0.51, .25);
        duckDetector.setRightRectangle(0.90, .25);
        duckDetector.setRectangleSize(1,1);
        telemetry = tl;
        currentPos = duckDetector.getPosition();
    }


    @Override
    public void periodic() {
        currentPos = duckDetector.getPosition();
        Util.logger(this, telemetry, Level.INFO, "Duck Position: ", duckDetector.getPosition());
    }

    public TeamMarkerPipeline.Position getCurrentPosition() {
        return currentPos;
    }
}
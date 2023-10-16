package org.firstinspires.ftc.teamcode.subsystems.vision.ff;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Vision extends SubsystemBase {
    private final Telemetry telemetry;
    private final FFRectDetector duckDetector;
//    private TeamMarkerPipeline.Position currentPos;

    public Vision(HardwareMap hw, Telemetry tl) {
        duckDetector = new FFRectDetector(hw, tl);
        duckDetector.init();

        duckDetector.setLeftRectangle(.4, .5);
        duckDetector.setCenterRectangle(.3, .2);
        duckDetector.setRightRectangle(.8, .5);
        duckDetector.setRectangleSize(50,50);
        telemetry = tl;
//        currentPos = duckDetector.getPosition();
    }


    @Override
    public void periodic() {
//        currentPos = duckDetector.getPosition();
        telemetry.addData("Position", duckDetector.getPosition());
    }

    public TeamMarkerPipeline.FFPosition getPosition() {
        return duckDetector.getPosition();
    }
}
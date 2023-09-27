package org.firstinspires.ftc.teamcode.opmode.auto.misc;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.vision.ff.FFRectDetector;
import org.firstinspires.ftc.teamcode.util.teleop.MatchOpMode;

@Autonomous(group = "RED/BLUE")
public class TestAutonWithoutCam extends MatchOpMode {

    // Subsystems
//    private Pivot pivot;
//    private Claw claw;
//    private Drivetrain drivetrain;
//    private Slide slide;
//    private TurnServo turnServo;
//    private SensorColor sensorColor;
    private HardwareMap hardware;
    private final FFRectDetector detector = new FFRectDetector(hardwareMap, telemetry);

//    public MecanumDrive mecanumDrive;


    @Override
    public void robotInit() {
//        claw = new Claw( telemetry, hardwareMap);
//        pivot = new Pivot(telemetry, hardwareMap);
//        slide = new Slide(telemetry, hardwareMap);
//        turnServo = new TurnServo(telemetry, hardwareMap);
//        sensorColor = new SensorColor(hardwareMap, telemetry);
//        drivetrain = new Drivetrain(new SixWheel(hardwareMap), telemetry);
//        drivetrain.init();
        detector.setLeftRectangle(0.2,32);
        detector.setLeftRectangle(1,32);
        detector.setLeftRectangle(8,32);
        detector.init();
    }


    public void matchStart() {
        telemetry.addData("Location:", detector.getPosition());
//        waitForStart();
        schedule(
                new SequentialCommandGroup(
                )
        );
//        PoseStorage.currentPose = drivetrain.getPoseEstimate();
    }


};
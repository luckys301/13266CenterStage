package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.drive.teleop.DefaultTankDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drive.SixWheel;
import org.firstinspires.ftc.teamcode.subsystems.old.Claw;
import org.firstinspires.ftc.teamcode.subsystems.old.Pivot;
import org.firstinspires.ftc.teamcode.subsystems.old.Slide;
import org.firstinspires.ftc.teamcode.subsystems.old.TurnServo;
import org.firstinspires.ftc.teamcode.util.teleop.MatchOpMode;

@Config
@TeleOp
public class TeleOpMain extends MatchOpMode {

    // Gamepad
    private GamepadEx driverGamepad;
    private GamepadEx operatorGamepad;


    // Subsystems
    private Pivot pivot;
    private Claw claw;
    private Drivetrain drivetrain;
    private Slide slide;
    private TurnServo turnServo;

    @Override
    public void robotInit() {
        driverGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);

        pivot = new Pivot(telemetry, hardwareMap);
        claw = new Claw(telemetry, hardwareMap);
        turnServo = new TurnServo(telemetry, hardwareMap);
        drivetrain = new Drivetrain(new SixWheel(hardwareMap), telemetry);
        drivetrain.init();
        slide = new Slide(telemetry, hardwareMap);
//        pivot.resetOffset();
        pivot.moveInitializationPosition();
    }


    @Override
    public void configureButtons() {
//        Button up = (new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_UP)
//                .whenPressed(new InstantCommand(()->turnServo.setClawS3(turnServo.getPos()+0.05))));
//
//        Button down = (new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_DOWN)
//                .whenPressed(new InstantCommand(()->turnServo.setClawS3(turnServo.getPos()-0.05))));
        /*
         *  DRIVER
         */
        drivetrain.setDefaultCommand(new DefaultTankDriveCommand(drivetrain, driverGamepad));

////        Button recenterIMU = (new GamepadButton(driverGamepad, GamepadKeys.Button.A))
////                .whenPressed(new InstantCommand(drivetrain::reInitializeIMU));
////
////        Button recenterIMU2 = (new GamepadButton(driverGamepad, GamepadKeys.Button.START))
////                .whenPressed(new InstantCommand(drivetrain::reInitializeIMU));
//
//        Button slowMode = (new GamepadButton(driverGamepad, GamepadKeys.Button.LEFT_BUMPER))
//                .whileHeld(new SlowDriveCommand(drivetrain, driverGamepad, true));
//
//        /*
//         * OPERATOR
//         */
//
//        slide.setDefaultCommand(new SlideMoveManual(slide, operatorGamepad::getRightY));
//
//        pivot.setDefaultCommand(new PivotMoveManual(pivot, operatorGamepad::getLeftY));
//
//        Button armIntake = (new GamepadTrigger(operatorGamepad, GamepadKeys.Trigger.LEFT_TRIGGER))
//                .whenPressed(new PickConeCommand(claw));
//
//        Button armOuttake = (new GamepadTrigger(operatorGamepad, GamepadKeys.Trigger.RIGHT_TRIGGER))
//                .whenPressed(new DropConeCommand(claw, slide, pivot));
//
////        Button armGroundFront = (new GamepadButton(operatorGamepad, GamepadKeys.Button.X)
////                .whenPressed(new ArmGroundFrontCommand(slide, pivot, claw, turnServo, false)));
//
//        Button armLowFront = (new GamepadButton(operatorGamepad, GamepadKeys.Button.DPAD_LEFT)
//                .whenPressed(new ArmLowFrontCommand(slide, pivot, claw, turnServo, false)));
//
//        Button armMidFront = (new GamepadButton(operatorGamepad, GamepadKeys.Button.DPAD_RIGHT)
//                .whenPressed(new ArmMidFrontCommand(slide, pivot, claw, turnServo, false)));
//
//        Button armHighFront = (new GamepadButton(operatorGamepad, GamepadKeys.Button.DPAD_UP)
//                .whenPressed(new ArmHighFrontCommand(slide, pivot, claw, turnServo, false)));
//
//        Button armIntakeBack = (new GamepadButton(operatorGamepad, GamepadKeys.Button.DPAD_DOWN)
//                .whenPressed(new ArmIntakeBackCommand(slide, pivot, claw, turnServo)));
//
//        Button pivotInitializationPosition = (new GamepadButton(operatorGamepad, GamepadKeys.Button.A)
//                .whenPressed(pivot::encoderReset));
//
//        Button slideRecenter = (new GamepadButton(operatorGamepad, GamepadKeys.Button.B))
//                .whenPressed(slide::encoderRecenter);

//        Button pivotRecenter = (new GamepadButton(operatorGamepad, GamepadKeys.Button.START))
//                .whenPressed(pivot::encoderReset);
    }

    @Override
    public void matchLoop() {}
    @Override
    public void disabledPeriodic() { }
    @Override
    public void matchStart() { }
    @Override
    public void robotPeriodic(){
    }
}

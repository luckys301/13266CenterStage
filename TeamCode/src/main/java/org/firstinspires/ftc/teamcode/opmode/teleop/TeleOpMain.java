package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.arm.slide.SlideMoveManual;
import org.firstinspires.ftc.teamcode.commands.drive.teleop.DefaultDriveCommand;
import org.firstinspires.ftc.teamcode.commands.drive.teleop.SlowDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.arm.Arm;
import org.firstinspires.ftc.teamcode.subsystems.arm.MotorArm;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.teleop.MatchOpMode;

@Config
@TeleOp
public class TeleOpMain extends MatchOpMode {

    // Gamepad
    private GamepadEx driverGamepad;
    private GamepadEx operatorGamepad;


    // Subsystems
    private Drivetrain drivetrain;
//    private Slide slide;
//    private Intake intake;
    private MotorArm arm;
//    private Claw claw;
//    private Shooter shooter;

    @Override
    public void robotInit() {
        driverGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);

//        claw = new Claw(telemetry, hardwareMap, true);
        drivetrain = new Drivetrain(new MecanumDrive(hardwareMap, telemetry), telemetry, hardwareMap);
        drivetrain.init();
//        intake = new Intake(telemetry, hardwareMap, true);
        arm = new MotorArm(telemetry, hardwareMap);
//        shooter = new Shooter(telemetry, hardwareMap, true);
//        slide = new Slide(telemetry, hardwareMap, false);
    }


    @Override
    public void configureButtons() {
//        //Claw
//        Button up = (new GamepadTrigger(driverGamepad, GamepadKeys.Trigger.LEFT_TRIGGER)
//                .whenPressed(claw.setClawPos(Claw.ClawPos.OPEN_POS)));
//        Button close = (new GamepadButton(driverGamepad,  Button.DPAD_DOWN)
//                .whenPressed(claw.setClawPos(Claw.ClawPos.CLOSE_POS)));
//
//        //Arm
        Button armTransfer = (new GamepadButton(operatorGamepad, GamepadKeys.Button.DPAD_DOWN))
                .whenPressed(arm.setSetPointCommand(MotorArm.ArmEnum.TRANSFER));
        Button armOuttake = (new GamepadButton(operatorGamepad, GamepadKeys.Button.DPAD_UP))
                .whenPressed(arm.setSetPointCommand(MotorArm.ArmEnum.LOW));
        Button fff = (new GamepadButton(operatorGamepad, GamepadKeys.Button.DPAD_RIGHT))
            .whenPressed(arm.setSetPointCommand(MotorArm.ArmEnum.MID));
        Button kklk = (new GamepadButton(operatorGamepad, GamepadKeys.Button.DPAD_LEFT))
            .whenPressed(arm.setSetPointCommand(MotorArm.ArmEnum.HIGH));
//
//        //Intake
//        Trigger INTAKE = (new GamepadTrigger(driverGamepad, GamepadKeys.Trigger.LEFT_TRIGGER)
//                .whileHeld(intake.setSetPointCommand(Intake.IntakeRPM.INTAKE)))
//                .whenReleased(intake.setSetPointCommand(Intake.IntakeRPM.STOP));
//        Trigger outtake = (new GamepadTrigger(driverGamepad, GamepadKeys.Trigger.RIGHT_TRIGGER)
//                .whileHeld(intake.setSetPointCommand(Intake.IntakeRPM.OUTTAKE)))
//                .whenPressed(cycleTracker.trackCycle())
//                .whenReleased(intake.setSetPointCommand(Intake.IntakeRPM.STOP));
//
//
//        //Shooter
//        com.arcrobotics.ftclib.command.button.Button shoot = (new GamepadButton(operatorGamepad, Button.RIGHT_BUMPER))
//                .whenPressed(shooter.shoot());

        //Climber


        //Slide
//        Button slideRest  = (new GamepadButton(operatorGamepad, GamepadKeys.Button.A))
//                .whenPressed(slide.setSetPointCommand(Slide.SlideEnum.TRANSFER));
//        Button slideLow  = (new GamepadButton(operatorGamepad, GamepadKeys.Button.X))
//                .whenPressed(slide.setSetPointCommand(Slide.SlideEnum.LOW));
//        Button slideMid  = (new GamepadButton(operatorGamepad, GamepadKeys.Button.B))
//                .whenPressed(slide.setSetPointCommand(Slide.SlideEnum.MID));
//        Button slideHigh  = (new GamepadButton(operatorGamepad, GamepadKeys.Button.Y))
//                .whenPressed(slide.setSetPointCommand(Slide.SlideEnum.HIGH));


//        Button down = (new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_DOWN)
//                .whenPressed(new InstantCommand(()->turnServo.setClawS3(turnServo.getPos()-0.05))));
        /*
         *  DRIVER
         */

        drivetrain.setDefaultCommand(new DefaultDriveCommand(drivetrain, driverGamepad, true));

        Button recenterIMU = (new GamepadButton(driverGamepad, GamepadKeys.Button.A))
                .whenPressed(new InstantCommand(drivetrain::reInitializeIMU));

        Button slowMode = (new GamepadButton(driverGamepad, GamepadKeys.Button.LEFT_BUMPER))
                .whileHeld(new SlowDriveCommand(drivetrain, driverGamepad, true));

        /*
         * OPERATOR
         */

//        slide.setDefaultCommand(new SlideMoveManual(slide, operatorGamepad::getRightY));
//        pivot.setDefaultCommand(new PivotMoveManual(pivot, operatorGamepad::getLeftY));
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

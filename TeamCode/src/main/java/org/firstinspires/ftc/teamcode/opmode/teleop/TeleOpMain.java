package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Button;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.arm.slide.SlideMoveManual;
import org.firstinspires.ftc.teamcode.commands.drive.teleop.DefaultTankDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.arm.Arm;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drive.SixWheel;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.old.Slide;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.util.teleop.GamepadTrigger;
import org.firstinspires.ftc.teamcode.util.teleop.MatchOpMode;

@Config
@TeleOp
public class TeleOpMain extends MatchOpMode {

    // Gamepad
    private GamepadEx driverGamepad;
    private GamepadEx operatorGamepad;


    // Subsystems
    private Drivetrain drivetrain;
    private Slide slide;
    private Intake intake;
    private Arm arm;
    private Claw claw;
    private Shooter shooter;

    @Override
    public void robotInit() {

        driverGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);

//        pivot = new Pivot(telemetry, hardwareMap);
        claw = new Claw(telemetry, hardwareMap, true);
//        turnServo = new TurnServo(telemetry, hardwareMap);
        drivetrain = new Drivetrain(new SixWheel(hardwareMap), telemetry);
//        drivetrain.init();
//        slide = new Slide(telemetry, hardwareMap);
//        pivot.resetOffset();
//        pivot.moveInitializationPosition();
        intake = new Intake(telemetry, hardwareMap, true);
        arm = new Arm (telemetry, hardwareMap, true);
        shooter = new Shooter(telemetry, hardwareMap, true);
    }


    @Override
    public void configureButtons() {
        //Claw
        com.arcrobotics.ftclib.command.button.Button up = (new GamepadTrigger(driverGamepad, GamepadKeys.Trigger.LEFT_TRIGGER)
                .whenPressed(new InstantCommand(()->claw.clawOpen())));

        com.arcrobotics.ftclib.command.button.Button close= (new GamepadButton(driverGamepad,  Button.DPAD_DOWN)
                .whenPressed(new  InstantCommand(()->claw.clawClose())));

        //Arm
        com.arcrobotics.ftclib.command.button.Button armTransfer = (new GamepadButton(operatorGamepad, Button.DPAD_DOWN))
                .whenPressed(arm.armSetPositionCommand(Arm.ArmPos.TRANSFER));
        com.arcrobotics.ftclib.command.button.Button armOuttake = (new GamepadButton(operatorGamepad, Button.DPAD_UP))
                .whenPressed(arm.armSetPositionCommand(Arm.ArmPos.OUTTAKE));

        //Intake
        Trigger INTAKE= (new GamepadTrigger(driverGamepad, GamepadKeys.Trigger.LEFT_TRIGGER)
                .whileHeld(intake.setSetPointCommand(Intake.IntakeRPM.INTAKE)))
                .whenReleased(intake.setSetPointCommand(Intake.IntakeRPM.STOP));
        Trigger outtake= (new GamepadTrigger(driverGamepad, GamepadKeys.Trigger.RIGHT_TRIGGER)
                .whileHeld(intake.setSetPointCommand(Intake.IntakeRPM.OUTTAKE)))
                .whenReleased(intake.setSetPointCommand(Intake.IntakeRPM.STOP));


        //Shooter
        com.arcrobotics.ftclib.command.button.Button shoot = (new GamepadButton(operatorGamepad, Button.RIGHT_BUMPER))
                .whenPressed(shooter.shoot());

        //Climber


        //Slide
//        Button slideRest  = (new GamepadButton(operatorGamepad, Button.A))
//                .whenPressed(slide.setSetPositionCommand(Slide.SlidePos.REST));
//        Button slideLow  = (new GamepadButton(operatorGamepad, Button.X))
//                .whenPressed(slide.setSetPositionCommand(Slide.SlidePos.LOW));
//        Button slideMid  = (new GamepadButton(operatorGamepad, Button.B))
//                .whenPressed(slide.setSetPositionCommand(Slide.SlidePos.MID));
//        Button slideHigh  = (new GamepadButton(operatorGamepad, Button.Y))
//                .whenPressed(slide.setSetPositionCommand(Slide.SlidePos.HIGH));


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
        slide.setDefaultCommand(new SlideMoveManual(slide, operatorGamepad::getRightY));
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

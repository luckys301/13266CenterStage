package org.firstinspires.ftc.teamcode.subsystems.slide;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.NebulaConstants;

@Config
public class Slide extends SubsystemBase {
//    public final NebulaMotorGroup motorGroup;
    protected Telemetry telemetry;
    protected MotorEx slideR, slideL;
    protected PIDFController slideController;
    protected double output = 0;
//    protected boolean dropBoolean = false;

    //TODO: Should the Slide even drop?

    public enum SlideEnum {
        TRANSFER(0.0),

        LOW(-150),
        MID(-300),
        HIGH(-400),

        MANUAL(0.0);
        public final double slidePos;
        SlideEnum(double slidePos) {
            this.slidePos = slidePos;
        }
    }


    protected static SlideEnum slidePos;

    public Slide(Telemetry tl, HardwareMap hw) {
        slideR = new MotorEx(hw, "slideR");
        slideL = new MotorEx(hw, "slideL");

//        motorGroup = new NebulaMotorGroup(slideR, slideL);
        slideR.setDistancePerPulse(1);

        slideController = new PIDFController(0,0,0,0,
            getEncoderDistance(),
            getEncoderDistance());
        slideController.setTolerance(1);

        this.telemetry = tl;
        slidePos = SlideEnum.TRANSFER;
    }

    @Override
    public void periodic() {
        slideController.setF(slideController.getF() * Math.cos(Math.toRadians(slideController.getSetPoint())));
        output = slideController.calculate(getEncoderDistance());
        setPower(output);//TODO: Probably shouldn't be like this

        telemetry.addData("Slide Motor Output:", output);
        telemetry.addData("Slide1 Encoder: ", slideR.getCurrentPosition());
        telemetry.addData("Slide2 Encoder: ", slideL.getCurrentPosition());
        telemetry.addData("Slide Pos:", getSetPoint());
    }

    public double getEncoderDistance() {
        return slideR.getDistance();
//        return slideR.getPosition();
        //TODO:Does this work?
    }


    public void setPower(double power) {
        slideR.set(power);
        slideL.set(-power);
//        slideM1.setPower(power);
//        slideM2.setPower(power);//Instead of putting -power, maybe reverse the motor
    }

    public void stopSlide() {
        slideR.stopMotor();
        slideController.setSetPoint(getEncoderDistance());
    }
    /****************************************************************************************/


    public void resetEncoder() {
        slideR.resetEncoder();
        slideL.resetEncoder();
    }


    private void setSetPoint(double setPoint) {
        slideController.setSetPoint(setPoint);
    }

    //TODO: Test!
    public Command setSetPointCommand(double setPoint) {
        slidePos = SlideEnum.MANUAL;    //WTH is this; The booleans don't match
        return new InstantCommand(()->{this.setSetPoint(setPoint);});
    }
    public Command setSetPointCommand(SlideEnum pos) {
        return new InstantCommand(()->{setSetPoint(pos.slidePos);});
    }

    public double getSetPoint() {
        return slideController.getSetPoint();
    }
}
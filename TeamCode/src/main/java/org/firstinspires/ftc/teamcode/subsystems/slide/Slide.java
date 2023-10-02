package org.firstinspires.ftc.teamcode.subsystems.slide;

import static org.firstinspires.ftc.teamcode.subsystems.slide.SlideValue.SlideEnum;
import static org.firstinspires.ftc.teamcode.subsystems.slide.SlideValue.make;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.NebulaConstants;
import org.firstinspires.ftc.teamcode.util.nebulaHardware.NebulaMotor;
import org.firstinspires.ftc.teamcode.util.nebulaHardware.NebulaMotorGroup;

@Config
public class Slide extends SubsystemBase {
    public final NebulaMotorGroup motorGroup;
    protected Telemetry telemetry;
    protected NebulaMotor slideM1, slideM2;
    protected PIDFController slideController;
    protected boolean slideAutomatic;
    protected double output = 0;
    protected boolean dropBoolean = false;

    //TODO: Should the Slide even drop?
    public static SlideValue REST = make(SlideEnum.TRANSFER, true);
    public static SlideValue LOW = make(SlideEnum.LOW, true);
    public static SlideValue MID = make(SlideEnum.MID,5,false);
    public static SlideValue HIGH = make(SlideEnum.HIGH,5,false);



    protected static SlideEnum slidePos;

    public Slide(Telemetry tl, HardwareMap hw, boolean isEnabled) {
        slideM1 = new NebulaMotor(hw,
            NebulaConstants.Slide.slideRName,
            NebulaConstants.Slide.slideType,
            NebulaConstants.Slide.slideRDirection,
            NebulaConstants.Slide.slideIdleMode,
            isEnabled);
        slideM2 = new NebulaMotor(hw,
            NebulaConstants.Slide.slideLName,
            NebulaConstants.Slide.slideType,
            NebulaConstants.Slide.slideLDirection,
            NebulaConstants.Slide.slideIdleMode,
            isEnabled);

        motorGroup = new NebulaMotorGroup(slideM1, slideM2);
        slideM1.setDistancePerPulse(NebulaConstants.Slide.slideDistancePerPulse);
        slideM2.setDistancePerPulse(NebulaConstants.Slide.slideDistancePerPulse);

        slideController = new PIDFController(NebulaConstants.Slide.slidePID.p,
            NebulaConstants.Slide.slidePID.i,
            NebulaConstants.Slide.slidePID.d,
            NebulaConstants.Slide.slidePID.f,
            getEncoderDistance(),
            getEncoderDistance());
        slideController.setTolerance(NebulaConstants.Slide.slideTolerance);

        this.telemetry = tl;
        slideAutomatic = false;
        slidePos = SlideEnum.TRANSFER;
    }

    @Override
    public void periodic() {
        if (slideAutomatic) {
//            slideController.setF(NebulaConstants.Slide.slidePID.f * Math.cos(Math.toRadians(slideController.getSetPoint())));

            output = slideController.calculate(getEncoderDistance());
//            if (output >= 1) output = 1;
//            if (output <= -1) output = -1;

            setPower(output);//TODO: Probably shouldn't be like this
//            if (lowBool) {
//                slideM1.set(output * LOW_POWER);
//                slideM2.set(output * LOW_POWER);
//            }
//            else {
//                slideM1.set(output * POWER);
//                slideM2.set(output * POWER);
//            }
        }
        telemetry.addData("Slide Motor Output:", output);
        telemetry.addData("Slide1 Encoder: ", slideM1.getPosition());
        telemetry.addData("Slide2 Encoder: ", slideM2.getPosition());
        telemetry.addData("Slide Pos:", getSetPoint());
    }

    public double getEncoderDistance() {
        return slideM1.getDistance();
    }


    public void setPower(double power) {
        slideM1.setPower(power);
        slideM2.setPower(-power);//Instead of putting -power, maybe reverse the motor
    }

    public void stopSlide() {
        slideM1.stop();
        slideM2.stop();
        slideController.setSetPoint(getEncoderDistance());

        slideAutomatic = false;
    }
    /****************************************************************************************/

    public void slideResting() {
        slideAutomatic = true;
        slideController.setSetPoint(REST.slidePosition);
        slidePos = SlideEnum.TRANSFER;
    }

    public void resetEncoder() {
        slideM1.resetEncoder();
        slideM2.resetEncoder();
    }

//    public boolean isSlideAutomatic(){
//        return slideAutomatic;
//    }

//    public void dropSlide(){
//        switch (slidePos){
////            case LOW:
////                upController.setSetPoint(LOW_POS+350);
////                break;
////            case MID:
////                upController.setSetPoint(MID_POS+200);
////                break;
//            case HIGH:
//                slideController.setSetPoint(HIGH.slidePosition-740);
//                break;
////            case AUTO_MID:
////                slideController.setSetPoint(AUTO_MID.slidePosition-290);
////                break;
////            case AUTO_HIGH:
////                slideController.setSetPoint(AUTO_HIGH.slidePosition-650);
////                break;
//        }
//    }

    public void setSetPoint(double setPoint, boolean lowBool) {
        if(NebulaConstants.GamePad.overrideSafety){
            if(setPoint>NebulaConstants.Slide.MAX_POSITION ||
                setPoint<NebulaConstants.Slide.MIN_POSITION){
                slideM1.stop();
                return;
            }
        }

        slideController.setSetPoint(setPoint);
        this.dropBoolean = lowBool;
    }

    //TODO: Test!
    public Command setSetPointCommand(double setPoint, boolean shouldSensorWork) {
        slidePos = SlideEnum.MANUAL;
        return new InstantCommand(()->{this.setSetPoint(setPoint, shouldSensorWork);});
    }
    public Command setSetPointCommand(SlideValue pos) {
        slidePos = pos.slideEnum;
        return new InstantCommand(()->{
            this.setSetPoint(pos.slidePosition, pos.shouldSlideDrop);});
    }

    public double getSetPoint() {
        return slideController.getSetPoint();
    }
}
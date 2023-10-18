package org.firstinspires.ftc.teamcode.subsystems.climber;

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
public class Climber extends SubsystemBase {
    protected Telemetry telemetry;
    protected MotorEx climber;
    protected PIDFController climberController;
    protected double output = 0;

    public enum ClimbEnum {
        CLIMB(0.0),
        REST(0.0),
        MANUAL(0.0);
        public final double climbPos;
        ClimbEnum(double climbPos) {
            this.climbPos = climbPos;
        }
    }


    protected static ClimbEnum climbPos;

    public Climber(Telemetry tl, HardwareMap hw) {
        climber = new MotorEx(hw,"climb");

        climber.setDistancePerPulse(1);

        climberController = new PIDFController(0,0,0,0,
            getEncoderDistance(),
            getEncoderDistance());
        climberController.setTolerance(1);

        this.telemetry = tl;
        climbPos = ClimbEnum.REST;
    }

    @Override
    public void periodic() {
//        slideController.setF(NebulaConstants.Slide.slidePID.f * Math.cos(Math.toRadians(slideController.getSetPoint())));
        output = climberController.calculate(getEncoderDistance());
        setPower(output);//TODO: Probably shouldn't be like this

        telemetry.addData("Slide Motor Output:", output);
        telemetry.addData("Climber Encoder: ", climber.getCurrentPosition());
        telemetry.addData("Slide Pos:", getSetPoint());
    }

    public double getEncoderDistance() {
//        return slideM1.getDistance();
        return climber.getCurrentPosition();
        //TODO:Does this work?
    }


    public void setPower(double power) {
        climber.set(power);
    }

    public void stopSlide() {
        climber.stopMotor();
        climberController.setSetPoint(getEncoderDistance());
    }
    /****************************************************************************************/


    public void resetEncoder() {
        climber.resetEncoder();
    }


    public void setSetPoint(double setPoint) {
        //TODO: Maybe should remove all Safety Stuff
//        if(NebulaConstants.GamePad.overrideSafety){
//            if(setPoint>NebulaConstants.Climber.MAX_POSITION ||
//                setPoint<NebulaConstants.Climber.MIN_POSITION){
////                slideM1.stop();
//                return;
//            }
//        }

        climberController.setSetPoint(setPoint);
    }

    //TODO: Test!
    public Command setSetPointCommand(double setPoint) {
        climbPos = ClimbEnum.MANUAL;    //WTH is this; The booleans don't match
        return new InstantCommand(()->{this.setSetPoint(setPoint);});
    }
    public Command setSetPointCommand(ClimbEnum pos) {
        return new InstantCommand(()->{setSetPoint(pos.climbPos);});
    }

    public double getSetPoint() {
        return climberController.getSetPoint();
    }
}
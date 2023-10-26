package org.firstinspires.ftc.teamcode.subsystems.arm;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class MotorArm extends SubsystemBase {
//    public final NebulaMotorGroup motorGroup;
    protected Telemetry telemetry;
    protected MotorEx arm;
    protected PIDFController armController;
    protected double output = 0;
//    protected boolean dropBoolean = false;

    //TODO: Should the Slide even drop?

    public enum ArmEnum {
        TRANSFER(0.0),

        LOW(-150),
        MID(-300),
        HIGH(-400),

        MANUAL(0.0);
        public final double armPos;
        ArmEnum(double armPos) {
            this.armPos = armPos;
        }
    }


    protected static ArmEnum slidePos;

    public MotorArm(Telemetry tl, HardwareMap hw) {
        arm = new MotorEx(hw, "arm");

//        motorGroup = new NebulaMotorGroup(slideR, slideL);
        arm.setDistancePerPulse(1);

        armController = new PIDFController(0,0,0,0,
            getEncoderDistance(),
            getEncoderDistance());
        armController.setTolerance(1);

        this.telemetry = tl;
        slidePos = ArmEnum.TRANSFER;
    }

    @Override
    public void periodic() {
        armController.setF(armController.getF() * Math.cos(Math.toRadians(armController.getSetPoint())));
        output = armController.calculate(getEncoderDistance());
        setPower(output);//TODO: Probably shouldn't be like this

        telemetry.addData("Arm Motor Output:", output);
        telemetry.addData("Slide1 Encoder: ", arm.getCurrentPosition());
        telemetry.addData("Arm Pos:", getSetPoint());
    }

    public double getEncoderDistance() {
        return arm.getDistance();
//        return slideR.getPosition();
        //TODO:Does this work?
    }


    public void setPower(double power) {
        arm.set(power);
//        slideM1.setPower(power);
//        slideM2.setPower(power);//Instead of putting -power, maybe reverse the motor
    }

    public void stopSlide() {
        arm.stopMotor();
        armController.setSetPoint(getEncoderDistance());
    }
    /****************************************************************************************/


    public void resetEncoder() {
        arm.resetEncoder();
    }


    private void setSetPoint(double setPoint) {
        armController.setSetPoint(setPoint);
    }

    //TODO: Test!
    public Command setSetPointCommand(double setPoint) {
        slidePos = ArmEnum.MANUAL;    //WTH is this; The booleans don't match
        return new InstantCommand(()->{this.setSetPoint(setPoint);});
    }
    public Command setSetPointCommand(ArmEnum pos) {
        return new InstantCommand(()->{setSetPoint(pos.armPos);});
    }

    public double getSetPoint() {
        return armController.getSetPoint();
    }
}
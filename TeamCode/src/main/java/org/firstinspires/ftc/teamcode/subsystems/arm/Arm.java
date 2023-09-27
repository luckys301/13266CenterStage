package org.firstinspires.ftc.teamcode.subsystems.arm;

import static org.firstinspires.ftc.teamcode.subsystems.arm.ArmValue.PivotEnum;
import static org.firstinspires.ftc.teamcode.subsystems.arm.ArmValue.make;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.NebulaConstants;
import org.firstinspires.ftc.teamcode.util.misc.Util;
import org.firstinspires.ftc.teamcode.util.nebulaHardware.NebulaMotor;

import java.util.logging.Level;


@Config
public class Arm extends SubsystemBase {
    protected static Double A = PivotEnum.RESET.value;

    public static ArmValue INTAKE_FRONT = make(PivotEnum.INTAKE_FRONT,1, true);
    public static ArmValue INTAKE_BACK = make(PivotEnum.INTAKE_BACK,5,false);
    public static ArmValue AUTO_INTAKE_BACK = make(PivotEnum.AUTO_INTAKE_BACK,1, true);
    public static ArmValue AUTO_INTAKE_FRONT= make(PivotEnum.AUTO_INTAKE_FRONT,5,false);
    public static ArmValue DROP_FRONT = make(PivotEnum.DROP_FRONT,1, true);
    public static ArmValue DROP_BACK = make(PivotEnum.DROP_FRONT,1, true);
    public static ArmValue AUTO_DROP_BACK = make(PivotEnum.AUTO_DROP_BACK,5,false);
    public static ArmValue RESET = make(PivotEnum.AUTO_DROP_BACK,5,false);

    protected final PIDFController controller;
    protected boolean armAutomatic;
    protected boolean shouldSensorWork = true;

    protected static PivotEnum pivotPos = PivotEnum.RESET; // HOlds Current Pivot Position
//    protected final static double POWER = 0.93;
    protected double encoderOffset = 0;
    protected Telemetry telemetry;
    protected final NebulaMotor armMotor;

    public Arm(Telemetry tl, HardwareMap hw, boolean isEnabled) {
        armMotor = new NebulaMotor(hw, NebulaConstants.Arm.armMName,
            NebulaConstants.Arm.armType,
            NebulaConstants.Arm.armDirection,
            NebulaConstants.Arm.armIdleMode,
            isEnabled);
        armMotor.setDistancePerPulse(1);
        controller = new PIDFController(NebulaConstants.Arm.armPID.p,
            NebulaConstants.Arm.armPID.i,
            NebulaConstants.Arm.armPID.d,
            NebulaConstants.Arm.armPID.f,
            getEncoderDistance(),
            getEncoderDistance());
        controller.setTolerance(NebulaConstants.Arm.armTolerance);
//        this.telemetry = tl;
        //TODO: Does removing this make it still work?^^^
        armAutomatic = false;
    }

    @Override
    public void periodic() {
        if (armAutomatic) {
//            controller.setF(NebulaConstants.Pivot.pivotPID.f * Math.cos(Math.toRadians(controller.getSetPoint())));
            double output = (controller.calculate(getEncoderDistance()));
            telemetry.addData("CLaw Motor Output:", output);

            armMotor.setPower(output);
        }
        Util.logger(this, telemetry, Level.INFO, "Arm Encoder Pos: ", armMotor.getPosition());
        Util.logger(this, telemetry, Level.INFO, "Arm Set Point: ", getSetPoint());

    }


    public double getEncoderDistance() {
        return armMotor.getDistance() - encoderOffset;
    }
    public double getAngle() {
        return getEncoderDistance();
    }

//    public void stopArm() {
//        armMotor.stopMotor();
//        armAutomatic = false;
//    }

    public void moveInitializationPosition() {
        armAutomatic = true;
        setSetPoint(RESET.pivotPosition - encoderOffset - 10, true);
        pivotPos = PivotEnum.RESET;
    }
    public void moveIntakeBAuto() {
        armAutomatic = true;
        setSetPointCommand(this.AUTO_INTAKE_BACK);
    }
    public void dropArmTeleop(){
        switch (pivotPos){
            case FRONT:
            case HIGH_FRONT:
                setSetPointCommand(DROP_FRONT);
                break;
            case BACK:
            case HIGH_BACK:
                setSetPointCommand(DROP_BACK);
                break;
        }
    }

    public void dropArmAuto(){
        switch (pivotPos){
            case AUTO_HIGH_BACK:
            case AUTO_BACK:
                setSetPoint(AUTO_DROP_BACK.pivotPosition, false);
                break;
            case AUTO_HIGH_FRONT:
            case AUTO_FRONT:
                setSetPoint(AUTO_INTAKE_FRONT.pivotPosition,false);
                break;
            case AUTO_INTAKE_FRONT:
                setSetPoint(AUTO_INTAKE_FRONT.pivotPosition+25, true);
                break;
            case AUTO_INTAKE_BACK:
                setSetPoint(AUTO_INTAKE_BACK.pivotPosition-25, true);
                break;
        }
    }

    public void setSetPoint(double setPoint, boolean shouldSensorWork) {
        if(NebulaConstants.Gamepad.overrideSafety){
            if(setPoint> NebulaConstants.Arm.MAX_POSITION ||
                setPoint< NebulaConstants.Arm.MIN_POSITION){
                armMotor.stop();
                return;
            }
        }

        controller.setSetPoint(setPoint + encoderOffset);
        this.shouldSensorWork = shouldSensorWork;
    }

    //TODO: Test!
    public Command setSetPointCommand(double setPoint, boolean shouldSensorWork) {
        pivotPos = PivotEnum.MANUAL;
        return new InstantCommand(()->{setSetPoint(setPoint, shouldSensorWork);});
    }
    public Command setSetPointCommand(ArmValue pos) {
        pivotPos = pos.pivotEnum;
        return new InstantCommand(()->{setSetPoint(pos.pivotPosition, pos.shouldSensorWork);});
    }

    public void encoderReset() {
        armMotor.resetEncoder();
        telemetry.addLine("ARM RESET");
    }
    public double getSetPoint(){
        return controller.getSetPoint();
    }
}
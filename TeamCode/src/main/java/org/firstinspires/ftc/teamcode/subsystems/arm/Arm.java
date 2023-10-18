package org.firstinspires.ftc.teamcode.subsystems.arm;


import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Arm extends SubsystemBase
{
    public enum ArmPos {
        TRANSFER(0,0),
        OUTTAKE(0,0);

        public final double armRPos, armLPos;
        ArmPos(double armRPos, double armLPos) {
            this.armRPos = armRPos;
            this.armLPos = armLPos;
        }
    }
    Telemetry telemetry;
    private final ServoEx armR, armL;

    public Arm(Telemetry tl, HardwareMap hw) {
        armR = new SimpleServo(hw, "armR", 0, 360);
        armL = new SimpleServo(hw, "armL", 0, 36);
        armSetPosition(ArmPos.TRANSFER);

        this.telemetry = tl;
    }

    @Override
    public void periodic() {
        telemetry.addData("ArmR Pos: ", armR.getPosition());
        telemetry.addData("ArmL Pos: ", armL.getPosition());
    }

    private void armSetPosition(ArmPos armPos) {
        armR.setPosition(armPos.armRPos);
        armL.setPosition(armPos.armLPos);
    }
    public Command armSetPositionCommand(ArmPos armPos) {
        return new InstantCommand(()->{armSetPosition(armPos);});
    }
}
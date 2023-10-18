package org.firstinspires.ftc.teamcode.subsystems.intake;


import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

//@Deprecated
@Config
public class PowerIntake extends SubsystemBase {
    public enum IntakePower {
        OUTTAKE(0.5),
        INTAKE(-0.5),
        STOP(0),
        OUTTAKE_YELLOW(0.1);

        public final double power;
        IntakePower(double power) {
            this.power = power;
        }
    }
    
    public enum IntakePos {
        UP(0),
        DOWN(0);
        
        public final double rPos;
        IntakePos(double rPos){
            this.rPos = rPos;
        }
    }
    
    IntakePower shooterRPM = IntakePower.STOP;
    Telemetry telemetry;
    
    private final MotorEx intakeMotor;
    private final ServoEx intakeServoR;
    

    public PowerIntake(Telemetry tl, HardwareMap hw) {
        intakeMotor = new MotorEx(hw, "intake");
        intakeServoR = new SimpleServo(hw, "intakeServo", 0,360);
        this.telemetry = tl;
    }

    @Override
    public void periodic() {
        telemetry.addData("Intake Speed:", intakeMotor.getVelocity());
    }

    private void setPower(double power) {
        intakeMotor.set(power);
    }

    //TODO: Test!
    public Command setPowerCommand(double power) {
        return new InstantCommand(()->{
            setPower(power);
        });
    }
    public Command setPowerCommand(IntakePower pos) {
        return setPowerCommand(pos.power);
//        return new InstantCommand(()->{setSetPoint(pos);});
    }

    public void encoderReset() {//Motors wouldn't need reset
        intakeMotor.resetEncoder();
    }
}

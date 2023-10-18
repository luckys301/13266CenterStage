package org.firstinspires.ftc.teamcode.commands.drive.teleop;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;


public class DefaultArmCommand extends CommandBase {
    private GamepadEx operatorGamepad;

    protected double multiplier;
    boolean mecDrive = true;
    int choice = 0;

    public DefaultArmCommand( GamepadEx operatorGamepad) {
        this.operatorGamepad = operatorGamepad;
    }

    @Override
    public void execute() {
//        if(operatorGamepad.getRightY()>0.5) {
//            pivot.raiseClawManual();
//        } else if(operatorGamepad.getRightY()<-0.5){
//            pivot.lowerClawManual();
//        }
    }


    @Override
    public void end(boolean interrupted) {
//        arm.stopArm();
    }
}

package org.firstinspires.ftc.teamcode.commands.outtake;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.arm.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;

public class DropCommand extends SequentialCommandGroup  {

    public DropCommand(Arm arm, Claw claw){
        addCommands(
                arm.armSetPositionCommand(Arm.ArmPos.OUTTAKE),
                claw.setClawPos(Claw.ClawPos.OPEN_POS)
        );
    }

}

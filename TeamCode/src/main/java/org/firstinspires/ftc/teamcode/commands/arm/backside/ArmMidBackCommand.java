package org.firstinspires.ftc.teamcode.commands.arm.backside;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.old.Pivot;
import org.firstinspires.ftc.teamcode.subsystems.old.Claw;
import org.firstinspires.ftc.teamcode.subsystems.old.Slide;
import org.firstinspires.ftc.teamcode.subsystems.old.TurnServo;

public class ArmMidBackCommand extends SequentialCommandGroup {
    public ArmMidBackCommand(Slide slide, Pivot pivot, Claw claw, TurnServo turnServo, boolean auto) {
        if (auto){
            addCommands(

                    new InstantCommand(claw::clawClose),
                    new ParallelCommandGroup(
                            new InstantCommand(slide::slideMid),
                            new InstantCommand(pivot::moveBAuto)
                    ),
                    new WaitCommand(200),
                    new InstantCommand(turnServo::setBClawPos)
            );
        }
        else {
            addCommands(
                    new ParallelCommandGroup(
                            new InstantCommand(() ->
                                    new Thread(() -> {
                                        claw.clawClose();
                                        slide.slideMid();
                                        pivot.moveB();
                                    }).start())
                    ),
                    new WaitCommand(800),
                    new InstantCommand(turnServo::setBClawPos)
            );
        }
    }
}
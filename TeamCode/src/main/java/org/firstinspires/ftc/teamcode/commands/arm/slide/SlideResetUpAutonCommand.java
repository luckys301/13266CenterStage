package org.firstinspires.ftc.teamcode.commands.arm.slide;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.old.Pivot;
import org.firstinspires.ftc.teamcode.subsystems.old.Claw;
import org.firstinspires.ftc.teamcode.subsystems.old.Slide;
import org.firstinspires.ftc.teamcode.subsystems.old.TurnServo;

public class SlideResetUpAutonCommand extends SequentialCommandGroup{
    public SlideResetUpAutonCommand(Slide slide, Pivot pivot, Claw claw, TurnServo turnServo){
        addCommands(
                new WaitCommand(67),
                new InstantCommand(claw::clawAutoClose),
                new InstantCommand(turnServo::setFClawPos),
                new InstantCommand(
                        () -> new Thread(() -> {
//                            pivot.moveInitializationPosition();
                            pivot.moveIntakeBAuto();
                            slide.slideResting();
                            turnServo.setFClawPos();
                        }).start()
                ),
                new WaitCommand(200),
                new InstantCommand(pivot::stopArm)
        );
    }
}

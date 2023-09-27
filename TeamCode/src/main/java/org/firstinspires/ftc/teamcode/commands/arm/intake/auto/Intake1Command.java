package org.firstinspires.ftc.teamcode.commands.arm.intake.auto;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.old.Pivot;
import org.firstinspires.ftc.teamcode.subsystems.old.Claw;
import org.firstinspires.ftc.teamcode.subsystems.old.Slide;
import org.firstinspires.ftc.teamcode.subsystems.old.TurnServo;

public class Intake1Command extends SequentialCommandGroup   {
    public Intake1Command(Slide slide, Claw claw, Pivot pivot, TurnServo turnServo){
        addCommands(
                new WaitCommand(420),
                new ParallelCommandGroup(
                        new InstantCommand(claw::clawClose),
//                        new InstantCommand(claw::clawAutoClose),
                        new InstantCommand(pivot::moveIntakeBAuto)

                ),
                new WaitCommand(150),
                new ParallelCommandGroup(
                        new InstantCommand(slide::slideCone1),
                        new InstantCommand(turnServo::setBClawPos)
                ),
                new WaitCommand(210),
                new InstantCommand(claw::clawOpen)
        );
    }
}
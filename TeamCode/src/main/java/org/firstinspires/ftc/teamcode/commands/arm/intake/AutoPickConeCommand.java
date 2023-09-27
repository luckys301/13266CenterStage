package org.firstinspires.ftc.teamcode.commands.arm.intake;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.old.Claw;
import org.firstinspires.ftc.teamcode.subsystems.old.Slide;

public class AutoPickConeCommand extends SequentialCommandGroup  {
    public AutoPickConeCommand(Slide slide, Claw claw){
        addCommands(
                new InstantCommand(claw::clawClose),
//                new InstantCommand(claw::clawAutoClose),
                new WaitCommand(300),
                new InstantCommand(slide::slideLow)
        );
    }
}
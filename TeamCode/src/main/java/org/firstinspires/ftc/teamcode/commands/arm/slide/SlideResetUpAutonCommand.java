package org.firstinspires.ftc.teamcode.commands.arm.slide;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.slide.Slide;


public class SlideResetUpAutonCommand extends SequentialCommandGroup{
    public SlideResetUpAutonCommand(Slide slide, Claw claw){
        addCommands(
        );
    }
}

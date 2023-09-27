package org.firstinspires.ftc.teamcode.commands.example;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;


//nowadays lance isnt a very common name, but in older times people were named lance a lot
public class ThreadCommand extends SequentialCommandGroup{
    public ThreadCommand(){
        addRequirements();    //Add Subsystems that you need to run this Command - not necessary
        addCommands(
            new InstantCommand( //Better Idea to use Parallel Command
                () -> new Thread(() -> {
//                    pivot.moveIntakeBAuto();
//                    slide.slideResting();
                }).start()
            )
        );
    }
}
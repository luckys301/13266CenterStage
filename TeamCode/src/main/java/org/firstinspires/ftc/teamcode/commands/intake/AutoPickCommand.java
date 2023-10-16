package org.firstinspires.ftc.teamcode.commands.intake;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.intake.PowerIntake;

public class AutoPickCommand extends SequentialCommandGroup  {
    public AutoPickCommand(PowerIntake intake){
        addCommands(
            intake.setSetPointCommand(PowerIntake.IntakePower.INTAKE)
        );
    }
}
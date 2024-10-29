package frc.robot.Auton.Commands;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Auton.Commands.Drive.driveToTime;
import frc.robot.Auton.Commands.Shooter.shootToTime;

public class AutonList{
    public static final Command testAuton = new SequentialCommandGroup(
        new driveToTime(0.2, 0.2, 1),
        new shootToTime(0.27, false, 1),
        new shootToTime(0.27, true, 1)
    );

    public static Optional<Command> getDefaultCommand(){
        return Optional.ofNullable(testAuton);
    }
}   

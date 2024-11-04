package frc.robot.Auton.Commands;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Auton.Commands.Drive.driveToDistance;
import frc.robot.Auton.Commands.Drive.driveToRotation;
import frc.robot.Auton.Commands.Drive.driveToTime;
import frc.robot.Auton.Commands.Intake.intakeBall;
import frc.robot.Auton.Commands.Intake.setIntakeExt;
import frc.robot.Auton.Commands.Intake.setIntakeState;
import frc.robot.Auton.Commands.Shooter.shootToTime;

public class AutonList{
    public static final Command leftSideAuton = new SequentialCommandGroup(
        new driveToTime(1, 1, 1.2),
        new shootToTime(0.27, false, 1),
        new shootToTime(0.27, true, 1),
        new setIntakeExt(true),
        new driveToRotation(-38, 3),
        new ParallelRaceGroup(
            new intakeBall(),
            new driveToTime(1, 1, 1),
            new WaitCommand(3)
        ),
        new WaitCommand(0.2),
        new driveToTime(-1, -1, 1),
        new driveToRotation(0, 3),
        new ParallelRaceGroup(
            new setIntakeState(-Constants.handoffPower, -Constants.intakePower),
            new WaitCommand(0.15)
        ),
        new shootToTime(.27, false, 1),
        new shootToTime(.27, true, 3)
    );

    public static final Command rightSideAuton = new SequentialCommandGroup(
        new driveToTime(1, 1, 1.2),
        new shootToTime(0.27, false, 1),
        new shootToTime(0.27, true, 1),
        new setIntakeExt(true),
        new driveToRotation(38, 2),
        new ParallelRaceGroup(
            new intakeBall(),
            new driveToTime(1, 1, 1),
            new WaitCommand(3)
        ),
        new WaitCommand(0.2),
        new driveToTime(-1, -1, 1),
        new driveToRotation(0, 3),
        new ParallelRaceGroup(
            new setIntakeState(-Constants.handoffPower, -Constants.intakePower),
            new WaitCommand(0.15)
        ),
        new shootToTime(.27, false, 1),
        new shootToTime(.27, true, 3)
    );

    public static Optional<Command> getDefaultCommand(){
        return Optional.ofNullable(rightSideAuton);
    }
}   

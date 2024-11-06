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
        new ParallelRaceGroup(
            new driveToRotation(-38, 3),
            new WaitCommand(1)
        ),
        new ParallelRaceGroup(
            new intakeBall(),
            new driveToTime(1, 1, 1),
            new WaitCommand(3)
        ),
        new WaitCommand(0.2),
        new driveToTime(-1, -1, 1),
        
        new ParallelRaceGroup(
            new driveToRotation(0, 3),
            new WaitCommand(1)
        ),
        new ParallelRaceGroup(
            new setIntakeState(-Constants.handoffPower, 0),
            new WaitCommand(0.15)
        ),
        new ParallelCommandGroup(
            new shootToTime(.27, false, 1),
            new setIntakeExt(false)
        ),
        new shootToTime(.27, true, 1.5),
        new ParallelRaceGroup(
            new driveToRotation(-55, 2),
            new WaitCommand(1)
        ),
        new driveToTime(1, 1, 1.5),
        new ParallelCommandGroup(
            new ParallelRaceGroup(
            new driveToRotation(0, 2),
            new WaitCommand(1)
            ),
            new setIntakeExt(true)
        ),
        new ParallelRaceGroup(
            new intakeBall(),
            new driveToTime(1, 1, 1.6),
            new WaitCommand(3)
        ),
        new ParallelCommandGroup(
            new driveToTime(-1, -1, 1.6),
            new setIntakeExt(false),
            new ParallelRaceGroup(
                new setIntakeState(-Constants.handoffPower, 0),
                new WaitCommand(0.15)
            )
        ),
        new ParallelRaceGroup(
            new driveToRotation(-55, 3),
            new WaitCommand(1)
        ),
        new driveToTime(-1, -1, 1.5),
        new ParallelRaceGroup(
            new driveToRotation(0, 2),
            new shootToTime(.27, true, 1.5)
        ),
        new shootToTime(.27, true, 1.5)

    );

    public static final Command rightSideAuton = new SequentialCommandGroup(
        new driveToTime(1, 1, 1.2),
        new shootToTime(0.27, false, 1),
        new shootToTime(0.27, true, 1),
        new setIntakeExt(true),
        new ParallelRaceGroup(
            new driveToRotation(38, 3),
            new WaitCommand(1)
        ),
        new ParallelRaceGroup(
            new intakeBall(),
            new driveToTime(1, 1, 1),
            new WaitCommand(3)
        ),
        new WaitCommand(0.2),
        new driveToTime(-1, -1, 1),
        
        new ParallelRaceGroup(
            new driveToRotation(0, 3),
            new WaitCommand(1)
        ),
        new ParallelRaceGroup(
            new setIntakeState(-Constants.handoffPower, 0),
            new WaitCommand(0.15)
        ),
        new ParallelCommandGroup(
            new shootToTime(.27, false, 1),
            new setIntakeExt(false)
        ),
        new shootToTime(.27, true, 1.5),
        new ParallelRaceGroup(
            new driveToRotation(55, 2),
            new WaitCommand(1)
        ),
        new driveToTime(1, 1, 1.5),
        new ParallelCommandGroup(
            new ParallelRaceGroup(
            new driveToRotation(0, 2),
            new WaitCommand(1)
            ),
            new setIntakeExt(true)
        ),
        new ParallelRaceGroup(
            new intakeBall(),
            new driveToTime(1, 1, 1.6),
            new WaitCommand(3)
        ),
        new ParallelCommandGroup(
            new driveToTime(-1, -1, 1.6),
            new setIntakeExt(false),
            new ParallelRaceGroup(
                new setIntakeState(-Constants.handoffPower, 0),
                new WaitCommand(0.15)
            )
        ),
        new ParallelRaceGroup(
            new driveToRotation(55, 3),
            new WaitCommand(1)
        ),
        new driveToTime(-1, -1, 1.5),
        new ParallelRaceGroup(
            new driveToRotation(0, 2),
            new shootToTime(.27, true, 1.5)
        ),
        new shootToTime(.27, true, 1.5)
    );

    public static Optional<Command> getDefaultCommand(){
        return Optional.ofNullable(leftSideAuton);
    }
}   

package frc.robot.Auton.Commands;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
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
import frc.robot.Auton.Commands.Shooter.chargeShoot;
import frc.robot.Auton.Commands.Shooter.shootToTime;
import frc.robot.Auton.Commands.Shooter.shootToVelocity;

public class AutonList{
    public static final Command leftSideAuton = new SequentialCommandGroup(
        new driveToTime(1, 1, 1.2),
        new shootToTime(Constants.shooterRPM, false, 1),
        new shootToTime(Constants.shooterRPM, true, 1),
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
        new shootToTime(Constants.shooterRPM, false, 1),
        new shootToTime(Constants.shooterRPM, true, 1),
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

    public static final Command left3Ball = new SequentialCommandGroup(
        new ParallelRaceGroup(
            new driveToDistance(1, 1, 0.2),
            new chargeShoot(Constants.shooterRPM)
        ),
        new shootToVelocity(Constants.shooterRPM, true, 7, 1),
        
        new ParallelCommandGroup(
            new ParallelRaceGroup(
                new driveToRotation(-27, 1),
                new WaitCommand(1.5)
            )
            ,
            new setIntakeExt(true)
        ),
        new ParallelCommandGroup(
            new ParallelRaceGroup(
                new intakeBall(),
                new WaitCommand(2)
            ),
            new driveToDistance(1.5, 1.5, 0.2)
            
        ),
        new ParallelRaceGroup(
            new chargeShoot(2100),
            new driveToRotation(-15, 0.5),
            new WaitCommand(1.5)
        ),
        new shootToVelocity(2100, true, 30, 1.2),
        new ParallelCommandGroup(
            new ParallelRaceGroup(
                new driveToRotation(-60, 0.5),
                new WaitCommand(1.5)
            ),
            new setIntakeExt(false)
        ),
        new driveToDistance(1.15, 1.15, 0.05),
        new ParallelCommandGroup(
            new ParallelRaceGroup(
                new driveToRotation(0, 0.3),
                new WaitCommand(1)
            ),
            new setIntakeExt(true)
        ),
        new ParallelCommandGroup(
            new ParallelRaceGroup(
                new intakeBall(),
                new WaitCommand(2)
            ),
            new driveToDistance(1.2, 1.2, 0.1)
        ),
        new ParallelRaceGroup(
            new intakeBall(),
            new WaitCommand(1)
        )
        ,
        new ParallelRaceGroup(
            new driveToDistance(-1.2, -1.2, 0.1),
            new WaitCommand(3)
        ),
        new ParallelCommandGroup(
            new ParallelRaceGroup(
                new driveToRotation(-60, 0.5),
                new WaitCommand(1.5)
            ),
            new setIntakeExt(false)
        ),
        new ParallelRaceGroup(
            new driveToDistance(-1.15, -1.15, 0.05),
            new WaitCommand(3)
        ),
        new ParallelRaceGroup(
            new chargeShoot(2100),
            new driveToRotation(-15, 0),
            new WaitCommand(1)
        ),
        new shootToVelocity(2100, true, 30, 1)
        

    );

    public static final Command right3Ball = new SequentialCommandGroup(
        new ParallelRaceGroup(
            new driveToDistance(1, 1, 0.2),
            new chargeShoot(Constants.shooterRPM)
        ),
        new ParallelRaceGroup(
            new driveToRotation(10, 2),
            new WaitCommand(0.5)
        ),
        new shootToVelocity(Constants.shooterRPM, true, 7, 1),
        
        new ParallelCommandGroup(
            new ParallelRaceGroup(
                new driveToRotation(27, 1),
                new WaitCommand(1.5)
            )
            ,
            new setIntakeExt(true)
        ),
        new ParallelCommandGroup(
            new ParallelRaceGroup(
                new intakeBall(),
                new WaitCommand(2)
            ),
            new driveToDistance(1.5, 1.5, 0.2)
            
        ),
        new ParallelRaceGroup(
            new chargeShoot(2100),
            new driveToRotation(15, 0.5),
            new WaitCommand(1.5)
        ),
        new shootToVelocity(2100, true, 20, 1.2),
        new ParallelCommandGroup(
            new ParallelRaceGroup(
                new driveToRotation(60, 0.5),
                new WaitCommand(1.5)
            ),
            new setIntakeExt(false)
        ),
        new driveToDistance(1, 1, 0.1),
        new ParallelCommandGroup(
            new ParallelRaceGroup(
                new driveToRotation(0, 0.5),
                new WaitCommand(1)
            ),
            new setIntakeExt(true)
        ),
        new ParallelCommandGroup(
            new ParallelRaceGroup(
                new intakeBall(),
                new WaitCommand(2)
            ),
            new driveToDistance(1.2, 1.2, 0.1)
        ),
        new ParallelRaceGroup(
            new intakeBall(),
            new WaitCommand(1)
        )
        ,
        new ParallelRaceGroup(
            new driveToDistance(-1.2, -1.2, 0.1),
            new WaitCommand(3)
        ),
        new ParallelCommandGroup(
            new ParallelRaceGroup(
                new driveToRotation(60, 0.5),
                new WaitCommand(1.5)
            ),
            new setIntakeExt(false)
        ),
        new ParallelRaceGroup(
            new driveToDistance(-1.1, -1.1, 0.1),
            new WaitCommand(3)
        ),
        new ParallelRaceGroup(
            new chargeShoot(2100),
            new driveToRotation(15, 0),
            new WaitCommand(1)
        ),
        new shootToVelocity(2100, true, 30, 1)
        

    );

    public static final Command middleAuton = new SequentialCommandGroup(
        new ParallelRaceGroup(
            new driveToDistance(1, 1, 0.2),
            new chargeShoot(Constants.shooterRPM)
        ),
        new shootToVelocity(Constants.shooterRPM, true, 7, 1),
        new ParallelRaceGroup(
            new driveToRotation(13, 2),
            new WaitCommand(0.5)
        ),
        new ParallelCommandGroup(
            new ParallelRaceGroup(
                new intakeBall(),
                new WaitCommand(4)
            ),
            new driveToDistance(2.9, 2.9, 0.2),
            new setIntakeExt(true)
        ),
        new ParallelRaceGroup(
            new driveToRotation(0, 2),
            new WaitCommand(0.5)
        ),
        new ParallelRaceGroup(
            new driveToDistance(-1, -1, 0.2),
            new chargeShoot(2100)
        ),
        new shootToVelocity(2100, true, 7, 1)
    );

    public static Optional<Command> getDefaultCommand(){
        return Optional.ofNullable(left3Ball);
    }
}   

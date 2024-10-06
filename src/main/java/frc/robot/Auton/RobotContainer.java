package frc.robot.Auton;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Auton.Commands.intakeBall;
import frc.robot.Auton.Commands.shootBall;
import frc.robot.subsystems.Drive;

public class RobotContainer {
    public Command getAutonomousCommand(){
        NamedCommands.registerCommand("intakeBall", new intakeBall());
        NamedCommands.registerCommand("shootBall", new shootBall());

        return Drive.getInstance().autoBuilder.buildAuto("BL Auto");
    }
}

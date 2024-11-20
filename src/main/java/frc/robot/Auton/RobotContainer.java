package frc.robot.Auton;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Auton.Commands.Intake.intakeBall;
import frc.robot.Auton.Commands.Shooter.chargeShoot;
import frc.robot.subsystems.Drive;

public class RobotContainer {
    public Command getAutonomousCommand(){
        NamedCommands.registerCommand("intakeBall", new intakeBall());

        return Drive.getInstance().autoBuilder.buildAuto("BL Auto");
    }
}

package frc.robot.Auton.Commands.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class shootToVelocity extends Command {
    double shootSpeed;
    boolean setHandoff;
    double tolerance;

    Shooter shooter = Shooter.getInstance();
    Intake intake = Intake.getInstance();

    public shootToVelocity(double shootSpeed, boolean setHandoff, double tolerance){
        this.shootSpeed = shootSpeed;
        this.setHandoff = setHandoff;
        this.tolerance = tolerance;
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute(){
        shooter.setShooterSpeed(shootSpeed);
        intake.setHandoff(setHandoff);

    }

    @Override
    public void end(boolean interrupt){
        shooter.setShooterSpeed(0);
        intake.setHandoff(false);
    }

    @Override
    public boolean isFinished(){
        return Math.abs(shootSpeed -shooter.getShooterVelocity()) <= tolerance;
    }

}

package frc.robot.Auton.Commands;

import java.io.Console;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class chargeShoot extends Command{
    
    @Override 
    public void initialize(){
        Shooter.getInstance().setShooter(Constants.shooterChargePower);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}

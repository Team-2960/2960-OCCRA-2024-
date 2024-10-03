package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    private static Shooter shooter = null;

    private CANSparkMax shooterMotor;

    private Shooter(){
        shooterMotor = new CANSparkMax(Constants.shooterMotorID, MotorType.kBrushless);
    }

    public void setShooter(double setSpeed){
        shooterMotor.set(setSpeed);
    }

    public static Shooter getInstance(){
        if (shooter == null){
            shooter = new Shooter();
        }
        return shooter;
    }
}

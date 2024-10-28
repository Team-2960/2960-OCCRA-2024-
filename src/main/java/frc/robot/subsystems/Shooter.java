package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    private static Shooter shooter = null;

    private CANSparkFlex shooterMotor;
    //private DigitalInput shooterPhotoeye;

    private Shooter(){
        shooterMotor = new CANSparkFlex(Constants.shooterMotorID, MotorType.kBrushless);
        //shooterPhotoeye = new DigitalInput(Constants.shooterPhotoeye);
        shooterMotor.setIdleMode(IdleMode.kBrake);
        shooterMotor.setInverted(true);
    }

    /**
     * Sets shooter speed
     * @param setSpeed
     */
    public void setShooter(double setSpeed){
       shooterMotor.set(setSpeed * Constants.shooterChargePower);
    }

    /**
     * Reverses shooter (TODO remove if this breaks code)
     * @param speed
     */
    public void revShootSpeed(boolean enable){
        if (enable){
        shooterMotor.set(Constants.reverseShooterPower);
    }else{
        shooterMotor.set(0);
    }
}

    /**
     * Sets shooter power
     * @param enable
     */
    public void enableShooter(boolean enable){
        if (enable){
            shooterMotor.set(Constants.shooterPower);
        }else{
            shooterMotor.set(0);
        }
    }

    public boolean isShooterAtSpeed(double desiredVelocity){
        if (shooterMotor.getAbsoluteEncoder().getVelocity() == desiredVelocity){
            return true;
        }else{
            return false;
        }
    }

    /* 
    public boolean isBallShot(){
        return shooterPhotoeye.get();
    }
    */
    
    public static Shooter getInstance(){
        if (shooter == null){
            shooter = new Shooter();
        }
        return shooter;
    }
}

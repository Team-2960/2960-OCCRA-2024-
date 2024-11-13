package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    private static Shooter shooter = null;

    private CANSparkFlex shooterMotor;
    //private DigitalInput shooterPhotoeye;
    private SimpleMotorFeedforward shooterFF;

    private GenericEntry sbShooterVoltage;
    private GenericEntry sbShooterVelocity;


    private Shooter(){
        shooterMotor = new CANSparkFlex(Constants.shooterMotorID, MotorType.kBrushless);
        //shooterPhotoeye = new DigitalInput(Constants.shooterPhotoeye);
        shooterMotor.setIdleMode(IdleMode.kBrake);
        shooterMotor.setInverted(true);

        shooterFF = new SimpleMotorFeedforward(0, 0.00195);

        var shooterLayout = Shuffleboard.getTab("Shooter")
            .getLayout("Shooter", BuiltInLayouts.kList)
            .withSize(1, 4);

        sbShooterVoltage = shooterLayout.add("Shooter Volatge", shooterMotor.getBusVoltage()).getEntry();
        sbShooterVelocity = shooterLayout.add("Shooter Velocity", shooterMotor.getEncoder().getVelocity()).getEntry();
    }

    /**
     * Sets shooter speed
     * @param setSpeed
     */
    public void setShooter(double setSpeed){
       shooterMotor.set(setSpeed);
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
            setShooterSpeed(Constants.shooterRPM);
        }else{
            shooterMotor.set(0);
        }
    }

    public void setShooterSpeed(double targetRPM){
        double voltage = shooterFF.calculate(targetRPM);
        shooterMotor.setVoltage(voltage);
    }

    public boolean isShooterAtSpeed(double desiredVelocity){
        if (shooterMotor.getAbsoluteEncoder().getVelocity() == desiredVelocity){
            return true;
        }else{
            return false;
        }
    }

    public double getShooterVelocity(){
        return shooterMotor.getAbsoluteEncoder().getVelocity();
    }

    /* 
    public boolean isBallShot(){
        return shooterPhotoeye.get();
    }
    */
    
    public void updateUI(){
        sbShooterVoltage.setDouble(shooterMotor.getBusVoltage());
        sbShooterVelocity.setDouble(shooterMotor.getEncoder().getVelocity());
    }

    @Override
    public void periodic(){
        updateUI();
    }

    public static Shooter getInstance(){
        if (shooter == null){
            shooter = new Shooter();
        }
        return shooter;
    }
}

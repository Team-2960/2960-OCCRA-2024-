package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pneumatics extends SubsystemBase{
    private static Pneumatics pneumatics = null;
    private Compressor compressor;

    Pneumatics(){
        compressor = new Compressor(20, PneumaticsModuleType.CTREPCM);
        compressor.enableDigital();
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Pressure", compressor.getPressure());
        SmartDashboard.putNumber("Compressor Current", compressor.getCurrent());
    }

    public static Pneumatics getInstance(){
        if (pneumatics == null){
            pneumatics = new Pneumatics();
        }
        return pneumatics;
    }
}

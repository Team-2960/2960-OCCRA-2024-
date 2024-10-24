package frc.robot.subsystems;



import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkRelativeEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.WheelPositions;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.gyrosetup.BNO055;


public class Drive extends SubsystemBase { 
    private static Drive drive = null;
    private CANSparkMax leftMotor1;
    private CANSparkMax leftMotor2;
    private CANSparkMax rightMotor1;
    private CANSparkMax rightMotor2;

    private RelativeEncoder leftEncoder;
    private RelativeEncoder rightEncoder;
    private Encoder encoder;

    private static BNO055 imu;
    private BNO055.CalData calibration;

    private PIDController leftPID;
    private PIDController rightPID;

    private DifferentialDriveKinematics kinematics;

    private DifferentialDrivePoseEstimator poseEstimator;

    private SimpleMotorFeedforward feedforward;

    private PIDController leftPidController;
    private PIDController rightPidController;

    public AutoBuilder autoBuilder;

    public int number = 4;

    

    private Drive(){
        //Initialize Drive Motors
        leftMotor1 = new CANSparkMax(Constants.leftMotor1ID, MotorType.kBrushless);
        leftMotor2 = new CANSparkMax(Constants.leftMotor2ID, MotorType.kBrushless);
        rightMotor1 = new CANSparkMax(Constants.rightMotor1ID, MotorType.kBrushless);
        rightMotor2 = new CANSparkMax(Constants.rightMotor2ID, MotorType.kBrushless);

        //leftMotor2.follow(leftMotor1);
        //rightMotor2.follow(rightMotor1);

        //Invert One side of the chassis
        leftMotor1.setInverted(true);
        leftMotor2.setInverted(true);

        //Encoders are based off of the values from the motor's HallSensor. There is no encoder on the wheel's shaft, it is just straight NEO motor values.
        leftEncoder = leftMotor1.getEncoder(Type.kHallSensor, 42);
        rightEncoder = rightMotor1.getEncoder(Type.kHallSensor, 42);
        
        //Initialize Gyroscope
        imu = BNO055.getInstance(BNO055.opmode_t.OPERATION_MODE_IMUPLUS,
            BNO055.vector_type_t.VECTOR_EULER);

        //Initialize kinematics and Pose Estimator
        kinematics = new DifferentialDriveKinematics(Constants.trackWidth);

        poseEstimator = new DifferentialDrivePoseEstimator(kinematics, Rotation2d.fromDegrees(imu.getHeading()), 
            leftEncoder.getPosition() * Constants.driveGearRatio * Constants.wheelCirc, 
            rightEncoder.getPosition() * Constants.driveGearRatio * Constants.wheelCirc, 
            new Pose2d());

        //feedforward and PID values
        feedforward = new SimpleMotorFeedforward(1, 3);
        leftPidController = new PIDController(1, 0, 0);
        rightPidController = new PIDController(1, 0, 0);

        //Configuration of the Ramsete Controller used to control the robot in PathPlanner Autons.
        autoBuilder = new AutoBuilder();
        AutoBuilder.configureRamsete(
            this::getPose, // Robot pose supplier
            this::presetPosition, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getCurrentSpeeds, // Current ChassisSpeeds supplier
            this::drive, // Method that will drive the robot given ChassisSpeeds
            new ReplanningConfig(), // Default path replanning config. See the API for the options here
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
        );
    }

    //Set left and right side speeds with option of turbo
    public void setPower(double leftPower, double rightPower, boolean turbo){
        if (turbo){
            rightMotor1.set(rightPower * Constants.turboMode);
            rightMotor2.set(rightPower * Constants.turboMode);
            leftMotor1.set(leftPower * Constants.turboMode);
            leftMotor2.set(leftPower * Constants.turboMode);
        }else{
            rightMotor1.set(rightPower * Constants.normalMode);
            rightMotor2.set(rightPower * Constants.normalMode);
            leftMotor1.set(leftPower * Constants.normalMode);
            leftMotor2.set(leftPower * Constants.normalMode);
        }
    }

    /**
     * Sets the voltage of the drive motors
     * @param leftVoltage
     * @param rightVoltage
     */
    //Set voltage of drive motors
    public void setVoltage(double leftVoltage, double rightVoltage){
        rightMotor1.setVoltage(rightVoltage);
        rightMotor2.setVoltage(rightVoltage);
        leftMotor1.setVoltage(leftVoltage);
        leftMotor2.setVoltage(leftVoltage);
    }

    //Set speeds of chassis motors based on calculation by feeding DifferentialDriveWheelSpeeds into feedforwards and PID Controllers
    public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
        final double leftFeedforward =
            feedforward.calculate(speeds.leftMetersPerSecond);
        final double rightFeedforward =
            feedforward.calculate(speeds.rightMetersPerSecond);
    
        final double leftOutput =
            leftPidController.calculate(leftEncoder.getVelocity(), speeds.leftMetersPerSecond);
        final double rightOutput =
            rightPidController.calculate(rightEncoder.getVelocity(), speeds.rightMetersPerSecond);
        leftMotor1.setVoltage(leftOutput + leftFeedforward);
        rightMotor1.setVoltage(rightOutput + rightFeedforward);
    }

    //Converts ChassisSpeeds to DifferentialDriveWheelSpeeds using kinematics function, then input the speed into the setSpeeds method.
    public void drive(ChassisSpeeds chassisSpeeds){
        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);
        setSpeeds(wheelSpeeds);
    }

    //Method used to preset the position of the robot. Pose estimator applies an offset to the gyroscope, resets the encoders, 
    //presets the your pose on the field based on the parameters you give the method.
    public void presetPosition(Pose2d pose2d){
        poseEstimator.resetPosition(Rotation2d.fromDegrees(imu.getHeading()), 
            new DifferentialDriveWheelPositions(leftEncoder.getPosition() * Constants.wheelCirc, 
            rightEncoder.getPosition() * Constants.wheelCirc),
            pose2d);
    }

    //Method that gets the pose from the poseEstimator, mainly created for the pathPlanner to call.
    public Pose2d getPose(){
        return poseEstimator.getEstimatedPosition();
    }

    //Method to return the motor velocities, motor velocities (Rotations per minute divided by 60) altered using wheel Circumfrence and gearbox gear ratio to accomodate for the robot.
    public ChassisSpeeds getCurrentSpeeds(){
        return kinematics.toChassisSpeeds(new DifferentialDriveWheelSpeeds((leftEncoder.getVelocity() * Constants.wheelCirc * Constants.driveGearRatio)/60, 
            (rightEncoder.getVelocity() * Constants.wheelCirc * Constants.driveGearRatio)/60));
    }

    public void updatePose(){
        poseEstimator.update(Rotation2d.fromDegrees(imu.getHeading()), 
            new DifferentialDriveWheelPositions(leftEncoder.getPosition() * Constants.driveGearRatio * Constants.wheelCirc, 
                rightEncoder.getPosition() * Constants.driveGearRatio * Constants.wheelCirc));
    }
    @Override
    public void periodic(){
        updatePose();
        Pose2d pose = poseEstimator.update(Rotation2d.fromDegrees(imu.getHeading()), 
            new DifferentialDriveWheelPositions(leftEncoder.getPosition() * Constants.driveGearRatio * Constants.wheelCirc, 
                rightEncoder.getPosition() * Constants.driveGearRatio * Constants.wheelCirc));
        
        //Motor Values
        SmartDashboard.putNumber("LMotor1 Pos", leftEncoder.getPosition());
        SmartDashboard.putNumber("RMotor1 Pos", rightEncoder.getPosition());

        SmartDashboard.putNumber("LMotor1 Velocity", leftEncoder.getVelocity());
        SmartDashboard.putNumber("RMotor1 Velocity", rightEncoder.getVelocity());

        //SmartDashboard.putNumber("LeftMotor1 V", leftMotor1.getBusVoltage());
        //SmartDashboard.putNumber("LeftMotor2 V", leftMotor2.getBusVoltage());

        /* 
        //Gyroscope Values
        SmartDashboard.putBoolean("Gyro Initialized", imu.isInitialized());
        SmartDashboard.putBoolean("Gyro Calibrated", imu.isCalibrated());
        SmartDashboard.putBoolean("Gyro Present", imu.isSensorPresent());
        SmartDashboard.putNumberArray("Gyro Vector", imu.getVector());
        SmartDashboard.putNumber("Gyro Heading", imu.getHeading());
        */
        
        //Pose Estimator Values
        SmartDashboard.putNumber("X Pose", pose.getX());
        SmartDashboard.putNumber("Y Pose", pose.getY());
        SmartDashboard.putNumber("Rotation Pose", getPose().getRotation().getDegrees());
        
        //Chassis Speed Values
        SmartDashboard.putNumber("X Velocity", getCurrentSpeeds().vxMetersPerSecond);
        SmartDashboard.putNumber("Y Velocity", getCurrentSpeeds().vyMetersPerSecond);
        SmartDashboard.putNumber("Angular Velocity", Math.toDegrees(getCurrentSpeeds().omegaRadiansPerSecond));
        
    }

    public static Drive getInstance(){
        if(drive == null){
            drive = new Drive();
        }
        return drive;
    }
    
}
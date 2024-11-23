package frc.robot.subsystems;



import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkRelativeEncoder.Type;

import edu.wpi.first.math.MathUtil;
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
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
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
    private PIDController distanceController;
    private PIDController rotationController;

    private GenericEntry sbPoseX;
    private GenericEntry sbPoseY;
    private GenericEntry sbRotationPose;
    private GenericEntry sbRawRotation;

    private GenericEntry sbLeftVoltage;
    private GenericEntry sbRightVoltage;
    private GenericEntry sbLeftVelocity;
    private GenericEntry sbRightVelocity;
    private GenericEntry sbLeftPosition;
    private GenericEntry sbRightPosition;
    private GenericEntry sbLeftOutput;
    private GenericEntry sbRightOutput;
    private GenericEntry sbRightDistanceOutput;
    private GenericEntry sbLeftDistanceOutput;
    private GenericEntry sbDistanceController;

    public AutoBuilder autoBuilder;

    private double rawRotation2d;
    private Rotation2d currentRotation2d;

    private double leftOutput;
    private double rightOutput;

    private double leftDistanceOutput;
    private double rightDistanceOutput;

    private double distancePIDController;
    

    

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
        leftEncoder.setVelocityConversionFactor(1/60);
        rightEncoder.setVelocityConversionFactor(1/60);
        
        //Initialize Gyroscope
        imu = BNO055.getInstance(BNO055.opmode_t.OPERATION_MODE_IMUPLUS,
            BNO055.vector_type_t.VECTOR_EULER);

        //Initialize kinematics and Pose Estimator
        kinematics = new DifferentialDriveKinematics(Constants.trackWidth);
        rawRotation2d = imu.getHeading();
        currentRotation2d = Rotation2d.fromDegrees(rawRotation2d);
        poseEstimator = new DifferentialDrivePoseEstimator(kinematics, currentRotation2d, 
            getLeftPosition(), 
            getRightPosition(), 
            new Pose2d());

        //feedforward and PID values
        feedforward = new SimpleMotorFeedforward(0, 2.16, 0.39);
        leftPidController = new PIDController(0, 0, 0);
        rightPidController = new PIDController(0, 0, 0);
        distanceController = new PIDController(1.7, 0, 0);
        rotationController = new PIDController(0.06, 0, 0.006);// 0.052, 0.007, 0.008 
        rotationController.enableContinuousInput(-180, 180);

        leftOutput = 0;
        rightOutput = 0;
        leftDistanceOutput = 0;
        rightDistanceOutput = 0;
        distancePIDController = 0;
        var poseLayout = Shuffleboard.getTab("Drive")
            .getLayout("Drive Pose", BuiltInLayouts.kList)
            .withSize(1, 4);
        var motorLayout = Shuffleboard.getTab("Drive").
            getLayout("Drive Motors", BuiltInLayouts.kList)
            .withSize(1, 4);
            
        sbPoseX = poseLayout.add("Pose X", poseEstimator.getEstimatedPosition().getX()).getEntry();
        sbPoseY = poseLayout.add("Pose Y", poseEstimator.getEstimatedPosition().getY()).getEntry();
        sbRotationPose = poseLayout.add("Pose Rotation", poseEstimator.getEstimatedPosition().getRotation().getDegrees()).getEntry();
        sbRawRotation = poseLayout.add("Raw Rotation", rawRotation2d).getEntry();

        sbLeftVoltage = motorLayout.add("Left Voltage", leftMotor1.getBusVoltage()).getEntry();
        sbRightVoltage = motorLayout.add("Right Voltage", rightMotor1.getBusVoltage()).getEntry();
        sbLeftVelocity = motorLayout.add("Left Velocity", getLeftVelocity()).getEntry();
        sbRightVelocity = motorLayout.add("Right Velocity", getRightVelocity()).getEntry();
        sbLeftPosition = motorLayout.add("Left Position", getLeftPosition()).getEntry();
        sbRightPosition = motorLayout.add("Right Position", getRightPosition()).getEntry();
        sbLeftOutput = motorLayout.add("Left Output (FF)", leftOutput).getEntry();
        sbRightOutput = motorLayout.add("Right Output (FF)", rightOutput).getEntry();
        sbLeftDistanceOutput = motorLayout.add("Left Distance output", leftDistanceOutput).getEntry();
        sbRightDistanceOutput = motorLayout.add("Right Distance Output", rightDistanceOutput).getEntry();
        sbDistanceController = motorLayout.add("Distance PID Controller", distancePIDController).getEntry();

        

        

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

    public void arcadeDrive(double leftJoy, double rightJoy, boolean turbo){
        double leftCalc = Math.max(Math.min(leftJoy - rightJoy, 1), -1);
        double rightCalc = Math.max(Math.min(leftJoy + rightJoy, 1), -1);
        if (!turbo){
            leftCalc *= Constants.normalMode;
            rightCalc *= Constants.normalMode;
        }else{
            leftCalc *= Constants.turboMode;
            rightCalc *= Constants.turboMode;
        }
        leftMotor1.set(leftCalc);
        leftMotor2.set(leftCalc);
        rightMotor1.set(rightCalc);
        rightMotor2.set(rightCalc);
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
            feedforward.calculate(-speeds.leftMetersPerSecond);
        final double rightFeedforward =
            feedforward.calculate(-speeds.rightMetersPerSecond);
    
        final double leftOutput =
            leftPidController.calculate(getLeftVelocity(), -speeds.leftMetersPerSecond);
        final double rightOutput =
            rightPidController.calculate(getRightVelocity(), -speeds.rightMetersPerSecond);
        double leftVoltage;
        double rightVoltage;
        if ((leftOutput + leftFeedforward) > 3.5){
            leftVoltage = 3.4;
        }else if((leftOutput + leftFeedforward) < -3.5){
            leftVoltage = -3.4;
        }else{
            leftVoltage = leftOutput + leftFeedforward;
        }

        if ((rightOutput + rightFeedforward) > 3.5){
            rightVoltage = 3.5;
        }else if ((rightOutput + rightFeedforward) < -3.5){
            rightVoltage = -3.5;
        }else{
            rightVoltage = rightOutput + rightFeedforward;
        }

        leftMotor1.setVoltage((leftVoltage));
        leftMotor2.setVoltage((leftVoltage));
        rightMotor1.setVoltage((rightVoltage));
        rightMotor2.setVoltage((rightVoltage));
        this.leftOutput = leftOutput + leftFeedforward;
        this.rightOutput = rightOutput + rightFeedforward;
    }

    public void driveDistance(double leftGoal, double rightGoal, double leftInitial, double rightInitial){
        double leftSpeed = distanceController.calculate(getLeftPosition() - leftInitial, leftGoal);
        double rightSpeed = distanceController.calculate(getRightPosition() - rightInitial, rightGoal);
        this.leftDistanceOutput = leftInitial;
        this.rightDistanceOutput = rightInitial;
        this.distancePIDController = leftSpeed;
        setSpeeds(new DifferentialDriveWheelSpeeds(leftSpeed, rightSpeed));

    }

    public void driveToRotation(double goalRotation){
        double curRotation = poseEstimator.getEstimatedPosition().getRotation().getDegrees();
        double rotationCalc = rotationController.calculate(curRotation, goalRotation);
        setSpeeds(new DifferentialDriveWheelSpeeds(rotationCalc, -rotationCalc));
    }

    public double getLeftPosition(){
        return -leftEncoder.getPosition() * Constants.driveGearRatio * Constants.wheelCirc;
    }

    public double getRightPosition(){
        return -rightEncoder.getPosition() * Constants.driveGearRatio * Constants.wheelCirc;
    }

    public double getLeftVelocity(){
        return -leftEncoder.getVelocity() * Constants.driveGearRatio;
    }

    public double getRightVelocity(){
        return -rightEncoder.getVelocity() * Constants.driveGearRatio;
    }
    //Converts ChassisSpeeds to DifferentialDriveWheelSpeeds using kinematics function, then input the speed into the setSpeeds method.
    public void drive(ChassisSpeeds chassisSpeeds){
        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);
        setSpeeds(wheelSpeeds);
    }

    //Method used to preset the position of the robot. Pose estimator applies an offset to the gyroscope, resets the encoders, 
    //presets the your pose on the field based on the parameters you give the method.
    public void presetPosition(Pose2d pose2d){
        poseEstimator.resetPosition(currentRotation2d, 
            new DifferentialDriveWheelPositions(getLeftPosition(), 
            getRightPosition()),
            pose2d);
    }


    //Method that gets the pose from the poseEstimator, mainly created for the pathPlanner to call.
    public Pose2d getPose(){
        return poseEstimator.getEstimatedPosition();
    }

    //Method to return the motor velocities, motor velocities (Rotations per minute divided by 60) altered using wheel Circumfrence and gearbox gear ratio to accomodate for the robot.
    public ChassisSpeeds getCurrentSpeeds(){
        return kinematics.toChassisSpeeds(new DifferentialDriveWheelSpeeds(
            getLeftVelocity(), 
            getLeftVelocity()));
    }

    public void updatePose(){
        poseEstimator.update(currentRotation2d, 
            new DifferentialDriveWheelPositions(getLeftPosition(), 
                getRightPosition()));
    }

    public void setDriveIdleMode(IdleMode mode){
        leftMotor1.setIdleMode(mode);
        leftMotor2.setIdleMode(mode);
        rightMotor1.setIdleMode(mode);
        rightMotor2.setIdleMode(mode);
    }

    
    

    public void updateUI(){
        Pose2d pose = poseEstimator.getEstimatedPosition();
        sbPoseX.setDouble(pose.getX());
        sbPoseY.setDouble(pose.getY());
        sbRotationPose.setDouble(pose.getRotation().getDegrees());
        sbRawRotation.setDouble(rawRotation2d);
        
        sbLeftVoltage.setDouble(leftMotor1.getBusVoltage());
        sbRightVoltage.setDouble(rightMotor1.getBusVoltage());
        sbLeftVelocity.setDouble(getLeftVelocity());
        sbRightVelocity.setDouble(getRightVelocity());
        sbLeftPosition.setDouble(getLeftPosition());
        sbRightPosition.setDouble(getRightPosition());
        sbLeftOutput.setDouble(leftOutput);
        sbRightOutput.setDouble(rightOutput);
        sbLeftDistanceOutput.setDouble(leftDistanceOutput);
        sbRightDistanceOutput.setDouble(rightDistanceOutput);
        sbDistanceController.setDouble(distancePIDController);
    }

    @Override
    public void periodic(){
        rawRotation2d = imu.getHeading();
        currentRotation2d = Rotation2d.fromDegrees(rawRotation2d);
        updatePose();
        updateUI();
               
    }

    public static Drive getInstance(){
        if(drive == null){
            drive = new Drive();
        }
        return drive;
    }
    
}
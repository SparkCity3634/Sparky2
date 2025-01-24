package frc;

public class oldRobot {
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.RelativeEncoder;
import java.lang.Math;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  //private RobotContainer m_robotContainer1;
  private Command m_autonomousCommand;

  //These are the motor bindings needed for our drivetrain, used in the SwerveModule
  
  private final SparkMax BackRightDrive = new SparkMax(41, MotorType.kBrushless);
  private final SparkMax BackRightTurn = new SparkMax(42, MotorType.kBrushless);
  private final SparkMax FrontRightDrive = new SparkMax(11, MotorType.kBrushless);
  private final SparkMax FrontRightTurn = new SparkMax(12, MotorType.kBrushless);
  private final SparkMax FrontLeftDrive = new SparkMax(21, MotorType.kBrushless);
  private final SparkMax FrontLeftTurn = new SparkMax(22, MotorType.kBrushless);
  private final SparkMax BackLeftDrive = new SparkMax(31, MotorType.kBrushless);
  private final SparkMax BackLeftTurn = new SparkMax(32, MotorType.kBrushless);
  
  //create SparkClosedLoopControllers to fix Drive and Turn motors

  private SparkClosedLoopController m_BackLeftDrivePID = BackLeftDrive.getClosedLoopController();
  private SparkClosedLoopController m_BackRightDrivePID = BackRightDrive.getClosedLoopController();
  private SparkClosedLoopController m_FrontRightDrivePID = FrontLeftDrive.getClosedLoopController();
  private SparkClosedLoopController m_FrontLeftDrivePID = FrontRightDrive.getClosedLoopController();
  private SparkClosedLoopController m_BackLeftTurnPID = BackLeftTurn.getClosedLoopController();
  private SparkClosedLoopController m_BackRightTurnPID = BackRightTurn.getClosedLoopController();
  private SparkClosedLoopController m_FrontRightTurnPID = FrontLeftTurn.getClosedLoopController();
  private SparkClosedLoopController m_FrontLeftTurnPID = FrontRightTurn.getClosedLoopController();
  
  //create Relative encoders (from built-in motor encoders) for Turn and Drive motors
  private RelativeEncoder m_BackLeftTurnEncoder = BackLeftTurn.getEncoder();
  private RelativeEncoder m_BackRightTurnEncoder = BackRightTurn.getEncoder();
  private RelativeEncoder m_FrontRightTurnEncoder = FrontRightTurn.getEncoder();
  private RelativeEncoder m_FrontLeftTurnEncoder = FrontLeftTurn.getEncoder();
  
  //Drive Encoders are not used in this code, but are included for reference
  //private RelativeEncoder m_BackLeftDriveEncoder = BackLeftDrive.getEncoder();
  //private RelativeEncoder m_BackRightDriveEncoder = BackRightDrive.getEncoder();
  //private RelativeEncoder m_FrontRightDriveEncoder = FrontRightDrive.getEncoder();
  //private RelativeEncoder m_FrontLeftDriveEncoder = FrontLeftDrive.getEncoder();

  //Bind CANcoders for Absolute Turn Encoding
  private static final String canBusName = "rio";
  private final CANcoder m_FrontRightTurnCancoder = new CANcoder(10, canBusName);
  private final CANcoder m_FrontLeftTurnCancoder = new CANcoder(20, canBusName);
  private final CANcoder m_BackLeftTurnCancoder = new CANcoder(30, canBusName);
  private final CANcoder m_BackRightTurnCancoder = new CANcoder(40, canBusName);
  //private final DutyCycleOut fwdOut = new DutyCycleOut(0);

  //Bind Module controllers
  
  XboxController o_controller = new XboxController(1);  
  XboxController m_controller = new XboxController(0);  
  private ADIS16470_IMU m_gyro = new ADIS16470_IMU();
  UsbCamera camera1;
  UsbCamera camera2;
  VideoSink server;
  Timer m_Timer = new Timer();

  //Create Variables to store PID coefficients and other config parameters
  public double maxRPM, setTurn;
  public double kdP, kdI, kdD, kdIz, kdFF, kdMaxOutput, kdMinOutput;
  public double ktP, ktI, ktD, ktIz, ktFF, ktMaxOutput, ktMinOutput;
  public double ksP, ksI, ksD, ksIz, ksFF, ksMaxOutput, ksMinOutput;
  public double xAxis, yAxis, kYawRate, maxVel, maxYaw, o_lyAxis, o_ryAxis;
  public double k_posConv = .048; // linear meters traveled at wheel, per motor rotation
  public double k_velConv = .0008106; // linear meters per second (m/s) speed at wheel, convert from motor RPM
  public double k_turnConv = 16.8; //wheel degrees per motor rotation from steering relative encoder
  public double rotations;

  


//Create Swerve Kinematics modules
Translation2d m_BackLeftLocation = new Translation2d(-0.25 ,0.250);
Translation2d m_BackRightLocation = new Translation2d(-0.25 ,-.250);
Translation2d m_FrontLeftLocation = new Translation2d(.250,.250);
Translation2d m_FrontRightLocation = new Translation2d(.250,-.250);

// Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

public final SwerveDriveKinematics m_Kinematics = new SwerveDriveKinematics(m_BackLeftLocation, m_BackRightLocation, m_FrontLeftLocation, m_FrontRightLocation);
 
private static final String kRedAuto = "Red";
private static final String kBlueAuto = "Blue";
private String m_autoSelected;
private final SendableChooser<String> m_chooser = new SendableChooser<>();

//Create instance of fieldspeeds to store the ChassisSpeeds
ChassisSpeeds fieldspeeds = new ChassisSpeeds(0,0,0);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    //m_robotContainer1 = new RobotContainer();
    rotations = 0;
    
    //set up our USB Camera 

    m_chooser.setDefaultOption("Red Auto", kRedAuto);
    m_chooser.addOption("Blue Auto", kBlueAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    
    camera1 = CameraServer.startAutomaticCapture(0);
    camera2 = CameraServer.startAutomaticCapture(1);
    server = CameraServer.getServer();
    camera1.setResolution(320, 240);
    camera2.setResolution(320, 240);
    camera1.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
    camera2.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
    
    //Create SparkMax Config for Drive motors
SparkMaxConfig config_Drive = new SparkMaxConfig();

//Declare Drive PID coefficients 

kdP = 0.4; 
kdI = 0.00000;
kdD = 0.05; 
kdIz = 0; 
kdFF = 0.01; 
kdMaxOutput = 1; 
kdMinOutput = -1;

config_Drive
    .smartCurrentLimit(40)
    .inverted(true)
    .idleMode(IdleMode.kCoast);
config_Drive.encoder
    .positionConversionFactor(k_posConv)
    .velocityConversionFactor(k_velConv);
config_Drive.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .pid(kdP, kdI, kdD)
    .iZone(kdIz)
    .velocityFF(kdFF)
    .outputRange(kdMinOutput, kdMaxOutput);

  //Create SparkMax Config for Turn motors
   /**
    * The PID Controller can be configured to use the analog sensor as its feedback
    * device with the method SetFeedbackDevice() and passing the PID Controller
    * the SparkMaxAnalogSensor object. 
    */
  SparkMaxConfig config_Turn = new SparkMaxConfig();
  
  // Turn PID coefficients
  ktP = 0.07; 
  ktI = 0.0000;
  ktD = 0.001;
  ktIz = 0; 
  ktFF = 0.003; 
  ktMaxOutput = .9; 
  ktMinOutput = -.9;

config_Turn
    .smartCurrentLimit(40)
    .inverted(true)
    .idleMode(IdleMode.kCoast);
config_Turn.encoder
    .positionConversionFactor(k_turnConv)
    .velocityConversionFactor(k_turnConv);
config_Turn.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .pid(ktP, ktI, ktD)
    .iZone(ktIz)
    .velocityFF(ktFF)
    .outputRange(ktMinOutput, ktMaxOutput)
    .positionWrappingEnabled(true)
    .positionWrappingMinInput(0)
    .positionWrappingMaxInput(360);

    //Set Configuration for Drive motors
    BackLeftDrive.configure(config_Drive, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    BackRightDrive.configure(config_Drive, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    FrontLeftDrive.configure(config_Drive, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    FrontRightDrive.configure(config_Drive, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    //Set Configuration for Turn motors
    BackLeftTurn.configure(config_Turn, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    BackRightTurn.configure(config_Turn, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    FrontLeftTurn.configure(config_Turn, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    FrontRightTurn.configure(config_Turn, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    /* Configure CANcoder */
    //var toApply = new CANcoderConfiguration();

    /* User can change the configs if they want, or leave it empty for factory-default */
    /*m_FrontRightTurnCancoder.getConfigurator().apply(toApply);
    m_FrontLeftTurnCancoder.getConfigurator().apply(toApply);
    m_BackLeftTurnCancoder.getConfigurator().apply(toApply);
    m_BackRightTurnCancoder.getConfigurator().apply(toApply);*/



    /* Speed up signals to an appropriate rate */
    m_FrontRightTurnCancoder.getPosition().setUpdateFrequency(100);
    m_FrontRightTurnCancoder.getVelocity().setUpdateFrequency(100);
    m_FrontLeftTurnCancoder.getPosition().setUpdateFrequency(100);
    m_FrontLeftTurnCancoder.getVelocity().setUpdateFrequency(100);
    m_BackLeftTurnCancoder.getPosition().setUpdateFrequency(100);
    m_BackLeftTurnCancoder.getVelocity().setUpdateFrequency(100);
    m_BackRightTurnCancoder.getPosition().setUpdateFrequency(100);
    m_BackRightTurnCancoder.getVelocity().setUpdateFrequency(100);

     
    //Initialize RelativeEncoders to CANCoder Absolute Value
    m_FrontRightTurnEncoder.setPosition(m_FrontRightTurnCancoder.getAbsolutePosition().getValueAsDouble());//might want to add the .waitForUpdate() method to reduce latency?
    m_FrontLeftTurnEncoder.setPosition(m_FrontLeftTurnCancoder.getAbsolutePosition().getValueAsDouble());
    m_BackLeftTurnEncoder.setPosition(m_BackLeftTurnCancoder.getAbsolutePosition().getValueAsDouble());
    m_BackRightTurnEncoder.setPosition(m_BackRightTurnCancoder.getAbsolutePosition().getValueAsDouble());

    maxVel = 4; // m/s linear velocity of drive wheel
    maxYaw = 2*Math.PI;   // max rad/s for chassis rotation rate
      
    //Set Initial Swerve States
    ChassisSpeeds speeds = new ChassisSpeeds(0, 0, 0);
    SwerveModuleState[] moduleStates = m_Kinematics.toSwerveModuleStates(speeds);
    SwerveModuleState BackLeftSwerve = moduleStates[0];
    SwerveModuleState BackRightSwerve = moduleStates[1];
    SwerveModuleState FrontLeftSwerve = moduleStates[2];
    SwerveModuleState FrontRightSwerve = moduleStates[3];
    
    //Optimize Swerve Module States using Deprecated Static Method
    //SwerveModuleState BackLeftOptimized = SwerveModuleState.optimize(BackLeftSwerve, Rotation2d.fromDegrees(MathUtil.inputModulus(m_BackLeftTurnEncoder.getPosition(),0 , 360)));
    //SwerveModuleState BackRightOptimized = SwerveModuleState.optimize(BackRightSwerve, Rotation2d.fromDegrees(MathUtil.inputModulus(m_BackRightTurnEncoder.getPosition(),0 , 360)));
    //SwerveModuleState FrontLeftOptimized = SwerveModuleState.optimize(FrontLeftSwerve, Rotation2d.fromDegrees(MathUtil.inputModulus(m_FrontLeftTurnEncoder.getPosition(),0 , 360)));
    //SwerveModuleState FrontRightOptimized = SwerveModuleState.optimize(FrontRightSwerve, Rotation2d.fromDegrees(MathUtil.inputModulus(m_FrontRightTurnEncoder.getPosition(),0 , 360)));
   
    //Optimize Swerve Module States using Instance Method
    BackLeftSwerve.optimize(Rotation2d.fromDegrees(m_BackLeftTurnEncoder.getPosition()));
    BackRightSwerve.optimize(Rotation2d.fromDegrees(m_BackRightTurnEncoder.getPosition()));
    FrontLeftSwerve.optimize(Rotation2d.fromDegrees(m_FrontLeftTurnEncoder.getPosition()));
    FrontRightSwerve.optimize(Rotation2d.fromDegrees(m_FrontRightTurnEncoder.getPosition()));
    
    m_gyro.reset();
    
    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kdP);
    //SmartDashboard.putNumber("Get P", m_BackLeftDrivePID.getP());
    SmartDashboard.putNumber("I Gain", kdI);
    //SmartDashboard.putNumber("Get I", m_BackLeftDrivePID.getI());
    SmartDashboard.putNumber("D Gain", kdD);    
    
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    double tx = LimelightHelpers.getTX("");
    System.out.println("The value of tx is: " + tx);
    /**Cancoder Abs Position updates the SparkMax Relative Encoders to reduce drift
       * getPosition automatically calls refresh(), no need to manually refresh.
       * 
       * StatusSignalValues also have the toString method implemented, to provide
       * a useful print of the signal.  Need to use the getValueAsDouble() method 
       * to convert to double for use in other methods.
       */
      
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    System.out.println("Auto selected: " + m_autoSelected);
    /*m_autonomousCommand = m_robotContainer1.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    
    //insert Auto initiatlization here, set Drive control to position
    */
    m_BackRightTurnPID.setReference(0, SparkMax.ControlType.kPosition);
    m_FrontRightTurnPID.setReference(0, SparkMax.ControlType.kPosition);
    m_FrontLeftTurnPID.setReference(0, SparkMax.ControlType.kPosition);
    m_BackLeftTurnPID.setReference(0, SparkMax.ControlType.kPosition);  
      
    m_BackLeftDrivePID.setReference(0, SparkMax.ControlType.kVelocity);
    m_BackRightDrivePID.setReference(0, SparkMax.ControlType.kVelocity);
    m_FrontLeftDrivePID.setReference(0, SparkMax.ControlType.kVelocity);
    m_FrontRightDrivePID.setReference(0, SparkMax.ControlType.kVelocity);
    m_Timer.restart();

  }
  /** This function is called periodically during autonomous. */
 @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kRedAuto:
      /*if(m_Timer.hasElapsed(2) != true) {
        //shooterLeft.set(-0.7);
        //shooterRight.set(0.4);
        //m_ShooterLeftPID.setReference(-.6 * maxShootVelocity, SparkMax.ControlType.kVelocity);
       // m_ShooterRightPID.setReference(-.3 * maxShootVelocity, SparkMax.ControlType.kVelocity);
      }
      else if(m_Timer.hasElapsed(5) != true) {
        intakeTop.set(1);
      }
      else if(m_Timer.hasElapsed(6) != true) {
        //shooterLeft.set(0.0);  //shooter stops
        //shooterRight.set(0.0);
        m_ShooterLeftPID.setReference(0.0, SparkMax.ControlType.kVelocity);
        m_ShooterRightPID.setReference(0.0, SparkMax.ControlType.kVelocity);
        intakeBot.set(0);
        intakeTop.set(0);    
      }
      else if(m_Timer.hasElapsed(10) != true) {
        m_BackRightTurnPID.setReference(-25, SparkMax.ControlType.kPosition);
        m_FrontRightTurnPID.setReference(-25, SparkMax.ControlType.kPosition);
        m_FrontLeftTurnPID.setReference(-25, SparkMax.ControlType.kPosition);
        m_BackLeftTurnPID.setReference(-25, SparkMax.ControlType.kPosition);  
      
        m_BackLeftDrivePID.setReference(-1, SparkMax.ControlType.kVelocity);
        m_BackRightDrivePID.setReference(-1, SparkMax.ControlType.kVelocity);
        m_FrontLeftDrivePID.setReference(-1, SparkMax.ControlType.kVelocity);
        m_FrontRightDrivePID.setReference(-1, SparkMax.ControlType.kVelocity);
      }
      else{
          m_BackRightTurnPID.setReference(0, SparkMax.ControlType.kPosition);
          m_FrontRightTurnPID.setReference(0, SparkMax.ControlType.kPosition);
          m_FrontLeftTurnPID.setReference(0, SparkMax.ControlType.kPosition);
          m_BackLeftTurnPID.setReference(0, SparkMax.ControlType.kPosition);  
      
          m_BackLeftDrivePID.setReference(0, SparkMax.ControlType.kVelocity);
          m_BackRightDrivePID.setReference(0, SparkMax.ControlType.kVelocity);
          m_FrontLeftDrivePID.setReference(0, SparkMax.ControlType.kVelocity);
          m_FrontRightDrivePID.setReference(0, SparkMax.ControlType.kVelocity);
      }
       
      break;
      case kBlueAuto:
      if(m_Timer.hasElapsed(2) != true) {
        m_ShooterLeftPID.setReference(-.6 * maxShootVelocity, SparkMax.ControlType.kVelocity);
        m_ShooterRightPID.setReference(-.3 * maxShootVelocity, SparkMax.ControlType.kVelocity);
      }
      else if(m_Timer.hasElapsed(5) != true) {
        intakeTop.set(1);
      }
      else if(m_Timer.hasElapsed(6) != true) {
        m_ShooterLeftPID.setReference(0.0 * maxShootVelocity, SparkMax.ControlType.kVelocity);
        m_ShooterRightPID.setReference(0.0 * maxShootVelocity, SparkMax.ControlType.kVelocity);
        intakeBot.set(0);
        intakeTop.set(0);    
      }
      else if(m_Timer.hasElapsed(10) != true) {
        m_BackRightTurnPID.setReference(25, SparkMax.ControlType.kPosition);
        m_FrontRightTurnPID.setReference(25, SparkMax.ControlType.kPosition);
        m_FrontLeftTurnPID.setReference(25, SparkMax.ControlType.kPosition);
        m_BackLeftTurnPID.setReference(25, SparkMax.ControlType.kPosition);  
      
        m_BackLeftDrivePID.setReference(-1, SparkMax.ControlType.kVelocity);
        m_BackRightDrivePID.setReference(-1, SparkMax.ControlType.kVelocity);
        m_FrontLeftDrivePID.setReference(-1, SparkMax.ControlType.kVelocity);
        m_FrontRightDrivePID.setReference(-1, SparkMax.ControlType.kVelocity);
      }
      else{
          m_BackRightTurnPID.setReference(0, SparkMax.ControlType.kPosition);
          m_FrontRightTurnPID.setReference(0, SparkMax.ControlType.kPosition);
          m_FrontLeftTurnPID.setReference(0, SparkMax.ControlType.kPosition);
          m_BackLeftTurnPID.setReference(0, SparkMax.ControlType.kPosition);  
      
          m_BackLeftDrivePID.setReference(0, SparkMax.ControlType.kVelocity);
          m_BackRightDrivePID.setReference(0, SparkMax.ControlType.kVelocity);
          m_FrontLeftDrivePID.setReference(0, SparkMax.ControlType.kVelocity);
          m_FrontRightDrivePID.setReference(0, SparkMax.ControlType.kVelocity);
      }
      break;
       */
    }
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    
    m_gyro.reset();
    
    maxVel = 4; // m/s linear velocity of drive wheel
    maxYaw = 2*Math.PI;   // max rad/s for chassis rotation rate
    
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    
    SmartDashboard.putNumber("Gyro", (m_gyro.getAngle()+180));    
    
    //Get the Direction input from the XBox controller left stick
    xAxis = MathUtil.applyDeadband(-m_controller.getLeftY(), .2)*maxVel; // linear m/s
    yAxis = MathUtil.applyDeadband(-m_controller.getLeftX(), .2)*maxVel; // linear m/s
    kYawRate = MathUtil.applyDeadband(-m_controller.getRightX(), .2)*maxYaw; // rad/s
    o_lyAxis = MathUtil.applyDeadband(-o_controller.getLeftY(), .2); // linear m/s
    o_ryAxis = MathUtil.applyDeadband(-o_controller.getRightY(), .2); // linear m/s

    final var xSpeed =
        -m_xspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftY(), 0.2))
            * maxVel;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    final var ySpeed =
        -m_yspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftX(), 0.2))
            * maxVel;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    final var rot =
        -m_rotLimiter.calculate(MathUtil.applyDeadband(m_controller.getRightX(), 0.2))
          *maxYaw;

      /*Calculate Swerve Module States based on controller readings
      if left bumper is pressed Drive will be Robot Relative
      otherwise Drive is Field Centric
      */

      if(m_controller.getLeftBumperButton()){
        fieldspeeds =  new ChassisSpeeds(xAxis, yAxis, rot);
      }
      else {
        fieldspeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Rotation2d.fromDegrees(m_gyro.getAngle()+180));
      }
    SwerveModuleState[] moduleStates = m_Kinematics.toSwerveModuleStates(fieldspeeds);
    SwerveModuleState BackLeftSwerve = moduleStates[0];
    SwerveModuleState BackRightSwerve = moduleStates[1];
    SwerveModuleState FrontLeftSwerve = moduleStates[2];
    SwerveModuleState FrontRightSwerve = moduleStates[3];
    
    //Optimize Swerve Module States using Deprecated Static Method
    //var BackLeftOptimized = SwerveModuleState.optimize(BackLeftSwerve, Rotation2d.fromDegrees(m_BackLeftTurnEncoder.getPosition()));
    //var BackRightOptimized = SwerveModuleState.optimize(BackRightSwerve, Rotation2d.fromDegrees(m_BackRightTurnEncoder.getPosition()));
    //var FrontLeftOptimized = SwerveModuleState.optimize(FrontLeftSwerve, Rotation2d.fromDegrees(m_FrontLeftTurnEncoder.getPosition()));
    //var FrontRightOptimized = SwerveModuleState.optimize(FrontRightSwerve, Rotation2d.fromDegrees(m_FrontRightTurnEncoder.getPosition()));
    
    //Optimize Swerve Module States using Instance Method
    BackLeftSwerve.optimize(Rotation2d.fromDegrees(m_BackLeftTurnEncoder.getPosition()));
    BackRightSwerve.optimize(Rotation2d.fromDegrees(m_BackRightTurnEncoder.getPosition()));
    FrontLeftSwerve.optimize(Rotation2d.fromDegrees(m_FrontLeftTurnEncoder.getPosition()));
    FrontRightSwerve.optimize(Rotation2d.fromDegrees(m_FrontRightTurnEncoder.getPosition()));
    
    /**
     * PIDController objects are commanded to a set point using the 
     * SetReference() method and the reference value of the optimized swerve mfodule states
     * 
     * The first parameter is the value of the set point, whose units vary
     * depending on the control type set in the second parameter.
     * 
     * The second parameter is the control type, can be set to one of four 
     * parameters:
     * com.revrobotics.SparkMax.ControlType.kDutyCycle
     *  com.revrobotics.SparkMax.ControlType.kPosition
     *  com.revrobotics.SparkMax.ControlType.kVelocity
     *  com.revrobotics.SparkMax.ControlType.kVoltage
     */
    
    //Set actual motor output values to drive

    //angle and SpeedMeter not
    m_BackRightTurnPID.setReference(BackRightSwerve.angle.getDegrees(), SparkMax.ControlType.kPosition);
    m_FrontRightTurnPID.setReference(FrontRightSwerve.angle.getDegrees(), SparkMax.ControlType.kPosition);
    m_FrontLeftTurnPID.setReference(FrontLeftSwerve.angle.getDegrees(), SparkMax.ControlType.kPosition);
    m_BackLeftTurnPID.setReference(BackLeftSwerve.angle.getDegrees(), SparkMax.ControlType.kPosition);  
    m_BackLeftDrivePID.setReference(BackLeftSwerve.speedMetersPerSecond, SparkMax.ControlType.kVelocity);
    m_BackRightDrivePID.setReference(BackRightSwerve.speedMetersPerSecond, SparkMax.ControlType.kVelocity);
    m_FrontLeftDrivePID.setReference(FrontLeftSwerve.speedMetersPerSecond, SparkMax.ControlType.kVelocity);
    m_FrontRightDrivePID.setReference(FrontRightSwerve.speedMetersPerSecond, SparkMax.ControlType.kVelocity);
        
        
    /* //TURN DRIVE MOTORS OFF
    BackLeftDrive.set(0);
    BackRightDrive.set(0);
    FrontRightDrive.set(0);
    FrontLeftDrive.set(0);
    */

      //Last years mechanisms 

    //Run Intake
    //intakeBot.set((m_controller.getRightTriggerAxis() - m_controller.getLeftTriggerAxis())*0.7);
    //intakeTop.set(m_controller.getRightTriggerAxis() - m_controller.getLeftTriggerAxis());
    
   // hangLeft.set(o_lyAxis*0.7);
   // hangRight.set(o_ryAxis*0.7);

    //Set Shooter Speeds for Speaker or Amp Load
    if(o_controller.getXButton()){ //shooter for reverse
      //shooterLeft.set(.2);
      //shooterRight.set(-0.2);
      //m_ShooterLeftPID.setReference(0.1 * maxShootVelocity, SparkMax.ControlType.kVelocity);
      //m_ShooterRightPID.setReference(0.1 * maxShootVelocity, SparkMax.ControlType.kVelocity);
    }
    else if(o_controller.getAButton()){  //shooter speaker
      //shooterLeft.set(-0.7);
      //shooterRight.set(0.4);  
    //m_ShooterLeftPID.setReference(-0.6 * maxShootVelocity, SparkMax.ControlType.kVelocity);
      //m_ShooterRightPID.setReference(-0.3 * maxShootVelocity, SparkMax.ControlType.kVelocity);  
    }
    else if(o_controller.getYButton()){  //slow shooter to load for amp/trap
      //shooterLeft.set(-0.2);`
      //shooterRight.set(0.2);
      //m_ShooterLeftPID.setReference(-0.2 * maxShootVelocity, SparkMax.ControlType.kVelocity);
     // m_ShooterRightPID.setReference(-0.2 * maxShootVelocity, SparkMax.ControlType.kVelocity);
    }
    else{
     //shooterLeft.set(0.0);  //shooter stops
     //shooterRight.set(0.0);
    // m_ShooterLeftPID.setReference(0, SparkMax.ControlType.kVelocity);
    // m_ShooterRightPID.setReference(0, SparkMax.ControlType.kVelocity);
    }

    if(o_controller.getRightBumperButton()){
     // m_ShooterAnglePID.setReference(traplocation, SparkMax.ControlType.kPosition);
    }
    else if(o_controller.getBButton()){
      //m_ShooterAnglePID.setReference(traplocation-7, SparkMax.ControlType.kPosition);
    }
    else {
    //  m_ShooterAnglePID.setReference(0, SparkMax.ControlType.kPosition);
    }

    if(m_controller.getAButton()){
      m_gyro.reset();
    }

    if(m_controller.getRightBumperButton()){
      maxVel = 1.5;
    }
    else{
      maxVel = 4;
    }
//60 degrees

  }
  
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {

    System.out.println(m_BackLeftTurnEncoder.getPosition());
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}


}

import frc.robot.RobotContainer;
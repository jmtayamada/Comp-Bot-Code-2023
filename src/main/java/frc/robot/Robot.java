// ||  |\      /||  |:==\\    //==\\   ||===\\   ====||====      /\      |\   ||  ====||====
// ||  ||\    //||  |    ||  //    \\  ||    ||      ||         //\\     ||\  ||      ||    
// ||  ||\\  // ||  |:==//   ||    ||  ||\\=//       ||        //==\\    ||\\ ||      ||    
// ||  || \\//  ||  ||       \\    //  || \\         ||       //    \\   || \\||      ||    
// ||  ||  \/   ||  ||        \\==//   ||  \\        ||      //      \\  ||  \||      ||    

// Please keep this organized, put objects in the right sections
// use c_name for constants, and m_name for objects, if your object uses a port or string, 
// or something else in the create (ex: new WPI_TalonSRX(port)) use a constant for the port value

// for school wifi github changes, run command in terminal
// shutdown: tailscale down
// setup: tailscale up --exit-node 100.109.252.124

/* to do list
* test drive code
* pneumatics ?? of the intake
* manual arm control
* arm PID
* autonomous using limelight and AprilTags
*/

// possible mechanics
// gyroscope: import com.ctre.phoenix.sensors.WPI_PigeonIMU;
// encoders: import edu.wpi.first.wpilibj.DigitalInput;
// limelight?
// servo: import edu.wpi.first.wpilibj.Servo;

// 1 HD motor
// 4 cim motors
// 1 mini cim motor
// 2 7-inch cylinder; pneumatics
// 1 5-inch cylinder; pneumatics
// 2 double solenoid

//intake 2
//arm 1
//back left 5
//extension arm 4
//back right 3
//front left 0
//front right 6
//pdp 31
//pcm 0

/* driverstick
* joystick: driving
* button 2: slow drivetrain
* button 7: reverse drivetrain?
*/

/* helperstick
* button 1 (A): intake wheels reverse
* button 2 (B): intake cube
* button 3 (X): arm grab
* button 4 (Y): intake cone
* right axis: arm extend
* left axis: arm lift
* button 5 (left button): intake in/out
* arm/arm extension could use the joystick (manual), but aim for presets right now
*/

// code for intake motors may need to be changed to sparkmax

// Motion Magic ex code

package frc.robot;

import java.text.BreakIterator;
import java.time.LocalDateTime;
import java.util.Date;
import java.io.FileWriter;
import java.io.BufferedWriter;
import java.io.IOException;
import java.io.File;
import java.time.format.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.AHRS.SerialDataType;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.EncoderPositionSubsystem;
import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.MjpegServer;

// notes
// 19.5 inches -apriltag height for scoring platform
// navigate .72 inches forward from apriltag, then rotate to apriltag

public class Robot extends TimedRobot {
  // motor troubleshooting
  private final String path = "/home/lvuser/deploy/log.txt";
  public Timer m_electricalTimer = new Timer();
  // PID controller
  // DO NOT EDIT THE PID VALUES OR DELETE THIS COMMENT
  double ALkf = 0.0;
  double ALkp = 0.0055;
  double ALki = 0.0;
  double ALkd = 0.0;
  private final PIDController P_ArmLiftController = new PIDController(ALkp, ALki, ALkd);
  double v_armLiftPID;
  final double GkP = 0.04;
  final double GkI = 0.0;
  final double GkD = 0.001;
  private final PIDController P_GyroController = new PIDController(GkP, GkI, GkP);
  final double AEkp = 0.0;
  final double AEki = 0.0;
  final double AEkd = 0.0;
  private final PIDController P_ArmExtendEncoder = new PIDController(AEkd, AEkd, AEkd);
  // basic constants
  final int c_DriverStickPort = 1; // port for driver stick
  final int c_HelperStickPort = 0; // port for helper stick
  final int c_LeftMotorPort = 0; // port for left motor
  final int c_LeftMotor2Port = 5; // port for left motor2
  final int c_RightMotorPort = 6; // port for right motor
  final int c_RightMotor2Port = 3; // port for right motor2
  final int c_IntakeMotorPort = 2;
  final int c_ArmExtensionMotorPort = 4;
  final int c_ArmMotorPort = 1;
  final double c_IntakeSpeedCone = .8;
  final double c_IntakeSpeedCube = .5;
  final double c_ArmExtendSpeed = 1;
  final int c_PortArmGrab = 2; // port for Arm grabber
  final int c_PortArmGrab2 = 3; // port for Arm grabber 2
  final int c_portIntakeout = 0; // port for intake out
  final int c_portintakeout2 = 1; // port for intake out 2
  final double c_armTo90degrees = -0.15;
  final int c_tagTarget = 1; // 1,2,3 6,7,8
  boolean v_armclosed = false;
  boolean v_intakeout = false;
  double v_armpositiontomove = 0;
  double v_armExtensionToMove = 0;
  double v_angle;
  double v_armencoder;
  double v_GyroPitch;
  boolean v_DriveToSeesaw;
  double v_PowerToWheelsAutonomousSeesaw;
  boolean v_AutoScore;
  // basic components
  private final WPI_TalonSRX m_leftMotor = new WPI_TalonSRX(c_LeftMotorPort);
  private final WPI_TalonSRX m_leftMotor2 = new WPI_TalonSRX(c_LeftMotor2Port);
  private final WPI_TalonSRX m_rightMotor = new WPI_TalonSRX(c_RightMotorPort);
  private final WPI_TalonSRX m_rightMotor2 = new WPI_TalonSRX(c_RightMotor2Port);
  private final WPI_TalonSRX m_intakeMotor = new WPI_TalonSRX(c_IntakeMotorPort);
  private final WPI_TalonSRX m_armExtension = new WPI_TalonSRX(c_ArmExtensionMotorPort);
  private final WPI_TalonSRX m_armMotor = new WPI_TalonSRX(c_ArmMotorPort);
  private final DifferentialDrive m_motors1 = new DifferentialDrive(m_leftMotor, m_rightMotor);
  private final DifferentialDrive m_motors2 = new DifferentialDrive(m_leftMotor2, m_rightMotor2);
  private final Encoder m_ArmExtendEncoder = new Encoder(0, 1);
  private final AnalogGyro m_basicGyro = new AnalogGyro(0);
  private final DoubleSolenoid m_armgrab = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, c_PortArmGrab,
      c_PortArmGrab2);
  private final DoubleSolenoid m_intakeout = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, c_portIntakeout,
      c_portintakeout2);
  private final Timer m_gyrotimer = new Timer();
  private final DigitalInput m_armLimitSwitch = new DigitalInput(2);

  // private final DoubleSolenoid s_intake = new DoubleSolenoid(c_DriverStickPort,
  // null, c_ArmMotorPort, c_ArmExtensionMotorPort);
  // private final DoubleSolenoid s_squeezer = new
  // DoubleSolenoid(c_DriverStickPort, null, c_ArmMotorPort,
  // c_ArmExtensionMotorPort);
  // sensors
  Timer m_timer = new Timer();
  AHRS m_gyro = new AHRS();
  // controllers
  private final Joystick m_driverStick = new Joystick(c_DriverStickPort);
  private final Joystick m_helperStick = new Joystick(c_HelperStickPort);
  final int b_driveSlow = 2;
  final int b_driveReverse = 7;
  final int b_armlift = 1; // axis helperstick
  final int b_armextend = 5; // axis helperstick
  final int b_intakerunbackwards = 1; // button helperstick
  final int b_intakerunCube = 2; // button helperstick
  final int b_armgrab = 3; // button helperstick
  final int b_intakerunCone = 4; // button helperstick
  final int b_intakeout = 5; // button helperstick
  final int b_kpUp = 11; // button driverstick
  final int b_kpDown = 12;

  double steeringSpeed = 1;
  double driveSpeed = 1;
  boolean intakeStatus = false;
  // limelight
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-rock"); // this name may be different
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  NetworkTableEntry tv = table.getEntry("tv");
  // JSON
  NetworkTableEntry json = table.getEntry("json");
  ObjectMapper lightMapper = new ObjectMapper();
  // autonomous values
  double a_turnValue;
  double a_driveValue;
  boolean a_scored = false;
  // MotionMagic values
  int kMeasuredPosHorizontal = 2595; //Placeholder
  double kTicksPerDegree = 4096 / 360; // 360 is placeholder
  // other
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  double v_distancefromtarget;
  double limelightheight = 0;
  double apriltagheight = 0;
  double limelightangle = 0;

  // test values, may delet9e
  double j_powerToArm = 0;

  public void writeFile(String reason, Object value) {
    FileWriter writer;
    try {
      File file = new File(path);
      if (!file.exists()) {
        file.createNewFile();
      }
      writer = new FileWriter(file, true);
    } catch (IOException e) {
      System.out.println("Failed to find file");
      e.printStackTrace();
      return;
    }
    BufferedWriter bWriter = new BufferedWriter(writer);
    try {
      bWriter.write((float) m_electricalTimer.get() + " " + reason + value.toString() + "\n");
      bWriter.close();
      writer.close();
    } catch (IOException e) {
      e.printStackTrace();
    }

  }

  @Override
  public void robotInit() {
    m_electricalTimer.start();
    DateTimeFormatter dtf = DateTimeFormatter.ofPattern("yyyy/MM/dd HH:mm:ss");
    LocalDateTime now = LocalDateTime.now();
    System.out.println(dtf.format(now));

    FileWriter writer;
    try {
      File file = new File(path);
      if (!file.exists()) {
        file.createNewFile();
      }
      writer = new FileWriter(file, true);
    } catch (IOException e) {
      System.out.println("Failed to find file");
      e.printStackTrace();
      return;
    }
    BufferedWriter bWriter = new BufferedWriter(writer);
    try {
      bWriter.write("Initializing 6658 Robot Code at " + dtf.format(now) + "\n");
      bWriter.close();
      writer.close();
    } catch (IOException e) {
      e.printStackTrace();
    }

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    EncoderPositionSubsystem.m_armEncoderMotor = m_armMotor;
    // EncoderPositionSubsystem.addEncoder(10, m_armMotor, 1);

    m_robotContainer = new RobotContainer();
    m_leftMotor2.setInverted(true);
    m_leftMotor.setInverted(true);
    m_gyro = new AHRS(SerialPort.Port.kMXP, SerialDataType.kProcessedData, (byte) 50);
    m_gyro.calibrate();
    m_ArmExtendEncoder.setDistancePerPulse(3);
    // HttpCamera httpCamera = new HttpCamera("LimelightCamera", );
    CameraServer.startAutomaticCapture(0);
    CvSink cvSink = CameraServer.getVideo();
    CvSource outputStream = CameraServer.putVideo("cameraStream", 1280, 960);

    m_armExtension.setNeutralMode(NeutralMode.Coast);
  }

  // -61.57
  @Override
  public void robotPeriodic() {
    SmartDashboard.putBoolean("limit", m_armLimitSwitch.get());
    v_GyroPitch = m_gyro.getPitch() + 60.27;
    SmartDashboard.putNumber("Gyro Pitch", v_GyroPitch);
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    // CommandScheduler.getInstance().run();
    // double totalheightdifference = Math.abs(apriltagheight-limelightheight);
    // double totalangle = ty.getDouble(0) + limelightangle;
    // v_distancefromtarget = totalheightdifference/(Math.tan(totalangle));
    v_angle = m_gyro.getYaw();
    writeFile("Right Motor: ", m_rightMotor.getBusVoltage());
    writeFile("Right Motor2: ", m_rightMotor2.getBusVoltage());
    writeFile("Left Motor: ", m_leftMotor.getBusVoltage());
    writeFile("Left Motor2: ", m_leftMotor2.getBusVoltage());
    writeFile("Arm Motor: ", m_armMotor.getBusVoltage());
    writeFile("Arm Extension Motor: ", m_armExtension.getBusVoltage());
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  double setpointAngle = 90.0;
  double setpointDistance = 20;

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    v_intakeout = false;
    m_intakeout.set(Value.kReverse);
    v_armclosed = false;
    m_armgrab.set(Value.kReverse);
    m_timer.reset();

    // Commented Code 1 [here]

    v_DriveToSeesaw = true;
    m_timer.start();
    v_armclosed = false;
    m_armgrab.set(Value.kReverse);
    P_ArmLiftController.setSetpoint(v_armpositiontomove);
    P_ArmLiftController.setTolerance(25);
    j_powerToArm = 0;
    m_gyrotimer.reset();
    m_gyrotimer.start();
    brakemode = false;
    m_ArmExtendEncoder.reset();
    P_GyroController.setTolerance(1);
    P_GyroController.setSetpoint(0);
    v_armpositiontomove = 2660;
  }

  int targetTag = 0;
  boolean moveout = false;
  boolean turn = false;

  @Override
  public void autonomousPeriodic() {
    SmartDashboard.putNumber("armencoder", v_armencoder);

    v_armencoder = MathUtil.clamp(m_armMotor.getSelectedSensorPosition(), 0, 4000);

    v_armpositiontomove = MathUtil.clamp(v_armpositiontomove, 8, 2661.919); // 7739
    v_armLiftPID = MathUtil.clamp(P_ArmLiftController.calculate(v_armencoder, v_armpositiontomove), -.5, 1);

    m_armMotor.set(ControlMode.PercentOutput, -v_armLiftPID);

    if (m_helperStick.getRawAxis(b_armextend) > 0.1 && m_ArmExtendEncoder.getDistance() >= -30) {
      m_armExtension.set(ControlMode.PercentOutput, c_ArmExtendSpeed);
    } else if (m_helperStick.getRawAxis(b_armextend) < -0.1 && m_ArmExtendEncoder.getDistance() <= 2100) {
      m_armExtension.set(ControlMode.PercentOutput, -c_ArmExtendSpeed);
    } else {
      m_armExtension.set(ControlMode.PercentOutput, 0);
    }

    m_armgrab.set(Value.kReverse);

    // Commented Code 2 [here]

    // if (v_DriveToSeesaw) {
    //   System.out.println("DrivingToSeesaw");
    //   if (v_GyroPitch > 8 || v_GyroPitch < -8) {
    //     v_DriveToSeesaw = false;
    //   }
    //   m_motors1.arcadeDrive(0.75, 0);
    //   m_motors2.arcadeDrive(0.75, 0);
    // } else {
    //   if (v_GyroPitch > 11) {
    //     m_motors1.arcadeDrive(.6, 0);
    //     m_motors2.arcadeDrive(.6, 0);
    //   } else if (v_GyroPitch < -11) {
    //     m_motors1.arcadeDrive(-.6, 0);
    //     m_motors2.arcadeDrive(-.6, 0);
    //   } else {
    //     m_motors1.arcadeDrive(0, 0);
    //     m_motors2.arcadeDrive(0, 0);
    //     // System.out.println("AtSeesaw");
    //     // v_PowerToWheelsAutonomousSeesaw =
    //     // MathUtil.clamp(P_GyroController.calculate(v_GyroPitch), -1, 1);
    //     // m_motors1.arcadeDrive(-v_PowerToWheelsAutonomousSeesaw, 0);
    //     // m_motors2.arcadeDrive(-v_PowerToWheelsAutonomousSeesaw, 0);
    //     // System.out.println(v_PowerToWheelsAutonomousSeesaw);
    //     // SmartDashboard.putNumber("powerToWheels", v_PowerToWheelsAutonomousSeesaw);
    //   }
    // }
    // }
    // }
  }

  int encoderPosition = 0;
  // int poll_ms = 1;

  /*
   * Thread t = new Thread(() -> {
   * double pollVelocity = m_armMotor.getSelectedSensorVelocity(0)/10;
   * try {Thread.sleep(poll_ms);} catch (Exception e) {return;}
   * encoderPosition += pollVelocity;
   * });
   */
  boolean reverseDrive = false;
  boolean driveslow = false;
  boolean brakemode = false;

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    brakemode = true;
    m_rightMotor2.setNeutralMode(NeutralMode.Brake);
    m_leftMotor.setNeutralMode(NeutralMode.Brake);
    m_leftMotor2.setNeutralMode(NeutralMode.Brake);
    m_rightMotor.setNeutralMode(NeutralMode.Brake);
    m_armExtension.setNeutralMode(NeutralMode.Brake);

    // m_armMotor.configMotionCruiseVelocity(/*cruise velocity*/, 5);
    // m_armMotor.configMotionAcceleration(/*half of cruise velocity*/, 5);


    m_armMotor.config_kP(0, ALkp, 0);
    m_armMotor.config_kI(0, ALki, 0);
    m_armMotor.config_kD(0, ALkd, 0);
    m_armMotor.config_kF(0, ALkf, 0);

    v_intakeout = false;
    m_intakeout.set(Value.kReverse);
    m_intakeMotor.set(ControlMode.PercentOutput, 0);
    v_armclosed = true;
    m_armgrab.set(Value.kReverse);
    P_ArmLiftController.setSetpoint(v_armpositiontomove);
    P_ArmLiftController.setTolerance(25);
    j_powerToArm = 0;
    m_gyrotimer.reset();
    m_gyrotimer.start();
    m_ArmExtendEncoder.reset();
    v_armencoder = 0;
    v_armpositiontomove = 0;
    m_armMotor.setSelectedSensorPosition(0);
  }

  @Override
  public void teleopPeriodic() {

    m_gyrotimer.reset();

    v_armencoder = MathUtil.clamp(m_armMotor.getSelectedSensorPosition(), 0, 4000);
    double turningAmount = 0;

    // if statement doesn't work
    if (m_driverStick.getRawAxis(3) == -1) {
      turningAmount = (m_driverStick.getZ()) + m_driverStick.getX() * steeringSpeed;
      SmartDashboard.putBoolean("Oliver Control?", false);
    } else {
      turningAmount = (m_driverStick.getX()) * steeringSpeed;
      SmartDashboard.putBoolean("Oliver Control?", true);
    }
    MathUtil.clamp(turningAmount, 0, 1);

    if (m_driverStick.getRawButtonPressed(b_driveReverse)) {
      reverseDrive = !reverseDrive;
    }

    if (m_driverStick.getRawButton(b_driveSlow)) {
      driveSpeed = .4;
    } else {
      driveSpeed = 1;
    }

    // Increments or decrements kP while robot is running to speed up PID tuning

    if (m_driverStick.getRawButtonPressed(b_kpUp)) {
      ALkp += 0.001;
    } else if (m_driverStick.getRawButtonPressed(b_kpDown)) {
      ALkp -= 0.001;
    }

    if (!reverseDrive) {
      m_motors1.arcadeDrive(m_driverStick.getY() * driveSpeed, turningAmount * driveSpeed);
      m_motors2.arcadeDrive(m_driverStick.getY() * driveSpeed, turningAmount * driveSpeed);
    } else {
      m_motors1.arcadeDrive(-m_driverStick.getY() * driveSpeed, turningAmount * driveSpeed);
      m_motors2.arcadeDrive(-m_driverStick.getY() * driveSpeed, turningAmount * driveSpeed);
    }

    if (m_driverStick.getRawButton(5)) {
      if (!brakemode) {
        m_rightMotor.setNeutralMode(NeutralMode.Brake);
        m_rightMotor2.setNeutralMode(NeutralMode.Brake);
        m_leftMotor.setNeutralMode(NeutralMode.Brake);
        m_leftMotor2.setNeutralMode(NeutralMode.Brake);
        brakemode = true;
      } else {
        m_rightMotor.setNeutralMode(NeutralMode.Coast);
        m_rightMotor2.setNeutralMode(NeutralMode.Coast);
        m_leftMotor.setNeutralMode(NeutralMode.Coast);
        m_leftMotor2.setNeutralMode(NeutralMode.Coast);
        brakemode = false;
      }
    }
    SmartDashboard.putBoolean("brakemode", brakemode);

    // old arm move code

    // if (m_helperStick.getRawAxis(b_armlift) < 0) {
    // m_armMotor.set(ControlMode.PercentOutput,
    // (m_helperStick.getRawAxis(b_armlift)/2));
    // } else {
    // m_armMotor.set(ControlMode.PercentOutput,
    // (m_helperStick.getRawAxis(b_armlift)/8));
    // }

    // 1875
    SmartDashboard.putNumber("armencoder", v_armencoder);
    SmartDashboard.putNumber("armextend encoder", m_ArmExtendEncoder.getDistance());
    if (m_helperStick.getRawAxis(b_armextend) > 0.1 && m_ArmExtendEncoder.getDistance() >= -30) {
      m_armExtension.set(ControlMode.PercentOutput, c_ArmExtendSpeed);
    } else if (m_helperStick.getRawAxis(b_armextend) < -0.1 && m_ArmExtendEncoder.getDistance() <= 2100) {
      m_armExtension.set(ControlMode.PercentOutput, -c_ArmExtendSpeed);
    } else {
      m_armExtension.set(ControlMode.PercentOutput, 0);
    }

    if (m_helperStick.getRawButtonPressed(b_armgrab)) {
      v_armclosed = !v_armclosed;

      if (v_armclosed) {
        m_armgrab.set(Value.kForward);
      } else {
        m_armgrab.set(Value.kReverse);
      }
    }

    // MotionMagic Code
    // int currentPos = m_armMotor.getSelectedSensorPosition(0);
    // double degrees = (currentPos - kMeasuredPosHorizontal) / kTicksPerDegree;
    // double radians = java.land.Math.toRadians(degrees);
    // double cosineScalar = java.land.Math.cos(radians);
    // double maxGravityFF = 0.12;
    // m_armMotor.set(ControlMode.MotionMagic, targetPos, DemandType.ArbitraryFeedForward, maxGravityFF * cosineScalar);

    if (m_helperStick.getRawAxis(b_armlift) < -0.1 || m_helperStick.getRawAxis(b_armlift) > 0.1) {
      v_armpositiontomove -= m_helperStick.getRawAxis(b_armlift) * 30;
    }

    v_armpositiontomove = MathUtil.clamp(v_armpositiontomove, 8, 2661.919); // 7739
    
    v_armLiftPID = MathUtil.clamp(P_ArmLiftController.calculate(v_armencoder, v_armpositiontomove), -.5, 1);

    SmartDashboard.putNumber("Target Angle Arm", v_armpositiontomove);
    SmartDashboard.putNumber("ActualPower", v_armLiftPID);
    SmartDashboard.putNumber("Arm Rotation", v_armencoder);

    m_armMotor.set(ControlMode.PercentOutput, -v_armLiftPID);
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }
}

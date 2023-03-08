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


  package frc.robot;

  import java.text.BreakIterator;

  import java.io.FileWriter;
  import java.io.PrintWriter;
  import java.io.IOException;

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
import edu.wpi.first.wpilibj.Filesystem;
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

  // notes
  // 19.5 inches -apriltag height for scoring platform
  // navigate .72 inches forward from apriltag, then rotate to apriltag
  

  public class Robot extends TimedRobot {
    // motor troubleshooting
    private final String path = "/home/lvuser/deploy/log.txt";
    public Timer m_electricalTimer = new Timer();
    // PID controller
    final double ALkp = 0.0008;
    final double ALki = 0.000;
    final double ALkd = 0.0001;
    private final PIDController P_ArmLiftController = new PIDController(ALkp, ALki, ALkd);
    double v_armLiftPID;
    final double GkP = 0.09090909;
    final double GkI = 0.0;
    final double GkD = 0.00001;
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
    final double c_ArmExtendSpeed = .8;
    final int c_PortArmGrab = 2; // port for Arm grabber
    final int c_PortArmGrab2 = 3; // port for Arm grabber 2
    final int c_portIntakeout = 0; // port for intake out 
    final int c_portintakeout2 = 1; // port for intake out 2
    final double c_armTo90degrees = -0.15;
    final int c_tagTarget = 1;  // 1,2,3 6,7,8
    boolean v_armclosed = false;
    boolean v_intakeout = false;
    double v_armpositiontomove = 0;
    double v_armExtensionToMove = 0;
    double v_angle;
    double v_armencoder;
    double v_GyroPitch;
    boolean v_DriveToSeesaw;
    double v_PowerToWheelsAutonomousSeesaw;
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
    private final DoubleSolenoid m_armgrab = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, c_PortArmGrab, c_PortArmGrab2);
    private final DoubleSolenoid m_intakeout = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, c_portIntakeout, c_portintakeout2);
    private final Timer m_gyrotimer = new Timer();
    private final DigitalInput m_AutnomousSwitch = new DigitalInput(2);
    //  private final DoubleSolenoid s_intake = new DoubleSolenoid(c_DriverStickPort, null, c_ArmMotorPort, c_ArmExtensionMotorPort);
    //  private final DoubleSolenoid s_squeezer = new DoubleSolenoid(c_DriverStickPort, null, c_ArmMotorPort, c_ArmExtensionMotorPort);
    // sensors
    Timer m_timer = new Timer();
    AHRS m_gyro = new AHRS();
    // controllers
    private final Joystick m_driverStick = new Joystick(c_DriverStickPort);
    private final Joystick m_helperStick = new Joystick(c_HelperStickPort);
    final int b_driveSlow = 2;
    final int b_driveReverse = 7;
    final int b_armlift = 1;                 //axis      helperstick
    final int b_armextend = 5;               //axis      helperstick
    final int b_intakerunbackwards = 1;      //button    helperstick
    final int b_intakerunCube = 2;           //button    helperstick
    final int b_armgrab = 3;                 //button    helperstick
    final int b_intakerunCone = 4;           //button    helperstick
    final int b_intakeout = 5;               //button    helperstick

    double steeringSpeed = 0.9;
    double driveSpeed = 0.9;
    boolean intakeStatus = false;
    // limelight
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-rock");     // this name may be different
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
      try { writer = new FileWriter(path, true); } catch (Exception e) {System.out.println("Failed to find file"); e.printStackTrace();return;}
      PrintWriter printer = new PrintWriter(writer);
      printer.println((float)m_electricalTimer.get()+ " " + reason + ": " + value.toString());

    }

    @Override
    public void robotInit() {
      m_electricalTimer.start();
      // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
      // autonomous chooser on the dashboard.
      EncoderPositionSubsystem.m_armEncoderMotor = m_armMotor;
      // EncoderPositionSubsystem.addEncoder(10, m_armMotor, 1);
      
      m_robotContainer = new RobotContainer();
      m_leftMotor2.setInverted(true);
      m_leftMotor.setInverted(true);
      m_gyro = new AHRS(SerialPort.Port.kMXP, SerialDataType.kProcessedData, (byte)50); 
      m_gyro.calibrate();
      m_ArmExtendEncoder.setDistancePerPulse(3);
      CameraServer.startAutomaticCapture(0);
      m_armExtension.setNeutralMode(NeutralMode.Brake);
    }
    //-61.57
    @Override
    public void robotPeriodic() {
      v_GyroPitch = m_gyro.getPitch() + 61.47;
      SmartDashboard.putNumber("Gyro Pitch", v_GyroPitch);
      // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
      // commands, running already-scheduled commands, removing finished or interrupted commands,
      // and running subsystem periodic() methods.  This must be called from the robot's periodic
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
    public void disabledInit() {}
  
    @Override
    public void disabledPeriodic() {}
    
    double setpointAngle = 90.0;
    double setpointDistance = 20;

    /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
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
      // P_AngleController.enableContinuousInput(-180, 180);
      // P_AngleController.setTolerance(5);
      // P_DistanceController.setTolerance(2);
      // P_AngleController2.setTolerance(5);
      // P_AngleController2.enableContinuousInput(-180, 180);

      v_intakeout = true;
      m_intakeout.set(Value.kForward);
      v_armclosed = true;
      m_armgrab.set(Value.kForward);
      m_timer.start();
      moveout = false;
      v_DriveToSeesaw = true;
      P_GyroController.setSetpoint(0);
    }
    int targetTag = 0;
    boolean moveout = false;
    boolean turn = false;
    @Override
    public void autonomousPeriodic() {

      // // AprilTag Manual
      // m_timer.start();
      // if (a_scored == false) {
      //   if (tv.getDouble(0) == 1) {
      //     a_turnValue = -tx.getDouble(0)/54;
      //     if (ty.getDouble(0) > 19.25) {
      //       a_driveValue = (ty.getDouble(0)-19.5)/5.35;
      //     } else if (ty.getDouble(0) < 18.75) {
      //       a_driveValue = -(9.25+(-(ty.getDouble(0)-9.25)))/18.5;
      //     } else {
      //       a_driveValue = 0;
      //       // arm code goes here
      //       a_scored = true;
      //     }
      //     if (a_driveValue > 1) {
      //       a_driveValue = 1;
      //     } else if(a_driveValue < -1) {
      //       a_driveValue = -1;
      //     } 
      //     m_motors1.arcadeDrive(a_driveValue, a_turnValue, false);
      //     m_motors2.arcadeDrive(a_driveValue, a_turnValue, false);
      //     // leftMotor.set(ControlMode.PercentOutput, a_driveValue-(tx.getDouble(0))/54);
      //     // leftMotor2.set(ControlMode.PercentOutput, a_driveValue-(tx.getDouble(0))/54);
      //     // rightMotor.set(ControlMode.PercentOutput, a_driveValue+(tx.getDouble(0))/54);
      //     // rightMotor2.set(ControlMode.PercentOutput, a_driveValue+(tx.getDouble(0))/54);
      //   } else {
      //     if (m_timer.get() < .5) {
      //       m_motors1.arcadeDrive(0, -.5);  //.5
      //       m_motors2.arcadeDrive(0, -.5);   //.5
      //     } 
      //     if (m_timer.get() < 1.5 && m_timer.get() >= .5) {
      //       m_motors1.arcadeDrive(0, 0);
      //       m_motors2.arcadeDrive(0, 0);
      //     }
      //     if (m_timer.get() >= 1.5) {
      //       m_timer.reset();
      //     }
      //   }
      // } else {
      // }

      // int tagID = 0;
      // // AprilTag JSON
      // try { 
      //   JsonNode rootNode = lightMapper.readTree(json.getString(""));
      //   JsonNode idNode = rootNode.at("/Results/Fiducial/0/fID");//.path("Fiducial[0]").path("fID");
      //   tagID = idNode.asInt();
      // } catch (Exception e) {
      // }
      
      // if (tv.getDouble(0) == 1 && tagID == c_tagTarget) {
       
        // P_AngleController.setSetpoint(tx.getDouble(0)+v_angle);
        // while (!P_AngleController.atSetpoint() || !P_DistanceController.atSetpoint()) {
        // m_motors1.arcadeDrive(P_DistanceController.calculate(v_distancefromtarget, setpointDistance), P_AngleController.calculate(v_angle, setpointAngle));
        //   m_motors2.arcadeDrive(P_DistanceController.calculate(v_distancefromtarget, setpointDistance), P_AngleController.calculate(v_angle, setpointAngle));
        //   if (P_AngleController.atSetpoint() && P_DistanceController.atSetpoint()) {
        //     m_leftMotor.set(ControlMode.PercentOutput, 0);
        //     m_leftMotor2.set(ControlMode.PercentOutput, 0);
        //     m_rightMotor.set(ControlMode.PercentOutput, 0);
        //     m_rightMotor2.set(ControlMode.PercentOutput, 0);
        //     break;
        //   }
        // }

        // test code, limelight only
      //   P_DistanceController.setSetpoint(20);
      //   P_AngleController2.setSetpoint(0);
      //   while (!P_AngleController2.atSetpoint() || !P_DistanceController.atSetpoint()) {
      //     m_motors1.arcadeDrive(P_DistanceController.calculate(v_distancefromtarget), P_AngleController2.calculate(tx.getDouble(0)));
      //     m_motors2.arcadeDrive(P_DistanceController.calculate(v_distancefromtarget), P_AngleController2.calculate(tx.getDouble(0)));
      //     if (P_AngleController2.atSetpoint() && P_DistanceController.atSetpoint()) {
      //       m_leftMotor.set(ControlMode.PercentOutput, 0);
      //       m_leftMotor2.set(ControlMode.PercentOutput, 0);
      //       m_rightMotor.set(ControlMode.PercentOutput, 0);
      //       m_rightMotor2.set(ControlMode.PercentOutput, 0);
      //       break;
      //     }
      //   }
      // } else {
      //   if (m_timer.get() < .5) {
      //     m_motors1.arcadeDrive(0, -.5);  //.5
      //     m_motors2.arcadeDrive(0, -.5);   //.5
      //   } 
      //   if (m_timer.get() < 1.5 && m_timer.get() >= .5) {
      //     m_motors1.arcadeDrive(0, 0);
      //     m_motors2.arcadeDrive(0, 0);
      //   }
      //   if (m_timer.get() >= 1.5) {
      //     m_timer.reset();
      //   }
      // }

      // autonomous alternative
      // m_armMotor.set(ControlMode.PercentOutput, c_ArmMotorSpeed);
      // m_armExtension.set(ControlMode.PercentOutput, c_ArmExtendSpeed);
      // m_armgrab.set(Value.kReverse);
      // m_armExtension.set(ControlMode.PercentOutput, -c_ArmExtendSpeed);
      // m_armgrab.set(Value.kForward);
      // m_armExtension.set(ControlMode.PercentOutput, 0);
      // // m_armMotor.set(ControlMode.PercentOutput, -c_ArmMotorSpeed);




      // if (m_armMotor.getSelectedSensorPosition(0) != 5) {
      //   m_armMotor.set(ControlMode.PercentOutput, c_armTo90degrees);
      // } else if (m_ArmExtendEncoder.getDistance() != 5) {
        
      // } else {
      //   v_armclosed = false;
      //   m_armgrab.set(Value.kReverse);
      //   m_timer.start();
      //   turn = true;
      // }
      // if (m_timer.get() > 1 && turn) {
      //   m_motors1.arcadeDrive(0, MathUtil.clamp(P_GyroController.calculate(m_basicGyro.getAngle(), 180.0), -1, 1));
      //   m_motors2.arcadeDrive(0, MathUtil.clamp(P_GyroController.calculate(m_basicGyro.getAngle(), 180.0), -1, 1));
      //   if (P_GyroController.atSetpoint()) {
      //     moveout = true;
      //     turn = false;
      //     m_timer.reset();
      //     m_intakeMotor.set(ControlMode.PercentOutput, 1);
      //   }
      // }
      // if (moveout && m_timer.get() < 5) {
      //   m_motors1.arcadeDrive(1, 0);
      //   m_motors2.arcadeDrive(1, 0);
      // }
        if (m_AutnomousSwitch.get()) {

        } else {
          if (v_DriveToSeesaw) {
            if (v_GyroPitch > 2 || v_GyroPitch < -2) {
              v_DriveToSeesaw = false;
            }
            m_motors1.arcadeDrive(.5, 0);
            m_motors2.arcadeDrive(.5, 0);
          } else {
            v_PowerToWheelsAutonomousSeesaw = P_GyroController.calculate(v_GyroPitch)/2;
            m_motors1.arcadeDrive(v_PowerToWheelsAutonomousSeesaw, 0);
            m_motors2.arcadeDrive(v_PowerToWheelsAutonomousSeesaw, 0);
          }
        }
    }
    int encoderPosition = 0;
    //int poll_ms = 1;
     
    /*Thread t = new Thread(() -> {
      double pollVelocity = m_armMotor.getSelectedSensorVelocity(0)/10;
      try {Thread.sleep(poll_ms);} catch (Exception e) {return;}
      encoderPosition += pollVelocity;
      }); */
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

      v_intakeout = false;
      m_intakeout.set(Value.kReverse);
      m_intakeMotor.set(ControlMode.PercentOutput, 0);
      v_armclosed = false;
      m_armgrab.set(Value.kReverse);
      P_ArmLiftController.setSetpoint(v_armpositiontomove); 
      P_ArmLiftController.setTolerance(25);
      j_powerToArm = 0;
      m_gyrotimer.reset();
      m_gyrotimer.start();
      brakemode = false;
      m_ArmExtendEncoder.reset();
      v_armencoder = 0;
      v_armpositiontomove = 0;
    }

    boolean reverseDrive = false;
    boolean driveslow = false;
    boolean brakemode = false;

    @Override
    public void teleopPeriodic() {

      m_gyrotimer.reset();
      
      v_armencoder = MathUtil.clamp(m_armMotor.getSelectedSensorPosition(), 0, 3000);

      double turningAmount = (m_driverStick.getZ() * .7) + m_driverStick.getX() * steeringSpeed;
      // double turningAmount = (m_driverStick.getX()) * steeringSpeed;
      MathUtil.clamp(turningAmount, 0, 1);

      if(m_driverStick.getRawButtonPressed(b_driveReverse)){
        reverseDrive = !reverseDrive;
      }

      if (m_driverStick.getRawButton(b_driveSlow)) {
        driveSpeed = .4;
      } else {
        driveSpeed = .9;
      }

      if (!reverseDrive){
        m_motors1.arcadeDrive(m_driverStick.getY() * driveSpeed, turningAmount * driveSpeed);
        m_motors2.arcadeDrive(m_driverStick.getY() * driveSpeed, turningAmount * driveSpeed);
      } else{
        m_motors1.arcadeDrive(-m_driverStick.getY() * driveSpeed, -turningAmount * driveSpeed);
        m_motors2.arcadeDrive(-m_driverStick.getY() * driveSpeed, -turningAmount * driveSpeed);
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
      //   m_armMotor.set(ControlMode.PercentOutput, (m_helperStick.getRawAxis(b_armlift)/2));
      // } else {
      //   m_armMotor.set(ControlMode.PercentOutput, (m_helperStick.getRawAxis(b_armlift)/8));
      // }

      //1875
        SmartDashboard.putNumber("armencoder", v_armencoder);
        SmartDashboard.putNumber("armextend encoder", m_ArmExtendEncoder.getDistance());
      if (m_helperStick.getRawAxis(b_armextend) > 0.1) { //&& m_ArmExtendEncoder.getDistance() <= 1875) {
        m_armExtension.set(ControlMode.PercentOutput, c_ArmExtendSpeed);
      } else if (m_helperStick.getRawAxis(b_armextend) < -0.1) { // && m_ArmExtendEncoder.getDistance() >= -34) {
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
    
      if (m_helperStick.getRawAxis(b_armlift) < -0.1 || m_helperStick.getRawAxis(b_armlift) > 0.1) {
        v_armpositiontomove -= m_helperStick.getRawAxis(b_armlift) * 5;
      }

      v_armpositiontomove = MathUtil.clamp(v_armpositiontomove, 0, 2477);
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
  
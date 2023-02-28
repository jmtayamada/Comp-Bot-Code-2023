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

  import edu.wpi.first.wpilibj.TimedRobot;
  import edu.wpi.first.wpilibj2.command.Command;
  import edu.wpi.first.wpilibj2.command.CommandScheduler;
  
  import edu.wpi.first.wpilibj.Solenoid;
  import edu.wpi.first.wpilibj.PneumaticsModuleType;
  import edu.wpi.first.wpilibj.DoubleSolenoid;
  
  import edu.wpi.first.wpilibj.Joystick;
  import edu.wpi.first.wpilibj.TimedRobot;
  import edu.wpi.first.wpilibj.drive.DifferentialDrive;
  import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
  import com.ctre.phoenix.motorcontrol.ControlMode;
  import com.kauailabs.navx.frc.AHRS;
  import edu.wpi.first.wpilibj.Timer;
  import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
  import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
  import edu.wpi.first.cameraserver.CameraServer;
  import edu.wpi.first.wpilibj.Encoder;
  import com.kauailabs.navx.frc.AHRS.SerialDataType;
  import edu.wpi.first.wpilibj.SerialPort;
  import edu.wpi.first.wpilibj.AnalogInput;
  import edu.wpi.first.networktables.NetworkTable;
  import edu.wpi.first.networktables.NetworkTableEntry;
  import edu.wpi.first.networktables.NetworkTableInstance;
  import com.ctre.phoenix.sensors.CANCoder;
  import com.ctre.phoenix.sensors.CANCoderConfiguration;
  import com.ctre.phoenix.sensors.SensorTimeBase;
  import edu.wpi.first.math.controller.PIDController;
  import java.util.*;

  // notes
  // 19.5 inches -apriltag height for scoring platform
  // navigate .72 inches forward from apriltag, then rotate to apriltag
  
  public class Robot extends TimedRobot {
    // basic constants
    final int c_DriverStickPort = 2; // port for driver stick
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
    final double c_ArmSpeedUp = 0.9;
    final double c_ArmSpeedDown = -0.2;
    final double c_ArmExtendSpeed = .5;
    final int c_PortArmGrab = 2; // port for Arm grabber
    final int c_PortArmGrab2 = 3; // port for Arm grabber 2
    final int c_portIntakeout = 0; // port for intake out 
    final int c_portintakeout2 = 1; // port for intake out 2
    boolean v_armclosed = false;
    boolean v_intakeout = false;
    double v_armposition = 0;
    final double kP = 0.0;
    final double kI = 0.0;
    final double kD = 0.0;
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
    private final Encoder m_armExtendEncoder = new Encoder(0, 1);
    private final Encoder m_armAngleEncoder = new Encoder(2, 3);
    private final DoubleSolenoid m_armgrab = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, c_PortArmGrab, c_PortArmGrab2);
    private final DoubleSolenoid m_intakeout = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, c_portIntakeout, c_portintakeout2);
    private final PIDController pid = new PIDController(kP, kI, kP);
    
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
  
    @Override
    public void robotInit() {
      // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
      // autonomous chooser on the dashboard.
      m_robotContainer = new RobotContainer();
      m_leftMotor2.setInverted(true);
      m_leftMotor.setInverted(true);
    }
  
    @Override
    public void robotPeriodic() {
      // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
      // commands, running already-scheduled commands, removing finished or interrupted commands,
      // and running subsystem periodic() methods.  This must be called from the robot's periodic
      // block in order for anything in the Command-based framework to work.
      CommandScheduler.getInstance().run();
      double totalheightdifference = Math.abs(apriltagheight-limelightheight);
      double totalangle = ty.getDouble(0) + limelightangle;
      v_distancefromtarget = totalheightdifference/(Math.tan(totalangle));
      SmartDashboard.putNumber("DistanceFromTag", v_distancefromtarget);
    }
  
    @Override
    public void disabledInit() {}
  
    @Override
    public void disabledPeriodic() {}
  
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
      double setpoint = 90.0;
      pid.setSetpoint(setpoint);
      pid.enableContinuousInput(-180, 180);
      pid.setTolerance(5);
    }
  
    @Override
    public void autonomousPeriodic() {
      m_timer.start();
      if (a_scored == false) {
        if (tv.getDouble(0) == 1) {
          a_turnValue = -tx.getDouble(0)/54;
          if (ty.getDouble(0) > 19.25) {
            a_driveValue = (ty.getDouble(0)-19.5)/5.35;
          } else if (ty.getDouble(0) < 18.75) {
            a_driveValue = -(9.25+(-(ty.getDouble(0)-9.25)))/18.5;
          } else {
            a_driveValue = 0;
            // arm code goes here
            a_scored = true;
          }
          if (a_driveValue > 1) {
            a_driveValue = 1;
          } else if(a_driveValue < -1) {
            a_driveValue = -1;
          } 
          SmartDashboard.putNumber("drive", a_driveValue);
          SmartDashboard.putNumber("turn", a_turnValue);
          System.out.println("turn: "+ a_turnValue);
          System.out.println("drive: "+ a_driveValue);
          m_motors1.arcadeDrive(a_driveValue, a_turnValue, false);
          m_motors2.arcadeDrive(a_driveValue, a_turnValue, false);
          // leftMotor.set(ControlMode.PercentOutput, a_driveValue-(tx.getDouble(0))/54);
          // leftMotor2.set(ControlMode.PercentOutput, a_driveValue-(tx.getDouble(0))/54);
          // rightMotor.set(ControlMode.PercentOutput, a_driveValue+(tx.getDouble(0))/54);
          // rightMotor2.set(ControlMode.PercentOutput, a_driveValue+(tx.getDouble(0))/54);
        } else {
          if (m_timer.get() < .5) {
            m_motors1.arcadeDrive(0, -.5);  //.5
            m_motors2.arcadeDrive(0, -.5);   //.5
          } 
          if (m_timer.get() < 1.5 && m_timer.get() >= .5) {
            m_motors1.arcadeDrive(0, 0);
            m_motors2.arcadeDrive(0, 0);
          }
          if (m_timer.get() >= 1.5) {
            m_timer.reset();
          }
        }
      } else {
  
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
      v_intakeout = false;
      m_intakeout.set(Value.kReverse);
      v_armclosed = false;
      m_armgrab.set(Value.kReverse);

    }

    boolean reverseDrive = false;
    boolean driveslow = false;

    @Override
    public void teleopPeriodic() {
      SmartDashboard.putNumber("driveSpeed", driveSpeed);
      
      //  double turningAmount = (m_driverStick.getZ() * 0.75) + m_driverStick.getX() * steeringSpeed;
      double turningAmount = (m_driverStick.getX()) * steeringSpeed;
      if (turningAmount > 1) {
        turningAmount = 1;
      }

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

      if (m_helperStick.getRawButton(b_intakerunCone)) {
        m_intakeMotor.set(ControlMode.PercentOutput, c_IntakeSpeedCone);
      } else if(m_helperStick.getRawButton(b_intakerunbackwards)){
        m_intakeMotor.set(ControlMode.PercentOutput, -c_IntakeSpeedCone);
      } else if(m_helperStick.getRawButton(b_intakerunCube)){
        m_intakeMotor.set(ControlMode.PercentOutput, c_IntakeSpeedCube);
      } else {
        m_intakeMotor.set(ControlMode.PercentOutput, 0);
      }

      //change arm motor speed based on joystick value
      //There's a better way to do this, but i'm too tired to think it up.
      if (m_helperStick.getRawAxis(b_armlift) < -0.5) {
        m_armMotor.set(ControlMode.PercentOutput, c_ArmSpeedUp);
      } else {
        m_armMotor.set(ControlMode.PercentOutput, 0);
      }

      if (m_helperStick.getRawButtonPressed(b_intakeout)) {
        v_intakeout = !v_intakeout;
        if (v_intakeout) {
          m_intakeout.set(Value.kForward);
        } else {
          m_intakeout.set(Value.kReverse);
        }
      }

      if (m_helperStick.getRawAxis(b_armextend) > 0.1 /* && m_armExtendEncoder.getDistance() < 50 && m_armAngleEncoder.getDistance() > 50 */) {
        m_armExtension.set(ControlMode.PercentOutput, c_ArmExtendSpeed);
      } else if (m_helperStick.getRawAxis(b_armextend) < -0.1 /* && m_armExtendEncoder.getDistance() > 0 */) {
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
    }

    @Override
    public void testInit() {
      // Cancels all running commands at the start of test mode.
      CommandScheduler.getInstance().cancelAll();
    }
  
    @Override
    public void testPeriodic() {}
  }
  
package frc.robot.subsystems;

import java.lang.Character.Subset;
import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

class MotorEncoder {
    public MotorEncoder(int pollrate, WPI_TalonSRX motor) 
    {
        
        poll_ms = pollrate;
        m_armEncoderMotor = motor;

    }
public int encoderPosition;
public WPI_TalonSRX m_armEncoderMotor;
public int poll_ms;

}

public class EncoderPositionSubsystem extends SubsystemBase{
    public static WPI_TalonSRX m_armEncoderMotor;
    public static double encoderPosition;
    private static ArrayList<MotorEncoder> encoders = new ArrayList<MotorEncoder>();
    final int poll_ms = 1;
    private boolean threadStarted = false;
    public EncoderPositionSubsystem() {}

    @Override
    public void periodic() {
      //System.out.println(m_armEncoderMotor.getSelectedSensorPosition(0));
       // System.out.println(encoderPosition);
        // for (int i = 0; i < encoders.size(); i++) {
        //     MotorEncoder current = encoders.get(i);
        //     double pollVelocity_forLoop = current.m_armEncoderMotor.getSelectedSensorPosition(0);
        //     try {Thread.sleep(current.poll_ms);} catch (Exception e) {return;}
            
    


        // }

    }
    
    public static int getPosition(int index) {
        try { return encoders.get(index).encoderPosition; } catch (Exception e) {return 0;}
        
    }
    public static void addEncoder(int poll_ms, WPI_TalonSRX m_encoderMotor, int index) {
        encoders.add(index, new MotorEncoder(poll_ms, m_encoderMotor));
    }
}

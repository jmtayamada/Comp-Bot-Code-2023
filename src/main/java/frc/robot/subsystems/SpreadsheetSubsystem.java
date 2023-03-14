// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;
import java.io.FileWriter;
import java.time.LocalDateTime;
import java.util.Date;

import com.opencsv.CSVWriter;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;

class LogMonad {
  String reason;
  String data;
  public LogMonad (String e, Object d) {
    reason = e;
    data = d.toString();


  }
}

public class SpreadsheetSubsystem extends SubsystemBase {
  private final String path = Filesystem.getDeployDirectory() + "log.csv";
  private FileWriter fWriter;
  private CSVWriter writer;
  private boolean functional = false;
  private String[] curLine;
  private String[] columns;
  private Timer timestampTimer = new Timer();
  /** Creates a new ExampleSubsystem. */
  public SpreadsheetSubsystem() {initialize();}


  public void initialize() {
    try {
      
      fWriter = new FileWriter(path, true);
      writer = new CSVWriter(fWriter);
      columns = new String[]{"", "Stuff", "Things"};
      curLine = new String[columns.length];
      writer.writeNext(columns);
      timestampTimer.start();
      functional = true;

    } catch (Exception e ) {

      e.printStackTrace();

    }


  }
  
  
  @Override
  public void periodic() {
    if (functional) {
      curLine[0] = new Double(timestampTimer.get()).toString();
      curLine[0] = "hehe";
      curLine[1] = "haha";
      
      writer.writeNext(curLine);




    }
    
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
 

}

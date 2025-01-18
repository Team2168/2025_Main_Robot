// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import org.opencv.core.Mat;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  public Vision() {

  }

    CvSink cvSink = CameraServer.getInstance().getVideo();
    
   Mat image = new Mat();

    cvSink.grabFrame(image);

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

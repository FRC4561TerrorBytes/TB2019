/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class CrosshairCommand extends Command {

  //OpenCV input/output streams
  CvSink cvSink; // Input source for the unmodified stream
  CvSource outputStream; // Output stream for the modified stream (ie it has crosshairs)

  // Points used to draw the crosshair 
  Point horizCrosshairPoint1 = new Point(83, 72); // Left point of the crosshair
  Point horizCrosshairPoint2 = new Point(93, 72); // Right point of the crosshair
  Point vertCrosshairPoint1 = new Point(88, 67); // Bottom point of the crosshair
  Point vertCrosshairPoint2 = new Point(88, 77); // Top point of the crosshair

  // Color Variable, in BGR (Blue, Green, Red) Format
  Scalar green = new Scalar(0, 255, 0);
  Scalar blue = new Scalar(255, 0, 0);
  Scalar red = new Scalar(0, 0, 255);

  Mat source = new Mat(); //Blank Matrix where we put the image we want to modify
  
  double xCenter; //x-coordinate of the center of the vision target (if we have one)

  public CrosshairCommand() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    cvSink = CameraServer.getInstance().getVideo(Robot.camera1);
    cvSink.setEnabled(true);
    outputStream = CameraServer.getInstance().putVideo("Crosshair Cam", 176, 144);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    cvSink.grabFrame(source); // Grab source image
    xCenter = Robot.networkTable.getEntry("xcenter").getDouble(4561); // grab x-coordinate of the center of the vision target
    xCenter = (xCenter - 0) * (176) / (640); // Do math to convert x-center to a coordinate that works with the drive cam
    Point visionCenterPoint = new Point(xCenter, 72); // point which is the center of the vision target
    Imgproc.line(source, horizCrosshairPoint1, horizCrosshairPoint2, green, 1); // Add horizontal crosshair
    Imgproc.line(source, vertCrosshairPoint1, vertCrosshairPoint2, green, 1); // Add vertical crosshair
    // Put a circle around the center of a vision target if we have acquired one
    if(xCenter != 1254.275) Imgproc.circle(source, visionCenterPoint, 20, blue, 1);
    outputStream.putFrame(source); // Add this image to the output stream
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}

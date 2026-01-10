// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

// This declares a public class named "Main"; "final" means it can't be extended by other classes.
public final class Main {
    // This is a private constructor for the Main class. Constructors are special methods that run when an object is created. Making it private prevents anyone from creating an instance of this class, which makes sense because this class is just a starting point and doesn't need objects made from it.
  private Main() {}

    // This is the main method, which is the starting point for any Java program. "Public static void" means it's accessible from anywhere, doesn't need an object to run, and doesn't return anything. "String... args" allows it to accept command-line arguments as an array of strings.
  public static void main(String... args) {
    
        // This line calls a static method from the RobotBase class to start the robot program. "Robot::new" is a method reference that tells it to create a new instance of a class called "Robot" (which is exists elsewhere in the project). This launches the actual robot code.
    RobotBase.startRobot(Robot::new);
  }
}

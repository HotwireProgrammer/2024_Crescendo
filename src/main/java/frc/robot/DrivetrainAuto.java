// package frc.robot;
// // package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.motorcontrol.Talon;

// import com.ctre.phoenix.motorcontrol.can.TalonFX;
// import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
// import com.ctre.phoenix.motorcontrol.can.TalonSRX;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
// import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
// import edu.wpi.first.wpilibj.ADXRS450_Gyro;
// import edu.wpi.first.wpilibj.Encoder;
// import edu.wpi.first.wpilibj.drive.DifferentialDrive;
// // import frc.robot.Constants.DriveConstants;
// import edu.wpi.first.wpilibj.interfaces.Gyro;
// import edu.wpi.first.wpilibj.motorcontrol.MotorController;
// import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
// // import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
// import edu.wpi.first.wpilibj.command.Subsystem;

// public class DrivetrainAuto extends Subsystem {
//     public TalonFX m_leftMotor1 = new TalonFX(54);
//     public TalonFX m_leftMotor2 = new TalonFX(55);
//     public TalonFX m_rightMotor1 = new TalonFX(56);
//     public TalonFX m_rightMotor2 = new TalonFX(57);

// // The left-side drive encoder

// private final Encoder m_leftEncoder = new Encoder(DriveConstants.kLeftEncoderPorts[0], DriveConstants.kLeftEncoderPorts[1], DriveConstants.kLeftEncoderReversed);

// // The right-side drive encoder

// private final Encoder m_rightEncoder = new Encoder(DriveConstants.kRightEncoderPorts[0], DriveConstants.kRightEncoderPorts[1], DriveConstants.kRightEncoderReversed);


// // The gyro sensor

// private final Gyro m_gyro = new ADXRS450_Gyro();


// // Odometry class for tracking robot pose

// private final DifferentialDriveOdometry m_odometry;


// /** Creates a new DriveSubsystem. */

// public DrivetrainAuto() {

// // We need to invert one side of the drivetrain so that positive voltages
// // result in both sides moving forward. Depending on how your robot's
// // gearbox is constructed, you might have to invert the left side instead.
// m_rightMotor1.setInverted(true);
// m_rightMotor2.setInverted(true);


// // Sets the distance per pulse for the encoders
// m_leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
// m_rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
// resetEncoders();
// m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());
// }



// public void autoDrivetrain() {
// // Update the odometry in the periodic block
// m_odometry.update(
//     m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
// }

// //Returns the currently-estimated pose of the robot.

// public Pose2d getPose() {
// return m_odometry.getPoseMeters();
// }



// // Returns the current wheel speeds of the robot.
// // return The current wheel speeds.

// public DifferentialDriveWheelSpeeds getWheelSpeeds() {
// return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
// }
// //Resets the odometry to the specified pose.

// // param pose The pose to which to set the odometry.

// public void resetOdometry(Pose2d pose) {
// resetEncoders();
// m_odometry.resetPosition(pose, m_gyro.getRotation2d());
// }

// // Controls the left and right sides of the drive directly with voltages.
// // param leftVolts the commanded left output
// // param rightVolts the commanded right output


// public void tankDriveVolts(double leftVolts, double rightVolts) {
// ((MotorController) m_leftMotor1).setVoltage(leftVolts);
// ((MotorController) m_leftMotor2).setVoltage(leftVolts);
// ((MotorController) m_rightMotor1).setVoltage(rightVolts);
// ((MotorController) m_rightMotor2).setVoltage(rightVolts);

// }

// //Resets the drive encoders to currently read a position of 0. 
// public void resetEncoders() {
// m_leftEncoder.reset();
// m_rightEncoder.reset();
// }

// // Gets the average distance of the two encoders.
// // return the average of the two encoder readings

// public double getAverageEncoderDistance() {
// return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2.0;
// }

// // Gets the left drive encoder.
// // return the left drive encoder
// public Encoder getLeftEncoder() {
// return m_leftEncoder;
// }

// // Gets the right drive encoder.
// // return the right drive encoder

// public Encoder getRightEncoder() {
// return m_rightEncoder;
// }

// // Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
// // param maxOutput the maximum output to which the drive will be constrained
// public void setMaxOutput(double maxOutput) {
// // .setMaxOutput(maxOutput);
// }

// // Zeroes the heading of the robot. 

// public void zeroHeading() {
// m_gyro.reset();
// }

// // Returns the heading of the robot.
// // return the robot's heading in degrees, from -180 to 180

// public double getHeading() {
// return m_gyro.getRotation2d().getDegrees();
// }
// // Returns the turn rate of the robot.
// // return The turn rate of the robot, in degrees per second
// public double getTurnRate() {
// return -m_gyro.getRate();
// }


// @Override
// protected void initDefaultCommand() {
//     // TODO Auto-generated method stub
    
// }

// }

package frc.robot;

import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.DriveTrain;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Limelight {

    private float targetBuffer = 3f;

    public double p = 0.03;//0.04// late 0.03
    public double i = 0.005;
    public double d = 0.005;//0.001

    String pKey = "limelight_P";
    String iKey = "limelight_I";
    String dKey = "limelight_D";

    public PIDController pid = new PIDController(p, i, d);

    public Limelight() {

    }

    public void Init() {
        SmartDashboard.putNumber(pKey, p);
        SmartDashboard.putNumber(iKey, i);
        SmartDashboard.putNumber(dKey, d);
    }

    public void SetLight(boolean turnOn) {
        if (turnOn) {
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
        } else {
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);// 1
        }
    }

    public double GetArea() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry ta = table.getEntry("ta");
        NetworkTableEntry tv = table.getEntry("tv");

        if (tv.getDouble(0.0f) > 0) {
            return ta.getDouble(0.0);
        }

        return 0.0;
    }
    
    public double gety() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry ty = table.getEntry("ty");
        return ty.getDouble(0.0);
    }

    public boolean OnTarget() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry tx = table.getEntry("tx");
        NetworkTableEntry ty = table.getEntry("ty");
        NetworkTableEntry ta = table.getEntry("ta");
        NetworkTableEntry tv = table.getEntry("tv");

        // Make sure we have valid targets first
        if (tv.getDouble(0.0f) > 0) {

            double x = Math.abs(tx.getDouble(0.0));
            return x < targetBuffer;
        }
        pid.reset();
        return true;
    }

    public void Position(DriveTrain driveTrain) {

        if (OnTarget()) {
            driveTrain.SetBothSpeed(0.0f);
            return;
        }

        // get data from smart dashboard
        p = SmartDashboard.getNumber(pKey, p);
        i = SmartDashboard.getNumber(iKey, i);
        d = SmartDashboard.getNumber(dKey, d);

        // give data to pid class
        pid.setP(p);
        pid.setI(i);
        pid.setD(d);

        // get current error
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry tx = table.getEntry("tx");

        // calculate 
        float pidOut = (float)pid.calculate(tx.getDouble(0.0), 0);

        // use to increase error
        // double x;
        // x = (float)tx.getDouble(0.0);
        // float dir = (float) x/ (float)Math.abs(x);
        // if (Math.abs(pidOut) < 0.13f){
        //     pidOut = 0.13f * dir;
        // }
        driveTrain.SetLeftSpeed(-pidOut);
        driveTrain.SetRightSpeed(pidOut);
    }
    public void reset(){
        pid.reset();
    }
}
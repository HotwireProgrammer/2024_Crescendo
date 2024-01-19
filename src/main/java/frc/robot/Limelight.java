package frc.robot;

import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.DriveTrain;
import frc.robot.swerve.DriveSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Limelight {

    private float targetBuffer = 3f;

    public double p = 0.02;
    public double i = 0.000;
    public double d = 0.25;

    public double pTranslate = 0.035;
    public double iTranslate = 0.000;
    public double dTranslate = 0.1;

    public double pDistance = 0.05;
    public double iDistance = 0.000;
    public double dDistance = 0.045;

    String pKey = "limelight_P";
    String iKey = "limelight_I";
    String dKey = "limelight_D";

    String pTranslateKey = "limelight_P_translate";
    String iTranslateKey = "limelight_I_translate";
    String dTranslateKey = "limelight_D_translate";

    String pDistanceKey = "limelight_P_distance";
    String iDistanceKey = "limelight_I_distance";
    String dDistanceKey = "limelight_D_distance";

    public PIDController pid = new PIDController(p, i, d);
    public PIDController pid_translate = new PIDController(p, i, d);
    public PIDController pid_distance = new PIDController(p, i, d);

    public Limelight() {

    }

    public void Init() {
        SmartDashboard.putNumber(pKey, p);
        SmartDashboard.putNumber(iKey, i);
        SmartDashboard.putNumber(dKey, d);

        SmartDashboard.putNumber(pTranslateKey, pTranslate);
        SmartDashboard.putNumber(iTranslateKey, iTranslate);
        SmartDashboard.putNumber(dTranslateKey, dTranslate);
    
        SmartDashboard.putNumber(pDistanceKey, pDistance);
        SmartDashboard.putNumber(iDistanceKey, iDistance);
        SmartDashboard.putNumber(dDistanceKey, dDistance);
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

    public double GetY() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry ty = table.getEntry("ty");
        return ty.getDouble(0.0);
    }

    public double GetX() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry ty = table.getEntry("tx");
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

    public void PositionRotate(DriveSubsystem swerve) {

        if (OnTarget()) {
            swerve.drive(0, 0, 0, true, true);
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
        double x = GetX();

        // calculate
        float pidOut = (float) pid.calculate(x, 0);

        swerve.drive(0, 0, pidOut, true, true);
    }

    public void PositionTranslate(DriveSubsystem swerve) {

        if (OnTarget()) {
            swerve.drive(0, 0, 0, true, true);
            return;
        }

        // get data from smart dashboard
        pTranslate = SmartDashboard.getNumber(pTranslateKey, pTranslate);
        iTranslate = SmartDashboard.getNumber(iTranslateKey, iTranslate);
        dTranslate = SmartDashboard.getNumber(dTranslateKey, dTranslate);

        // give data to pid class
        pid_translate.setP(pTranslate);
        pid_translate.setI(iTranslate);
        pid_translate.setD(dTranslate);

        // get current error
        double x = GetX();

        // calculate
        float pidOut = (float) pid_translate.calculate(x, 0);

        swerve.drive(-pidOut,0, 0, true, true);
    }

    public void PositionDistance(DriveSubsystem swerve) {

      

        // get data from smart dashboard
        pDistance = SmartDashboard.getNumber(pDistanceKey, pDistance);
        iDistance= SmartDashboard.getNumber(iDistanceKey, iDistance);
        dDistance = SmartDashboard.getNumber(dDistanceKey, dDistance);

        // give data to pid class
        pid_distance.setP(pDistance);
        pid_distance.setI(iDistance);
        pid_distance.setD(dDistance);

        // get current error
        double x = GetY();

        // calculate
        float pidOut = (float) pid_distance.calculate(x, 0);

        swerve.drive(0,pidOut, 0, true, true);
    }

    public void reset() {
        pid.reset();
    }
}
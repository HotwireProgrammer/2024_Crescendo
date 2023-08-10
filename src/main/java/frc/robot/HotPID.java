package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class HotPID {

    public String name;

    public double p = 0.0001;
    public double i = 0.0004;
    public double d = 0.000;
    public double setpoint;

    public PIDController pid;

    public HotPID(String name, double p, double i, double d) {
        this.p = p;
        this.i = i;
        this.d = d;
        this.name = name;
        pid = new PIDController(p, i, d);

        SmartDashboard.putNumber(name + "_P", p);
        SmartDashboard.putNumber(name + "_I", i);
        SmartDashboard.putNumber(name + "_D", d);
        SmartDashboard.putNumber(name + "_setpoint", setpoint);
    }

    public double Calculate(double sensorInput) {

        p = SmartDashboard.getNumber(name + "_P", p);
        i = SmartDashboard.getNumber(name + "_I", i);
        d = SmartDashboard.getNumber(name + "_D", d);

        pid.setP(p);
        pid.setI(i);
        pid.setD(d);

        return pid.calculate(sensorInput, setpoint);
    }

    public void reset() {
        pid.reset();
    }

}

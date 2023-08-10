package frc.robot.autostep;

import frc.robot.Shooter;

//import javax.swing.Timer;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.Indexer;

public class Shoot extends AutoStep {

    public Shooter shooter;
    public double rpmTarget;
    public Indexer indexer;
    private int ballsShot = 0;
    public boolean beamBreakClear = true;
    public Timer balltimer;
    public int ballsShotTarget = 5;
    private int tickStart = 0;
    private boolean toggle = false;
    public int ballcount;

    public boolean doingEnd = false;

    public Shoot(Shooter shooter, Indexer indexer) {
        super();
        this.shooter = shooter;
        this.indexer = indexer;

        autoIndex = false;
    }

    public void Begin() {
        tickStart = (int) indexer.indexerMotor.getSelectedSensorPosition();
        shooter.rpmTarget = rpmTarget;
        balltimer = new Timer();
        balltimer.stop();
        balltimer.reset();
        balltimer.start();
    }

    public void Update() {
        System.out.println(ballcount + " ballcount");
        // Timer for override

        if (balltimer.get() > 4) {
            isDone = true;
        }

        if (!indexer.secondBeam.get()) {
            toggle = true;
        }
        if (indexer.secondBeam.get() && toggle) {
            toggle = false;
            ballcount = ballcount + 1;
        }
        if (ballcount == 2) {
            isDone = true;
        } else {
            if (balltimer.get() > 2 && balltimer.get() < 2.1) {
                //System.out.println("ran back");
                indexer.RunManualForward(-1.0f, 1.0f);
            } else {
                shooter.Update();
                indexer.RunManualForward(0.6f, 0.03f);
            }
        }
    }
}
package org.firstinspires.ftc.teamcode.lib.motionprofiles;

public abstract class MotionProfile {
    //copied from another ftc team
    public double start;
    public double target;

    public MotionProfile(double start, double target) {
        this.start = start;
        this.target = target;
    }

    public double getStart() {
        return start;
    }

    public double getTarget() {
        return target;
    }
}

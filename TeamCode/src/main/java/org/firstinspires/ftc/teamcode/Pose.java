package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import java.util.Locale;

public class Pose {
    public double x, y, heading;
    public Pose(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    @NonNull
    public String toString() {
        return String.format(Locale.ENGLISH, "Pose(%f, %f, %f)",
                this.x, this.y, this.heading);
    }
}

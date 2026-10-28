package org.firstinspires.ftc.teamcode.commands;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Driver;
import org.firstinspires.ftc.teamcode.Pose;

import java.util.Locale;

public class DriveCommand extends Command {
    private enum DriveState {
        DRIVING,
        DONE
    }
    private Pose pose;
    private final AngleUnit angleUnits;
    private final DistanceUnit distanceUnits;
    private DriveState state;
    private final Driver driver;
    public DriveCommand(
            Pose pose,
            AngleUnit angleUnits, DistanceUnit distanceUnits,
            Driver driver
    ) {
        this.pose = pose;
        this.angleUnits = angleUnits;
        this.distanceUnits = distanceUnits;
        this.driver = driver;
        this.state = DriveState.DRIVING;
    }

    @Override
    public void run() {
        switch(state) {
            case DRIVING:
                pose = driver.drive(
                        pose,
                        angleUnits, distanceUnits
                );
                if(pose == null) {
                    state = DriveState.DONE;
                }
                break;

            case DONE:
                done = true;
                break;
        }
    }

    @NonNull
    public String toString() {
        if(this.pose == null) return "";
        return String.format(Locale.ENGLISH, "DriveCommand(%s): %s",
                this.pose.toString(), this.state.toString());
    }
}

package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Driver;
import org.firstinspires.ftc.teamcode.Pose;

public class DriveCommand extends Command {
    private enum DriveState {
        DRIVING,
        COMPLETE
    }
    private Pose pose;
    private final double speed;
    private final AngleUnit angleUnits;
    private final DistanceUnit distanceUnits;
    private DriveState state;
    private final Driver driver;
    public DriveCommand(
            Pose pose, double speed,
            AngleUnit angleUnits, DistanceUnit distanceUnits,
            Driver driver
    ) {
        this.pose = pose;
        this.speed = speed;
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
                        pose, speed,
                        angleUnits, distanceUnits
                );
                if(pose == null) {
                    done = true;
                    state = DriveState.COMPLETE;
                }
                break;

            case COMPLETE:
                break;
        }
    }
}

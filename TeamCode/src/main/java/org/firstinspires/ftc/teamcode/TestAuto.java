package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name="TestAuto")
public class TestAuto extends OpMode {

    private static final double SPEED = (500) / 52.0;

    private Driver driver;
    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;
    private DcMotorEx intake;

    private enum AutoState {
        DRIVING,
        DRIVING_WAIT,
        COMPLETE
    }

    private AutoState state;

    @Override
    public void init() {
        state = AutoState.DRIVING;

        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        driver = new Driver(frontLeft, frontRight, backLeft, backRight, telemetry);

        telemetry.addLine("Initialized auto");
        telemetry.update();
    }

    private Driver.Pose pose;

    @Override
    public void loop() {
        /*
         * In autonomous code we commonly use state
         * machines to execute actions in a certain
         * order. The standard procedure is to create
         * an enum (in this case AutoState) and write
         * out each step the robot will take. In
         * our case, the robot will start driving
         * (DRIVING), wait for the driving function
         * to end (DRIVING_WAIT) and exit (COMPLETE)
         * To exit we don't do anything since the
         * state doesn't change unless we change it.
         */
        switch(state) {
            case DRIVING:
                pose = new Driver.Pose(1, 1, (90.0 / 360) * 2 * Math.PI);
                pose = driver.driveTo(pose, SPEED, true);
                if(pose == null) {
                    state = AutoState.COMPLETE;
                } else {
                    state = AutoState.DRIVING_WAIT;
                    telemetry.addLine("Driving...");
                }
                break;

            case DRIVING_WAIT:
                pose = driver.driveTo(pose, SPEED, false);
                if(pose == null)
                    state = AutoState.COMPLETE;
                break;

            case COMPLETE:
                break;
        }
        telemetry.update();
    }
}

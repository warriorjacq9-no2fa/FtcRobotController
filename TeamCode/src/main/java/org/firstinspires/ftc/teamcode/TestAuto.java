package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name="TestAuto")
public class TestAuto extends OpMode {

    private static final double SPEED = (1000) / 52.0;

    private Driver driver;
    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;
    private CRServo intake;

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
        intake = hardwareMap.get(CRServo.class, "intake");

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
                pose = new Driver.Pose(0.5, 0.5, 90);
                pose = driver.driveTo(pose, SPEED, true);
                if(pose == null) {
                    state = AutoState.COMPLETE;
                    telemetry.addLine("Auto complete");
                } else {
                    if(Driver.DEBUG) {
                        telemetry.addData("driveTo", pose);
                    }
                    state = AutoState.DRIVING_WAIT;
                    telemetry.addLine("Driving...");
                }
                break;

            case DRIVING_WAIT:
                pose = driver.driveTo(pose, SPEED, false);
                if(pose == null) {
                    telemetry.addLine("Auto complete");
                    state = AutoState.COMPLETE;
                } else {
                    if(Driver.DEBUG) {
                        telemetry.addData("driveTo", pose);
                    }
                }
                break;

            case COMPLETE:
                break;
        }
        telemetry.update();
    }
}

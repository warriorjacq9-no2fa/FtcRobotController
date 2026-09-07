package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="TestAuto")
public class TestAuto extends OpMode {

    private static final double SPEED = (500) / 52.0;

    private Driver driver;
    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;
    private DcMotorEx intake;
    private IMU imu;

    private enum AutoState {
        START_DRIVE,
        DRIVING_WAIT,
        START_ROTATE,
        ROTATE_WAIT,
        COMPLETE
    }

    private AutoState state;

    @Override
    public void init() {
        state = AutoState.START_DRIVE;

        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        imu = hardwareMap.get(IMU.class, "imu");

        driver = new Driver(frontLeft, frontRight, backLeft, backRight, imu, telemetry);

        telemetry.addLine("Initialized auto");
        telemetry.update();
    }

    ElapsedTime timer = new ElapsedTime();
    private Driver.Pose pose;

    @Override
    public void loop() {
        driver.loop();
        /*
         * In autonomous code we commonly use state
         * machines to execute actions in a certain
         * order. The standard procedure is to create
         * an enum (in this case AutoState) and write
         * out each step the robot will take. In
         * our case, the robot will start driving
         * (START_DRIVE), wait for the driving function
         * to end (DRIVING_WAIT) and exit (COMPLETE)
         * To exit we don't do anything since the
         * state doesn't change unless we change it.
         */
        switch(state) {
            case START_DRIVE:
                pose = new Driver.Pose(1, 0.25, 0.5 * 2 * Math.PI);
                state = AutoState.DRIVING_WAIT;
                break;

            case DRIVING_WAIT:
                pose = driver.drive(pose, SPEED, AngleUnit.RADIANS, DistanceUnit.METER);
                if(pose == null) {
                    timer.reset();
                    state = AutoState.START_ROTATE;
                }
                break;

            case START_ROTATE:
                if(timer.seconds() < 2) break;
                pose = new Driver.Pose(0, 0, 0.5 * 2 * Math.PI);
                state = AutoState.ROTATE_WAIT;
                break;

            case ROTATE_WAIT:
                pose = driver.drive(pose, SPEED, AngleUnit.RADIANS, DistanceUnit.METER);
                if(pose == null)
                    state = AutoState.COMPLETE;
                break;

            case COMPLETE:
                telemetry.addLine("Done");
                break;
        }
        telemetry.update();
    }
}

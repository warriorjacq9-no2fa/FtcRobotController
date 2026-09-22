package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.WaitCommand;

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
        INIT,
        WAIT,
        COMPLETE
    }

    private AutoState state;

    @Override
    public void init() {
        state = AutoState.INIT;

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

    @Override
    public void loop() {
        switch(state) {
            case INIT:
                driver.doCommand(new DriveCommand(
                        new Pose(1, 1, 0), SPEED,
                        AngleUnit.RADIANS, DistanceUnit.METER,
                        driver
                        ));
                driver.doCommand(new WaitCommand(1.0));
                driver.doCommand(new DriveCommand(
                        new Pose(0, 0, 0.5 * 2 * Math.PI), SPEED,
                        AngleUnit.RADIANS, DistanceUnit.METER,
                        driver
                ));
                driver.doCommand(new WaitCommand(1.0));
                driver.doCommand(new DriveCommand(
                        new Pose(-1, -1, 0.5 * 2 * Math.PI), SPEED,
                        AngleUnit.RADIANS, DistanceUnit.METER,
                        driver
                ));
                state = AutoState.WAIT;
                break;

            case WAIT:
                if(driver.isCommandsEmpty())
                    state = AutoState.COMPLETE;
                break;

            case COMPLETE:
                telemetry.addLine("Done");
                break;
        }
        driver.loop();
        telemetry.update();
    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
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

    private Driver driver;

    private enum AutoState {
        INIT,
        WAIT,
        COMPLETE
    }

    private AutoState state;

    @Override
    public void init() {
        state = AutoState.INIT;

        DcMotorEx frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        DcMotorEx frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        DcMotorEx backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        DcMotorEx backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        IMU imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
        );
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        backLeft.setDirection(DcMotorEx.Direction.REVERSE);

        driver = new Driver(frontLeft, frontRight, backLeft, backRight, imu, telemetry);

        telemetry.addLine("Initialized auto");
        telemetry.update();
    }

    @Override
    public void loop() {
        switch(state) {
            case INIT:
                driver.doCommand(new DriveCommand(
                        new Pose(1, 1, 0),
                        AngleUnit.RADIANS, DistanceUnit.METER,
                        driver
                        ));
                driver.doCommand(new WaitCommand(1.0));
                driver.doCommand(new DriveCommand(
                        new Pose(0, 0, 0.5 * 2 * Math.PI),
                        AngleUnit.RADIANS, DistanceUnit.METER,
                        driver
                ));
                driver.doCommand(new WaitCommand(1.0));
                driver.doCommand(new DriveCommand(
                        new Pose(-1, -1, 0.5 * 2 * Math.PI),
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

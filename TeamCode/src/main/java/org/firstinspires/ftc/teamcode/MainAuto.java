package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.WaitCommand;

@Autonomous(name="MainAuto")
public class MainAuto extends OpMode {

    private static final double SPEED = (500) / 52.0;
    private static final double INTAKE_SPEED = 120;

    private Driver driver;
    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;
    private DcMotorEx intake;
    private IMU imu;

    private enum AutoState {
        INIT,
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
                intake.setVelocity(INTAKE_SPEED);
                driver.doCommand(new DriveCommand(
                        new Pose(-18, 0, 0), SPEED,
                        AngleUnit.RADIANS, DistanceUnit.INCH,
                        driver
                ));

                // TODO: shoot x4
                driver.doCommand(new WaitCommand(0.25));

                driver.doCommand(new DriveCommand(
                        new Pose(-18, 36, -0.25 * 2 * Math.PI), SPEED,
                        AngleUnit.RADIANS, DistanceUnit.INCH,
                        driver
                ));

                // TODO: intake from flower
                driver.doCommand(new WaitCommand(0.25));

                driver.doCommand(new DriveCommand(
                        new Pose(-78, -36, -0.75 * 2 * Math.PI), SPEED,
                        AngleUnit.RADIANS, DistanceUnit.INCH,
                        driver
                ));

                // TODO: shoot x4
                driver.doCommand(new WaitCommand(0.25));

                driver.doCommand(new DriveCommand(
                        new Pose(12, 42, 0), SPEED,
                        AngleUnit.RADIANS, DistanceUnit.INCH,
                        driver
                ));
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

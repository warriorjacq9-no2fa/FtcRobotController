package org.firstinspires.ftc.teamcode.mechbots;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Driver;
import org.firstinspires.ftc.teamcode.Pose;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.WaitCommand;
import org.firstinspires.ftc.teamcode.mechbots.commands.ShootCommand;

@Autonomous(name="MechBotsAuto")
public class MechBotsAuto extends OpMode {

    private static final double SPEED = (500) / 52.0;
    private static final double INTAKE_SPEED = 120;

    private Driver driver;
    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;
    private DcMotorEx intake;
    private DcMotorEx shooter, sGate;
    private IMU imu;

    private enum AutoState {
        INIT,
        WAIT,
        COMPLETE
    }

    private AutoState state;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        state = AutoState.INIT;

        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        sGate = hardwareMap.get(DcMotorEx.class, "sGate");
        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
        );
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

        frontLeft.setDirection(DcMotorEx.Direction.FORWARD);
        frontRight.setDirection(DcMotorEx.Direction.FORWARD);
        backLeft.setDirection(DcMotorEx.Direction.FORWARD);
        backRight.setDirection(DcMotorEx.Direction.REVERSE);

        driver = new Driver(frontLeft, frontRight, backLeft, backRight, imu, telemetry);

        telemetry.addLine("Initialized auto");
        telemetry.update();
    }

    @Override
    public void loop() {
        switch(state) {
            case INIT:
                driver.doCommand(new DriveCommand(
                        new Pose(300, 1100, 0),
                        AngleUnit.DEGREES, DistanceUnit.MM,
                        driver
                ));
                driver.doCommand(new WaitCommand(1));
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

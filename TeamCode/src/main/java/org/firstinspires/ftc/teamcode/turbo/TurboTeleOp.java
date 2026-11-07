package org.firstinspires.ftc.teamcode.turbo;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.LimelightDriver;
import org.firstinspires.ftc.teamcode.Driver;

import java.util.ArrayList;
import java.util.Arrays;

/*
 * The FTC controller has two types of operation
 * that we need to implement, TeleOp and Autonomous
 * TeleOp is how the drive team operates the robot
 * and is usually pretty simple
 */
@TeleOp(name="TurboTeleOp")
public class TurboTeleOp extends LinearOpMode {

    /*
     * Speed is in radians per second, so we
     * use the wheel radius to calculate speed
     * from mm/s
     */
    private static final double SPEED = (750) / 52.0;

    private LimelightDriver limelightDriver;

    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private DcMotorEx intake;
    private DcMotorEx sGate;
    private DcMotorEx shooter;
    private IMU imu;

    private void drive() {
        /* Use x^3 for better sensitivity in the middle range of the joystick */
        double x = SPEED * -Math.pow(gamepad1.left_stick_y, 3);
        double y = SPEED * Math.pow(gamepad1.left_stick_x, 3);
        double rx = SPEED * Math.pow(gamepad1.right_stick_x, 3);

        /* Mecanum drive equations */
        frontLeft.setVelocity(x + y + rx, AngleUnit.RADIANS);
        frontRight.setVelocity(x - y - rx, AngleUnit.RADIANS);
        backLeft.setVelocity(x - y + rx, AngleUnit.RADIANS);
        backRight.setVelocity(x + y - rx, AngleUnit.RADIANS);
    }

    double normalize(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle <= -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
    private double heading;
    private void runImu() {
        heading = normalize(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        telemetry.addData("Heading", heading);
    }

    double targetPosition = 0;

    private void runShooter() {
        if(gamepad2.right_bumper)
            sGate.setPower(0.5);
        else
            sGate.setPower(0);

        if(gamepad2.right_trigger_pressed) {
            /*
             * This just adds a small variance to
             * the shooter control. Using x^1/3
             * on the trigger makes it more
             * sensitive close to 0 (or in our
             * case 0.5 since we shift the graph
             * using x - 0.5)
             */

            shooter.setPower(0.7 + 0.075 * Math.pow(gamepad2.right_trigger - 0.5, 1.0 / 3));
        } else {
            shooter.setPower(0);
        }

        if(gamepad2.left_trigger_pressed) {
            targetPosition += Math.pow(gamepad2.right_stick_x, 3);
            limelightDriver.enterManualMode(targetPosition);
        } else {
            limelightDriver.exitManualMode();
        }

        if(gamepad2.aWasPressed()) limelightDriver.setFiducialSet(LimelightDriver.FiducialSets.BLUE_CLOSE);
        else if(gamepad2.bWasPressed()) limelightDriver.setFiducialSet(LimelightDriver.FiducialSets.BLUE_FAR);
        else if(gamepad2.xWasPressed()) limelightDriver.setFiducialSet(LimelightDriver.FiducialSets.RED_CLOSE);
        else if(gamepad2.yWasPressed()) limelightDriver.setFiducialSet(LimelightDriver.FiducialSets.RED_FAR);
    }

    private void runIntake() {
        if(gamepad1.a) {
            intake.setPower(0);
        } else {
            intake.setPower(1);
        }
    }
    private void driveGlobal() {
        /* Use x^3 for better sensitivity in the middle range of the joystick */
        double x = -Math.pow(gamepad1.left_stick_y, 3);
        double y = Math.pow(gamepad1.left_stick_x, 3);
        double rx = SPEED * Math.pow(gamepad1.right_stick_x, 3);

        double gx = SPEED * (Math.cos(heading) * x - Math.sin(heading) * y);
        double gy = SPEED * (Math.sin(heading) * x + Math.cos(heading) * y);

        frontLeft.setVelocity(gx + gy + rx, AngleUnit.RADIANS);
        frontRight.setVelocity(gx - gy - rx, AngleUnit.RADIANS);
        backLeft.setVelocity(gx - gy + rx, AngleUnit.RADIANS);
        backRight.setVelocity(gx + gy - rx, AngleUnit.RADIANS);
    }

    @Override
    public void runOpMode() {
        /*
         * Get pointers to the motors in the robot
         * The deviceName string is the name we
         * enter in the controller
         */
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        sGate = hardwareMap.get(DcMotorEx.class, "sGate");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        DcMotorEx turret = hardwareMap.get(DcMotorEx.class, "turret");
        imu = hardwareMap.get(IMU.class, "imu");

        Limelight3A ll = hardwareMap.get(Limelight3A.class, "limelight");

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
        );
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        intake.setDirection(DcMotorEx.Direction.REVERSE);

        frontRight.setDirection(DcMotorEx.Direction.REVERSE);
        backRight.setDirection(DcMotorEx.Direction.REVERSE);

        Driver driver = new Driver(frontLeft, frontRight, backLeft, backRight, imu, telemetry);
        limelightDriver = new LimelightDriver(ll, turret, driver, telemetry);

        waitForStart();

        while(opModeIsActive()) {
            runImu();
            runIntake();
            runShooter();
            if(gamepad1.right_bumper) {
                imu.resetYaw();
            }
            if(gamepad1.right_trigger_pressed) {
                driveGlobal();
            } else {
                drive();
            }
            driver.loop();
            limelightDriver.loop();
            telemetry.update();
        }
    }
}

package org.firstinspires.ftc.teamcode.mechbots;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/*
 * The FTC controller has two types of operation
 * that we need to implement, TeleOp and Autonomous
 * TeleOp is how the drive team operates the robot
 * and is usually pretty simple
 */
@TeleOp(name="MechBotsTeleOp")
public class MechBotsTeleOp extends LinearOpMode {

    /*
     * Speed is in degrees per second, so we
     * use the wheel radius to calculate speed
     * from mm/s
     */
    private static final double SPEED = (500) / 52.0;

    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private DcMotorEx intake, sGate, shooter;
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

    private void runIntake() {
        if(gamepad1.a) {
            intake.setPower(0);
        } else {
            intake.setPower(0.5);
        }

    }

    private void runShooter() {
        if(gamepad2.a)
            sGate.setPower(0.5);
        else
            sGate.setPower(0);

        if(gamepad2.right_trigger_pressed) {
            shooter.setPower(0.7);
        } else {
            shooter.setPower(0);
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
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
        );
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        backRight.setDirection(DcMotorEx.Direction.REVERSE);

        frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while(opModeIsActive()) {
            runImu();
            if(gamepad1.left_trigger_pressed) {
                intake.setPower(-0.5);
                sGate.setPower(-0.5);
                shooter.setPower(-0.5);
            } else {
                runIntake();
                runShooter();
            }
            
            if(gamepad1.right_bumper) {
                imu.resetYaw();
            }
            if(gamepad1.right_trigger > 0.2) {
                driveGlobal();
            } else {
                drive();
            }
            telemetry.update();
        }
    }
}

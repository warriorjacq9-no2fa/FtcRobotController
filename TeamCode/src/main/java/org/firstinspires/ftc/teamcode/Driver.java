package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.commands.Command;

import java.util.ArrayDeque;
import java.util.Queue;

public class Driver {

    public static final boolean DEBUG = true;

    private static final double TOLERANCE_M = 0.01;
    private static final double TOLERANCE_RAD = 0.02;

    /* Counts per revolution, found on the product page for the motor */
    private static final double ENCODER_CPR = 537.7;
    private static final double WHEEL_DIAMETER_M = 0.104;

    /* PID constants */
    private static final double Kp = 10;
    private static final double Ki = 0.025;
    private static final double Kd = 0;
    private final DcMotorEx frontLeft;
    private final DcMotorEx frontRight;
    private final DcMotorEx backLeft;
    private final DcMotorEx backRight;
    private final IMU imu;
    private final Telemetry telemetry;
    private double oldFlEncoder;
    private double oldFrEncoder;
    private double oldBlEncoder;
    private double oldBrEncoder;
    private double heading;
    private double oldHeading;
    private final ElapsedTime loopTimer = new ElapsedTime();
    private double lastTime = 0;
    private double xIntegral = 0;
    private double yIntegral = 0;
    private double rxIntegral = 0;
    private double xDerivative = 0;
    private double yDerivative = 0;
    private double rxDerivative = 0;
    private double oldXError = 0;
    private double oldYError = 0;
    private double oldRxError = 0;

    private Pose currentPose;

    public Driver(
            DcMotorEx frontLeft, DcMotorEx frontRight,
            DcMotorEx backLeft, DcMotorEx backRight,
            IMU imu,
            Telemetry telemetry
            ) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
        this.imu = imu;
        this.telemetry = telemetry;

        heading = 0;
        oldHeading = 0;

        currentPose = new Pose(0, 0, 0);

        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        /*
         * RUN_USING_ENCODER uses the motor
         * controller's integrated hardware
         * PID control, which helps to control
         * wheel slip
         */
        frontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        state = DriverState.GET_COMMAND;

        telemetry.addLine("Driver initialized");
        telemetry.update();
    }

    double normalize(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle <= -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    double filter(double a, double b) {
        return 0.2 * a + (1 - 0.2) * b;
    }

    /**
     * Get current pose relative to when the
     * Driver class was initialized
     * This result is only valid if {@link #loop()}
     * is called every loop
     * @param angleUnits Angle units for pose
     * @param distanceUnits Distance units for
     *                      pose
     * @return Current pose
     */
    public Pose getCurrentPose(AngleUnit angleUnits, DistanceUnit distanceUnits) {
        return new Pose(
                distanceUnits.fromMeters(currentPose.x),
                distanceUnits.fromMeters(currentPose.y),
                angleUnits.fromRadians(currentPose.heading)
        );
    }

    /**
     * Get current pose relative to when the
     * Driver class was initialized
     * This result is only valid if {@link #loop()}
     * is called every loop
     * @return Current pose
     */
    public Pose getCurrentPose() {
        return currentPose;
    }

    /**
     * Drive to a given relative pose in the
     * global frame
     * @param dPose Relative target pose
     * @param angleUnits Angle units for pose
     * @param distanceUnits Distance units for
     *                      pose
     * @return True if done, false otherwise
     */
    public boolean drive(
            @NonNull Pose dPose,
            AngleUnit angleUnits, DistanceUnit distanceUnits
    ) {
        double x = distanceUnits.toMeters(dPose.x) - currentPose.x;
        double y = distanceUnits.toMeters(dPose.y) - currentPose.y;
        double rxError = angleUnits.toRadians(dPose.heading) - currentPose.heading;
        if(
                Math.abs(x) < TOLERANCE_M &&
                Math.abs(y) < TOLERANCE_M &&
                Math.abs(rxError) < TOLERANCE_RAD
        ) {
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
            return true;
        }

        // Transform x and y errors to robot local frame
        double xError = Math.cos(currentPose.heading) * x +
                Math.sin(currentPose.heading) * y;
        double yError = -Math.sin(currentPose.heading) * x +
                Math.cos(currentPose.heading) * y;

        double currentTime = loopTimer.seconds();
        double loopTime = currentTime - lastTime;
        lastTime = currentTime;

        xIntegral += xError * loopTime;
        yIntegral += yError * loopTime;
        rxIntegral += rxError * loopTime;

        xDerivative = filter((xError - oldXError) / loopTime, xDerivative);
        yDerivative = filter((yError - oldYError) / loopTime, yDerivative);
        rxDerivative = filter((rxError - oldRxError) / loopTime, rxDerivative);

        double xUt = Kp * xError + Ki * xIntegral + Kd * xDerivative;
        double yUt = Kp * yError + Ki * yIntegral + Kd * yDerivative;
        double rxUt = Kp * rxError + Ki * rxIntegral + Kd * rxDerivative;

        oldXError = xError;
        oldYError = yError;
        oldRxError = rxError;

        double flSpeed = xUt + yUt + rxUt;
        double frSpeed = xUt - yUt - rxUt;
        double blSpeed = xUt - yUt + rxUt;
        double brSpeed = xUt + yUt - rxUt;

        frontLeft.setVelocity(flSpeed, AngleUnit.RADIANS);
        frontRight.setVelocity(frSpeed, AngleUnit.RADIANS);
        backLeft.setVelocity(blSpeed, AngleUnit.RADIANS);
        backRight.setVelocity(brSpeed, AngleUnit.RADIANS);

        if(DEBUG) {
            telemetry.addData("Global X error", distanceUnits.toMeters(x));
            telemetry.addData("Global Y error", distanceUnits.toMeters(y));
            telemetry.addData("X error", xError);
            telemetry.addData("Y error", yError);
            telemetry.addData("RX error", distanceUnits.toMeters(rxError));
            telemetry.addData("FL speed", flSpeed);
            telemetry.addData("FR speed", frSpeed);
            telemetry.addData("BL speed", blSpeed);
            telemetry.addData("BR speed", brSpeed);
            telemetry.addData("PID X out", xUt);
            telemetry.addData("PID Y out", yUt);
            telemetry.addData("PID RX out", rxUt);
            telemetry.addData("PID X integral", xIntegral);
            telemetry.addData("PID Y integral", yIntegral);
            telemetry.addData("PID RX integral", rxIntegral);
            telemetry.addData("PID X derivative", xDerivative);
            telemetry.addData("PID Y derivative", yDerivative);
            telemetry.addData("PID RX derivative", rxDerivative);
            telemetry.addData("Loop time", loopTime);
        }

        return false;
    }

    private enum DriverState {
        GET_COMMAND,
        RUN_COMMAND
    }

    private DriverState state;
    private final Queue<Command> commands = new ArrayDeque<>();
    private Command currentCmd;

    /**
     * To be called during loop() in an OpMode
     */
    public void loop() {
        heading = normalize(-imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        double flEncoder = frontLeft.getCurrentPosition();
        double frEncoder = frontRight.getCurrentPosition();
        double blEncoder = backLeft.getCurrentPosition();
        double brEncoder = backRight.getCurrentPosition();

        double flRadians = ((flEncoder - oldFlEncoder) / ENCODER_CPR) * 2 * Math.PI;
        double frRadians = ((frEncoder - oldFrEncoder) / ENCODER_CPR) * 2 * Math.PI;
        double blRadians = ((blEncoder - oldBlEncoder) / ENCODER_CPR) * 2 * Math.PI;
        double brRadians = ((brEncoder - oldBrEncoder) / ENCODER_CPR) * 2 * Math.PI;

        double dx = (WHEEL_DIAMETER_M / 8) * (flRadians + frRadians + blRadians + brRadians);
        double dy = (WHEEL_DIAMETER_M / 8) * (flRadians - frRadians - blRadians + brRadians);

        double g_dx = Math.cos(heading) * dx - Math.sin(heading) * dy;
        double g_dy = Math.sin(heading) * dx + Math.cos(heading) * dy;

        currentPose = new Pose(
                currentPose.x + g_dx,
                currentPose.y + g_dy,
                heading
        );

        switch(state) {
            case GET_COMMAND:
                Command cmd = commands.peek();
                if(cmd != null) {
                    currentCmd = cmd;
                    commands.remove();
                    state = DriverState.RUN_COMMAND;
                }
                break;

            case RUN_COMMAND:
                currentCmd.run();
                if(currentCmd.isDone())
                    state = DriverState.GET_COMMAND;
                break;
        }
        if(DEBUG) {
            telemetry.addData("FL wheel moved", flRadians);
            telemetry.addData("FR wheel moved", frRadians);
            telemetry.addData("BL wheel moved", blRadians);
            telemetry.addData("BR wheel moved", brRadians);
            telemetry.addData("Moved X", dx);
            telemetry.addData("Moved Y", dy);
            telemetry.addData("Moved RX", (heading - oldHeading));
            telemetry.addData("Moved global X", g_dx);
            telemetry.addData("Moved global Y", g_dy);
            telemetry.addData("Current pose", currentPose);
            telemetry.addData("Heading", heading);
            telemetry.addData("Driver state", state);
            telemetry.addData("currentCmd", currentCmd);
            telemetry.addData("Commands left", commands.size());
        }
        oldFlEncoder = flEncoder;
        oldFrEncoder = frEncoder;
        oldBlEncoder = blEncoder;
        oldBrEncoder = brEncoder;
        oldHeading = heading;
    }

    /**
     * Queue a command in the command queue to
     * be processed and executed
     *
     * @param cmd Command to be queued
     */
    public void doCommand(Command cmd) {
        commands.add(cmd);
    }

    /**
     * Check if current command is done
     *
     * @return True if done, false otherwise
     */
    public boolean isCurrentCommandDone() {
        if(currentCmd == null) return true;
        return currentCmd.isDone();
    }

    /**
     * Check if the command queue is empty
     *
     * @return True if empty, false otherwise
     */
    public boolean isCommandsEmpty() {
        return commands.isEmpty();
    }
}

package org.firstinspires.ftc.teamcode.mechbots.autonomousprograms;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

//Talk to Lily if you want to know where "position B" is on the field. They made it up. Eventually they'll get to telling everyone.

@Autonomous(name="BPosMechBotsAuto")
public class BPosMechBotsAuto extends OpMode {

    //speed = ?
    //set up intake to maybe get balls in auto
    //imu ?
    private DcMotorEx frontRight = null;
    private DcMotorEx frontLeft = null;
    private DcMotorEx backRight = null;
    private DcMotorEx backLeft = null;
    private final ElapsedTime timer = new ElapsedTime();

    //Should this be a state machine? I don't know. Is it going to be a state machine? Yeah, unless someone tells me no.
    private enum AutoState {
        INIT,
        FORWARD,
        TURN,
        COMPLETE
    }
    private AutoState state = AutoState.INIT;

    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");

        //!!!! Not yet certain what these will need to be set to for Real Bot, but for Test Guy #1 it's FR: R, BR: F, FL: F, BL: R
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    //Drive function. 'a' is for forward, 'b' is for strafe, 'c' is for turn. Positive goes right, negative goes left (I think).
    public void drive(double forward, double strafe, double turn) {
        frontLeft.setPower(forward+strafe+turn);
        frontRight.setPower(forward-strafe-turn);
        backLeft.setPower(forward-strafe+turn);
        backRight.setPower(forward+strafe-turn);
    }

    //The length of time it is in each state needs to be figured out through testing to make it work as intended!!!
    public void loop() {
        switch (state) {
            case INIT:
                if (timer.seconds() < 1) {
                    drive(0.0, 0.0, 0.0);
                    telemetry.addLine("INIT");
                    telemetry.update();
                }
                else {
                    state = AutoState.FORWARD;
                    timer.reset();
                }
                break;

            case FORWARD:
                //Moves forward into the parking space
                //Both Test and Real use timer < 1 and a: 0.2
                if (timer.seconds() < 1) {
                    drive(0.2, 0.0, 0.0);
                    telemetry.addLine("FORWARD");
                    telemetry.update();
                }
                else {
                    state = AutoState.TURN;
                    timer.reset();
                }
                break;

            case TURN:
                //Turns to face away from the wall
                //Real Bot (14 wide * 15 long) --> timer: 2.2, a: 0.0, b: 0.0, c: -0.2
                //Test Guy #1 (18 wide * 17 long) --> timer: 2.7, a: 0.0, b: 0.0, c: -0.2
                if (timer.seconds() < 2.5) {
                    drive(0.0, 0.0, -0.2);
                    telemetry.addLine("TURN");
                    telemetry.update();
                }
                else {
                    state = AutoState.COMPLETE;
                    timer.reset();
                }
                break;

            case COMPLETE:
                drive(0.0, 0.0, 0.0);
                telemetry.addLine("PARKED");
                telemetry.update();
                break;
        }
    }
}

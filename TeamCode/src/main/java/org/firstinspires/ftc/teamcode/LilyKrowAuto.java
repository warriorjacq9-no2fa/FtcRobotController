package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name="Lily/KrowAuto")
public class LilyKrowAuto extends OpMode {

    private DcMotorEx frontRight = null;
    private DcMotorEx frontLeft = null;
    private DcMotorEx backRight = null;
    private DcMotorEx backLeft = null;

    private enum AutoState {
        INIT,
        DRIVE,
        TURN,
        COMPLETE
    }

    private AutoState state;

    public void init() {

        state = AutoState.INIT;

        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void drive(double a, double b, double c) {
        frontLeft.setPower(a + b + c);
        frontRight.setPower(a + b - c);
        backLeft.setPower(a - b + c);
        backRight.setPower(a - b - c);
    }

    public void loop() {
        switch (state) {
            case INIT:
                drive(0.0, 0.0, 0.0);
                break;
            case DRIVE:
                drive(0.2, 0.0, 0.0);
                break;
            case TURN:
                drive(0.0, 0.0, 0.2);
                break;
            case COMPLETE:
                break;
        }
    }
}

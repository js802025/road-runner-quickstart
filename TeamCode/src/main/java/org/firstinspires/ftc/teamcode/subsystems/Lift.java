package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Lift {
    public static class LiftConstants {
        public static double speed = 0.5;
        public enum dropoffOptions {
            FLOOR (0),
            LOW (0),
            MEDIUM (-960),
            HIGH (-1920);

            public int position;
            dropoffOptions(int position) {this.position = position;}

            public int position() {return position;}
        }
    }
    Telemetry telemetry;
    DcMotor liftMotor;
    public Lift(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;
        liftMotor = hardwareMap.dcMotor.get("slide");
        liftMotor.setTargetPosition(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setHeight(int height) {
        liftMotor.setTargetPosition(height);
        liftMotor.setPower(1*LiftConstants.speed);
    }

    public void setHeight(LiftConstants.dropoffOptions drop) {
        setHeight(drop.position());
    }


}

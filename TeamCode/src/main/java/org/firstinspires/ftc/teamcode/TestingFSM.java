
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.mapping.ColorMapping;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

@Autonomous(name = "TestingFSM", group = "Sensor")

public class TestingFSM extends LinearOpMode {

    /**
     * The colorSensor field will contain a reference to our color sensor hardware object
     */
    NormalizedColorSensor colorSensor;


    public enum STATE {
        INIT,
        TOLINE,
        RAISE,
        LOWER,
        STOP
    }


    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "color");

        ColorMapping colorMapping = new ColorMapping(colorSensor, drive, telemetry);
        Lift lift = new Lift(telemetry, hardwareMap);
        STATE state = STATE.TOLINE;
        waitForStart();
        if (isStopRequested()) return;
        colorMapping.stopAtLine(ColorMapping.COLORS.BLUE); //start driving to blue line;
        while (opModeIsActive() && !isStopRequested() && state != STATE.STOP) {
            switch (state) {
                case INIT:
                    break;
                case TOLINE:
                    if (!colorMapping.isBusy()) {
                        state = STATE.STOP;
                    }
                    break;
                case RAISE:
                    lift.setHeight(400);
                    break;
                case LOWER:
                    lift.setHeight(0);
                    break;
                case STOP:
                    break;

            }
            drive.update();
            lift.update();

            colorMapping.update();
        }


    }
}



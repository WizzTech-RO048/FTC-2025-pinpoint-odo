package org.firstinspires.ftc.teamcode.Robot;

import androidx.annotation.NonNull;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Objects;

public class HorizontalSlider {

    private final Telemetry telemetry;
    private final HardwareMap hardwareMap;

    private final Servo HorizontalSliderLeft, HorizontalSliderRight;

    private final double leftExtended = 0.3, leftStationed = 0.0;
    private final double rightExtended = 0.3, rightStationed = 0.61;

    HorizontalSlider(@NonNull final Parameters parameters) {
        telemetry = Objects.requireNonNull(parameters.telemetry, "Telemetry was not set up");
        hardwareMap = Objects.requireNonNull(parameters.hardwareMap, "HardwareMap was not set up");

        HorizontalSliderLeft = hardwareMap.get(Servo.class, "sliderLeft");
        HorizontalSliderRight = hardwareMap.get(Servo.class, "sliderRight");
    }

    public void setExtendedPosition() {
        HorizontalSliderLeft.setPosition(leftExtended);
        HorizontalSliderRight.setPosition(rightExtended);
    }

    public void setStationaryPosition() {
        HorizontalSliderLeft.setPosition(leftStationed);
        HorizontalSliderRight.setPosition(rightStationed);
    }

    public static class Parameters {
        public HardwareMap hardwareMap;
        public Telemetry telemetry;
    }
}

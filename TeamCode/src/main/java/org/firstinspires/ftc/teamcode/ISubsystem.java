package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public interface ISubsystem {

    void init(HardwareMap hMap, Telemetry telemetry);

    void telemetry();

    boolean isDone();

    void update();

}

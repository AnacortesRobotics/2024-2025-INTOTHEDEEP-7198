package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.dfrobot.HuskyLens.*;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SimpleHusky {

    HuskyLens huskyLens;
    Telemetry telemetry;

    SimpleHusky(HardwareMap hardwareMap, Telemetry telem)
    {
        telemetry = telem;
        huskyLens = hardwareMap.get(HuskyLens.class,"Husky Lens");
    }

    public void test()
    {
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);

        Block[] blocks = huskyLens.blocks();

        for(Block block : blocks)
        {
            telemetry.addLine(block.id + " (" + block.x + ", " + block.y + ") (" + block.width + "x" + block.height + ")");
        }
    }

    public Block getBlockPos(int id)
    {
        if (huskyLens.blocks(id).length == 0)
            return null;

        Block[] blocks = huskyLens.blocks(id);
        int largest = 0;
        int volume = 0;
        for (int i = 0; i < blocks.length; i++)
        {
            if(volume < blocks[i].x * blocks[i].y)
            {
                volume = blocks[i].x * blocks[i].y;
                largest = i;
            }
        }

        return blocks[largest];
    }

    public void getConnectionInfo()
    {
        telemetry.addLine(huskyLens.getConnectionInfo());
    }
}

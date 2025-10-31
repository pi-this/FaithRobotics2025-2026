package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.robotcore.external.ExportToBlocks;


public class TestCustomBlock extends BlocksOpModeCompanion {
    
    @ExportToBlocks (
        comment = "Hello World",
        tooltip = "Test",
        parameterLabels = {"Recipient"}
        )
    
    public static String test(){
        return("Hello World!");
    }
}

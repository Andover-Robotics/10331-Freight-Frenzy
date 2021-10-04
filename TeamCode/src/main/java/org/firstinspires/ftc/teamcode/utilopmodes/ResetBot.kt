package org.firstinspires.ftc.teamcode.utilopmodes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.hardware.Bot

@Autonomous(name = "Reset Bot instance", group = "Utility")
class ResetBot: LinearOpMode() {
    override fun runOpMode() {
        Bot.instance = null
    }
}
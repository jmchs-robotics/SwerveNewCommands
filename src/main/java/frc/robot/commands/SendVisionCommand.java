/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.util.SocketVisionSendWrapper;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class SendVisionCommand extends InstantCommand {
  SocketVisionSendWrapper sender_;
  String message_;
  public SendVisionCommand(SocketVisionSendWrapper sender, String message) {
    // Use addRequirements() here to declare subsystem dependencies.
    sender_ = sender;
    message_ = message;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    sender_.get().setSendData(message_);
  }


}

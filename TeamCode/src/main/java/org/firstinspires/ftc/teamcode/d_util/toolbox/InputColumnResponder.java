package org.firstinspires.ftc.teamcode.d_util.toolbox;

import java.util.function.Supplier;

public interface InputColumnResponder {
  InputColumnResponder register(Supplier<Boolean> predicate, Runnable triggerCallback);
  void update();
  void clearRegistry();
}

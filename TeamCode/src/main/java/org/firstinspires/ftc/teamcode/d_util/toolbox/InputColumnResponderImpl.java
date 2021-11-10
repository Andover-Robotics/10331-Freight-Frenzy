package org.firstinspires.ftc.teamcode.d_util.toolbox;

import java.util.AbstractMap;
import java.util.ArrayList;
import java.util.BitSet;
import java.util.List;
import java.util.function.Supplier;

public class InputColumnResponderImpl implements InputColumnResponder {
  private List<AbstractMap.SimpleEntry<Supplier<Boolean>, Runnable>> registry =
      new ArrayList<>();
  private BitSet previousStates = new BitSet();

  @Override
  public InputColumnResponder register(Supplier<Boolean> predicate, Runnable triggerCallback) {
    registry.add(new AbstractMap.SimpleEntry<>(predicate, triggerCallback));
    previousStates.set(registry.size() - 1, predicate.get());
    return this;
  }

  @Override
  public void update() {
    for (int i = 0; i < registry.size(); i++) {
      AbstractMap.SimpleEntry<Supplier<Boolean>, Runnable> entry = registry.get(i);
      boolean newState = entry.getKey().get();

      if (newState && !previousStates.get(i)) {
        entry.getValue().run();
      }

      previousStates.set(i, newState);
    }
  }

  @Override
  public void clearRegistry() {
    registry.clear();
    previousStates.clear();
  }
}

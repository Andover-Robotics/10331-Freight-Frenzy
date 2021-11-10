package org.firstinspires.ftc.teamcode.d_util.toolbox;

import java.util.stream.Stream;

public class Selector {
  private int selected = 0;
  // Guaranteed nonempty
  private String[] choices;

  public Selector(Stream<String> choices) {
    this.choices = choices.toArray(String[]::new);
    if (this.choices.length == 0) {
      throw new IllegalArgumentException("no choices to select!");
    }
  }

  public void selectNext() {
    selected = (selected + 1) % choices.length;
  }

  public String selected() {
    return choices[selected];
  }

  public Stream<String> allChoices() {
    return Stream.of(choices);
  }
}

package frc4488.lib.dashboard.gui;

import java.io.IOException;
import java.util.LinkedHashMap;
import java.util.Map;
import org.msgpack.core.MessagePacker;

public class TabsWidget extends Widget {

  private final LinkedHashMap<String, Widget> tabs;

  public TabsWidget() {
    super("tabs");
    this.tabs = new LinkedHashMap<>();
  }

  public void addTab(String name, Widget widget) {
    this.tabs.put(name, widget);
  }

  @Override
  protected void sendData(MessagePacker packer) throws IOException {
    packer.packInt(tabs.size());
    for (Map.Entry<String, Widget> tab : tabs.entrySet()) {
      packer.packString(tab.getKey());
      tab.getValue().sendWidget(packer);
    }
  }

  @Override
  public Widget getWidget(long id) {
    if (this.getId() == id) {
      return this;
    }
    for (Widget widget : tabs.values()) {
      Widget found = widget.getWidget(id);
      if (found != null) {
        return found;
      }
    }
    return null;
  }
}

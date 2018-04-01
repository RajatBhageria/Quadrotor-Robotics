package std_srvs;

public interface SetBoolRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "std_srvs/SetBoolRequest";
  static final java.lang.String _DEFINITION = "bool data # e.g. for hardware enabling / disabling\n";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
  boolean getData();
  void setData(boolean value);
}

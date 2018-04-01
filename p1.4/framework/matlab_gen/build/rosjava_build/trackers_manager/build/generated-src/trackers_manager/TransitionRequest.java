package trackers_manager;

public interface TransitionRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "trackers_manager/TransitionRequest";
  static final java.lang.String _DEFINITION = "string tracker\n";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
  java.lang.String getTracker();
  void setTracker(java.lang.String value);
}

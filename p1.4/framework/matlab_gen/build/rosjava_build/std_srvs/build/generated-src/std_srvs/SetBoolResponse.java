package std_srvs;

public interface SetBoolResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "std_srvs/SetBoolResponse";
  static final java.lang.String _DEFINITION = "bool success   # indicate successful run of triggered service\nstring message # informational, e.g. for error messages";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
  boolean getSuccess();
  void setSuccess(boolean value);
  java.lang.String getMessage();
  void setMessage(java.lang.String value);
}

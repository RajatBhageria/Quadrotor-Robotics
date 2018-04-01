package quadrotor_msgs;

public interface FlatOutputs extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "quadrotor_msgs/FlatOutputs";
  static final java.lang.String _DEFINITION = "float32 x\nfloat32 y\nfloat32 z\nfloat32 yaw\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  float getX();
  void setX(float value);
  float getY();
  void setY(float value);
  float getZ();
  void setZ(float value);
  float getYaw();
  void setYaw(float value);
}

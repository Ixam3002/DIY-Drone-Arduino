#include <math.h>

struct joysticks {
  float stick_gx;
  float stick_gy;
  float stick_dx;
  float stick_dy;
};

struct CommandJoystick
{
    float roll;
    float pitch;
    float yaw;
    float throttle;
};



CommandJoystick joystick2angle (
    float stick_gx, float stick_gy,
    float stick_dx, float stick_dy,
    float angle_max
){
    CommandJoystick cmd;

    cmd.roll = stick_gx * angle_max;
    cmd.pitch = -stick_gy * angle_max;
    cmd.throttle = (stick_dy) * 5.0f;
    cmd.yaw = 0.0f;

    return cmd;
}
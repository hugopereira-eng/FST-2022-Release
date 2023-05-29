# message for sta position information

std_msgs/Header header

int32 deltaSTA                    # positionActual - positionDemandSTA
int32 deltaController             # positionActual - positionDemandController
int32 positionActual
int32 positionDemandSTA
int32 positionDemandController
float32 steeringEncoderSTA        # positionActual / GR / Encoder positions (4096) * 360º
float32 torqueActual              # Nm
float32 velocityActual            # rad/s
float32 powerActual               # Watt
bool limit
std_msgs/Header header

int64     msg_cnt
uint8[8]  id
int64 sec_mark      # DSecond

# TimeConfidence	 timeConfidence   #OPTIONAL,

Position3D          pos 
## PositionalAccuracy  posAccuracy    # Accuracy for GNSS system

## struct PositionConfidenceSet	*posConfidence	/* OPTIONAL */
int64 transmission
int64 speed 
int64 heading

## SteeringWheelAngle_t	*angle	   # /* OPTIONAL */
## struct MotionConfidenceSet	*motionCfd	/* OPTIONAL */

AccelerationSet4Way	 accel_set
BrakeSystemStatus	 brakes
VehicleSize	     size
VehicleClassification	 vehicle_class

# # struct VehicleSafetyExtensions	*safetyExt	/* OPTIONAL */
# # struct VehicleEmergencyExtensions	*emergencyExt	/* OPTIONAL */

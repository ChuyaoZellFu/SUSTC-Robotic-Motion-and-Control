; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude StatusFlags.msg.html

(cl:defclass <StatusFlags> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass StatusFlags (<StatusFlags>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StatusFlags>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StatusFlags)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<StatusFlags> is deprecated: use kortex_driver-msg:StatusFlags instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<StatusFlags>)))
    "Constants for message type '<StatusFlags>"
  '((:UNKNOWN_STATUS . 0)
    (:STABILIZED . 16)
    (:MOTOR_INDEXED . 32)
    (:MOTOR_INDEXING . 64)
    (:JOINT_INDEXED . 128)
    (:JOINT_INDEXING . 256)
    (:HIGH_PRECISION . 512)
    (:BRAKING . 1024)
    (:SERVOING . 2048)
    (:MAJOR_FAULT . 4096)
    (:MINOR_FAULT . 8192)
    (:CALIBRATED_TORQUE . 16384)
    (:CALIBRATED_MAG_SENSOR . 32768)
    (:CALIBRATED_ZERO . 65536)
    (:GPIO_0 . 131072)
    (:GPIO_1 . 262144)
    (:CS_QUASI_STATIC_CONTACT . 524288)
    (:CS_TRANSIENT_CONTACT . 1048576)
    (:VFD_HALL_SYNC . 2097152)
    (:VFD_INDEXED . 4194304)
    (:DRIVE_BOARD_READY . 8388608)
    (:CALIBRATED_CURRENT . 16777216)
    (:CALIBRATED_MOTOR . 33554432)
    (:SW0_ACTIVE . 67108864)
    (:SW1_ACTIVE . 134217728))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'StatusFlags)))
    "Constants for message type 'StatusFlags"
  '((:UNKNOWN_STATUS . 0)
    (:STABILIZED . 16)
    (:MOTOR_INDEXED . 32)
    (:MOTOR_INDEXING . 64)
    (:JOINT_INDEXED . 128)
    (:JOINT_INDEXING . 256)
    (:HIGH_PRECISION . 512)
    (:BRAKING . 1024)
    (:SERVOING . 2048)
    (:MAJOR_FAULT . 4096)
    (:MINOR_FAULT . 8192)
    (:CALIBRATED_TORQUE . 16384)
    (:CALIBRATED_MAG_SENSOR . 32768)
    (:CALIBRATED_ZERO . 65536)
    (:GPIO_0 . 131072)
    (:GPIO_1 . 262144)
    (:CS_QUASI_STATIC_CONTACT . 524288)
    (:CS_TRANSIENT_CONTACT . 1048576)
    (:VFD_HALL_SYNC . 2097152)
    (:VFD_INDEXED . 4194304)
    (:DRIVE_BOARD_READY . 8388608)
    (:CALIBRATED_CURRENT . 16777216)
    (:CALIBRATED_MOTOR . 33554432)
    (:SW0_ACTIVE . 67108864)
    (:SW1_ACTIVE . 134217728))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StatusFlags>) ostream)
  "Serializes a message object of type '<StatusFlags>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StatusFlags>) istream)
  "Deserializes a message object of type '<StatusFlags>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StatusFlags>)))
  "Returns string type for a message object of type '<StatusFlags>"
  "kortex_driver/StatusFlags")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StatusFlags)))
  "Returns string type for a message object of type 'StatusFlags"
  "kortex_driver/StatusFlags")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StatusFlags>)))
  "Returns md5sum for a message object of type '<StatusFlags>"
  "94f8e368a8b96139de6828bbd6ed2797")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StatusFlags)))
  "Returns md5sum for a message object of type 'StatusFlags"
  "94f8e368a8b96139de6828bbd6ed2797")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StatusFlags>)))
  "Returns full string definition for message of type '<StatusFlags>"
  (cl:format cl:nil "~%uint32 UNKNOWN_STATUS = 0~%~%uint32 STABILIZED = 16~%~%uint32 MOTOR_INDEXED = 32~%~%uint32 MOTOR_INDEXING = 64~%~%uint32 JOINT_INDEXED = 128~%~%uint32 JOINT_INDEXING = 256~%~%uint32 HIGH_PRECISION = 512~%~%uint32 BRAKING = 1024~%~%uint32 SERVOING = 2048~%~%uint32 MAJOR_FAULT = 4096~%~%uint32 MINOR_FAULT = 8192~%~%uint32 CALIBRATED_TORQUE = 16384~%~%uint32 CALIBRATED_MAG_SENSOR = 32768~%~%uint32 CALIBRATED_ZERO = 65536~%~%uint32 GPIO_0 = 131072~%~%uint32 GPIO_1 = 262144~%~%uint32 CS_QUASI_STATIC_CONTACT = 524288~%~%uint32 CS_TRANSIENT_CONTACT = 1048576~%~%uint32 VFD_HALL_SYNC = 2097152~%~%uint32 VFD_INDEXED = 4194304~%~%uint32 DRIVE_BOARD_READY = 8388608~%~%uint32 CALIBRATED_CURRENT = 16777216~%~%uint32 CALIBRATED_MOTOR = 33554432~%~%uint32 SW0_ACTIVE = 67108864~%~%uint32 SW1_ACTIVE = 134217728~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StatusFlags)))
  "Returns full string definition for message of type 'StatusFlags"
  (cl:format cl:nil "~%uint32 UNKNOWN_STATUS = 0~%~%uint32 STABILIZED = 16~%~%uint32 MOTOR_INDEXED = 32~%~%uint32 MOTOR_INDEXING = 64~%~%uint32 JOINT_INDEXED = 128~%~%uint32 JOINT_INDEXING = 256~%~%uint32 HIGH_PRECISION = 512~%~%uint32 BRAKING = 1024~%~%uint32 SERVOING = 2048~%~%uint32 MAJOR_FAULT = 4096~%~%uint32 MINOR_FAULT = 8192~%~%uint32 CALIBRATED_TORQUE = 16384~%~%uint32 CALIBRATED_MAG_SENSOR = 32768~%~%uint32 CALIBRATED_ZERO = 65536~%~%uint32 GPIO_0 = 131072~%~%uint32 GPIO_1 = 262144~%~%uint32 CS_QUASI_STATIC_CONTACT = 524288~%~%uint32 CS_TRANSIENT_CONTACT = 1048576~%~%uint32 VFD_HALL_SYNC = 2097152~%~%uint32 VFD_INDEXED = 4194304~%~%uint32 DRIVE_BOARD_READY = 8388608~%~%uint32 CALIBRATED_CURRENT = 16777216~%~%uint32 CALIBRATED_MOTOR = 33554432~%~%uint32 SW0_ACTIVE = 67108864~%~%uint32 SW1_ACTIVE = 134217728~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StatusFlags>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StatusFlags>))
  "Converts a ROS message object to a list"
  (cl:list 'StatusFlags
))

; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude OperatingMode.msg.html

(cl:defclass <OperatingMode> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass OperatingMode (<OperatingMode>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <OperatingMode>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'OperatingMode)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<OperatingMode> is deprecated: use kortex_driver-msg:OperatingMode instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<OperatingMode>)))
    "Constants for message type '<OperatingMode>"
  '((:UNSPECIFIED_OPERATING_MODE . 0)
    (:MAINTENANCE_MODE . 1)
    (:UPDATE_MODE . 2)
    (:UPDATE_COMPLETED_MODE . 3)
    (:UPDATE_FAILED_MODE . 4)
    (:SHUTTING_DOWN_MODE . 5)
    (:RUN_MODE . 6)
    (:UPDATING_DEVICE_MODE . 7))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'OperatingMode)))
    "Constants for message type 'OperatingMode"
  '((:UNSPECIFIED_OPERATING_MODE . 0)
    (:MAINTENANCE_MODE . 1)
    (:UPDATE_MODE . 2)
    (:UPDATE_COMPLETED_MODE . 3)
    (:UPDATE_FAILED_MODE . 4)
    (:SHUTTING_DOWN_MODE . 5)
    (:RUN_MODE . 6)
    (:UPDATING_DEVICE_MODE . 7))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <OperatingMode>) ostream)
  "Serializes a message object of type '<OperatingMode>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <OperatingMode>) istream)
  "Deserializes a message object of type '<OperatingMode>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<OperatingMode>)))
  "Returns string type for a message object of type '<OperatingMode>"
  "kortex_driver/OperatingMode")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'OperatingMode)))
  "Returns string type for a message object of type 'OperatingMode"
  "kortex_driver/OperatingMode")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<OperatingMode>)))
  "Returns md5sum for a message object of type '<OperatingMode>"
  "5399510deee390e03ac867dbfd636f27")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'OperatingMode)))
  "Returns md5sum for a message object of type 'OperatingMode"
  "5399510deee390e03ac867dbfd636f27")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<OperatingMode>)))
  "Returns full string definition for message of type '<OperatingMode>"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_OPERATING_MODE = 0~%~%uint32 MAINTENANCE_MODE = 1~%~%uint32 UPDATE_MODE = 2~%~%uint32 UPDATE_COMPLETED_MODE = 3~%~%uint32 UPDATE_FAILED_MODE = 4~%~%uint32 SHUTTING_DOWN_MODE = 5~%~%uint32 RUN_MODE = 6~%~%uint32 UPDATING_DEVICE_MODE = 7~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'OperatingMode)))
  "Returns full string definition for message of type 'OperatingMode"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_OPERATING_MODE = 0~%~%uint32 MAINTENANCE_MODE = 1~%~%uint32 UPDATE_MODE = 2~%~%uint32 UPDATE_COMPLETED_MODE = 3~%~%uint32 UPDATE_FAILED_MODE = 4~%~%uint32 SHUTTING_DOWN_MODE = 5~%~%uint32 RUN_MODE = 6~%~%uint32 UPDATING_DEVICE_MODE = 7~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <OperatingMode>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <OperatingMode>))
  "Converts a ROS message object to a list"
  (cl:list 'OperatingMode
))

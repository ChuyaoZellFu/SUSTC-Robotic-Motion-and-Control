; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude RunModes.msg.html

(cl:defclass <RunModes> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass RunModes (<RunModes>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RunModes>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RunModes)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<RunModes> is deprecated: use kortex_driver-msg:RunModes instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<RunModes>)))
    "Constants for message type '<RunModes>"
  '((:RUN_MODE . 0)
    (:CALIBRATION_MODE . 1)
    (:CONFIGURATION_MODE . 2)
    (:DEBUG_MODE . 3)
    (:TUNING_MODE . 4))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'RunModes)))
    "Constants for message type 'RunModes"
  '((:RUN_MODE . 0)
    (:CALIBRATION_MODE . 1)
    (:CONFIGURATION_MODE . 2)
    (:DEBUG_MODE . 3)
    (:TUNING_MODE . 4))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RunModes>) ostream)
  "Serializes a message object of type '<RunModes>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RunModes>) istream)
  "Deserializes a message object of type '<RunModes>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RunModes>)))
  "Returns string type for a message object of type '<RunModes>"
  "kortex_driver/RunModes")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RunModes)))
  "Returns string type for a message object of type 'RunModes"
  "kortex_driver/RunModes")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RunModes>)))
  "Returns md5sum for a message object of type '<RunModes>"
  "4d935d7a06871eb332098b80bd9adf59")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RunModes)))
  "Returns md5sum for a message object of type 'RunModes"
  "4d935d7a06871eb332098b80bd9adf59")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RunModes>)))
  "Returns full string definition for message of type '<RunModes>"
  (cl:format cl:nil "~%uint32 RUN_MODE = 0~%~%uint32 CALIBRATION_MODE = 1~%~%uint32 CONFIGURATION_MODE = 2~%~%uint32 DEBUG_MODE = 3~%~%uint32 TUNING_MODE = 4~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RunModes)))
  "Returns full string definition for message of type 'RunModes"
  (cl:format cl:nil "~%uint32 RUN_MODE = 0~%~%uint32 CALIBRATION_MODE = 1~%~%uint32 CONFIGURATION_MODE = 2~%~%uint32 DEBUG_MODE = 3~%~%uint32 TUNING_MODE = 4~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RunModes>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RunModes>))
  "Converts a ROS message object to a list"
  (cl:list 'RunModes
))

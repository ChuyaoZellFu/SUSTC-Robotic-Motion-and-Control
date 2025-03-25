; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude CalibrationItem.msg.html

(cl:defclass <CalibrationItem> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass CalibrationItem (<CalibrationItem>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CalibrationItem>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CalibrationItem)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<CalibrationItem> is deprecated: use kortex_driver-msg:CalibrationItem instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<CalibrationItem>)))
    "Constants for message type '<CalibrationItem>"
  '((:UNSPECIFIED_CALIBRATION_ITEM . 0)
    (:COGGING . 1)
    (:MAGNETIC . 2)
    (:MOTOR . 3)
    (:POSITION_RANGE . 4))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'CalibrationItem)))
    "Constants for message type 'CalibrationItem"
  '((:UNSPECIFIED_CALIBRATION_ITEM . 0)
    (:COGGING . 1)
    (:MAGNETIC . 2)
    (:MOTOR . 3)
    (:POSITION_RANGE . 4))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CalibrationItem>) ostream)
  "Serializes a message object of type '<CalibrationItem>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CalibrationItem>) istream)
  "Deserializes a message object of type '<CalibrationItem>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CalibrationItem>)))
  "Returns string type for a message object of type '<CalibrationItem>"
  "kortex_driver/CalibrationItem")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CalibrationItem)))
  "Returns string type for a message object of type 'CalibrationItem"
  "kortex_driver/CalibrationItem")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CalibrationItem>)))
  "Returns md5sum for a message object of type '<CalibrationItem>"
  "004ebe034d74d471c7b919bf63c0f51a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CalibrationItem)))
  "Returns md5sum for a message object of type 'CalibrationItem"
  "004ebe034d74d471c7b919bf63c0f51a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CalibrationItem>)))
  "Returns full string definition for message of type '<CalibrationItem>"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_CALIBRATION_ITEM = 0~%~%uint32 COGGING = 1~%~%uint32 MAGNETIC = 2~%~%uint32 MOTOR = 3~%~%uint32 POSITION_RANGE = 4~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CalibrationItem)))
  "Returns full string definition for message of type 'CalibrationItem"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_CALIBRATION_ITEM = 0~%~%uint32 COGGING = 1~%~%uint32 MAGNETIC = 2~%~%uint32 MOTOR = 3~%~%uint32 POSITION_RANGE = 4~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CalibrationItem>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CalibrationItem>))
  "Converts a ROS message object to a list"
  (cl:list 'CalibrationItem
))

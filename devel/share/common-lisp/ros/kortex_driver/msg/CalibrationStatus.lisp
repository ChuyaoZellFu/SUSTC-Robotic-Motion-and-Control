; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude CalibrationStatus.msg.html

(cl:defclass <CalibrationStatus> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass CalibrationStatus (<CalibrationStatus>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CalibrationStatus>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CalibrationStatus)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<CalibrationStatus> is deprecated: use kortex_driver-msg:CalibrationStatus instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<CalibrationStatus>)))
    "Constants for message type '<CalibrationStatus>"
  '((:UNSPECIFIED_CALIBRATION_STATUS . 0)
    (:NOT_CALIBRATED . 1)
    (:IN_PROGRESS . 2)
    (:CALIBRATED . 3)
    (:IN_FAULT . 4))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'CalibrationStatus)))
    "Constants for message type 'CalibrationStatus"
  '((:UNSPECIFIED_CALIBRATION_STATUS . 0)
    (:NOT_CALIBRATED . 1)
    (:IN_PROGRESS . 2)
    (:CALIBRATED . 3)
    (:IN_FAULT . 4))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CalibrationStatus>) ostream)
  "Serializes a message object of type '<CalibrationStatus>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CalibrationStatus>) istream)
  "Deserializes a message object of type '<CalibrationStatus>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CalibrationStatus>)))
  "Returns string type for a message object of type '<CalibrationStatus>"
  "kortex_driver/CalibrationStatus")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CalibrationStatus)))
  "Returns string type for a message object of type 'CalibrationStatus"
  "kortex_driver/CalibrationStatus")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CalibrationStatus>)))
  "Returns md5sum for a message object of type '<CalibrationStatus>"
  "a4d9ce00135ee251dc1d3b34f761f5f8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CalibrationStatus)))
  "Returns md5sum for a message object of type 'CalibrationStatus"
  "a4d9ce00135ee251dc1d3b34f761f5f8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CalibrationStatus>)))
  "Returns full string definition for message of type '<CalibrationStatus>"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_CALIBRATION_STATUS = 0~%~%uint32 NOT_CALIBRATED = 1~%~%uint32 IN_PROGRESS = 2~%~%uint32 CALIBRATED = 3~%~%uint32 IN_FAULT = 4~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CalibrationStatus)))
  "Returns full string definition for message of type 'CalibrationStatus"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_CALIBRATION_STATUS = 0~%~%uint32 NOT_CALIBRATED = 1~%~%uint32 IN_PROGRESS = 2~%~%uint32 CALIBRATED = 3~%~%uint32 IN_FAULT = 4~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CalibrationStatus>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CalibrationStatus>))
  "Converts a ROS message object to a list"
  (cl:list 'CalibrationStatus
))

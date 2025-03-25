; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude ServoingModeInformation.msg.html

(cl:defclass <ServoingModeInformation> (roslisp-msg-protocol:ros-message)
  ((servoing_mode
    :reader servoing_mode
    :initarg :servoing_mode
    :type cl:integer
    :initform 0))
)

(cl:defclass ServoingModeInformation (<ServoingModeInformation>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ServoingModeInformation>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ServoingModeInformation)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<ServoingModeInformation> is deprecated: use kortex_driver-msg:ServoingModeInformation instead.")))

(cl:ensure-generic-function 'servoing_mode-val :lambda-list '(m))
(cl:defmethod servoing_mode-val ((m <ServoingModeInformation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:servoing_mode-val is deprecated.  Use kortex_driver-msg:servoing_mode instead.")
  (servoing_mode m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ServoingModeInformation>) ostream)
  "Serializes a message object of type '<ServoingModeInformation>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'servoing_mode)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'servoing_mode)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'servoing_mode)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'servoing_mode)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ServoingModeInformation>) istream)
  "Deserializes a message object of type '<ServoingModeInformation>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'servoing_mode)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'servoing_mode)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'servoing_mode)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'servoing_mode)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ServoingModeInformation>)))
  "Returns string type for a message object of type '<ServoingModeInformation>"
  "kortex_driver/ServoingModeInformation")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ServoingModeInformation)))
  "Returns string type for a message object of type 'ServoingModeInformation"
  "kortex_driver/ServoingModeInformation")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ServoingModeInformation>)))
  "Returns md5sum for a message object of type '<ServoingModeInformation>"
  "fa12f11eee2104f3edd1a6487bc94db9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ServoingModeInformation)))
  "Returns md5sum for a message object of type 'ServoingModeInformation"
  "fa12f11eee2104f3edd1a6487bc94db9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ServoingModeInformation>)))
  "Returns full string definition for message of type '<ServoingModeInformation>"
  (cl:format cl:nil "~%uint32 servoing_mode~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ServoingModeInformation)))
  "Returns full string definition for message of type 'ServoingModeInformation"
  (cl:format cl:nil "~%uint32 servoing_mode~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ServoingModeInformation>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ServoingModeInformation>))
  "Converts a ROS message object to a list"
  (cl:list 'ServoingModeInformation
    (cl:cons ':servoing_mode (servoing_mode msg))
))

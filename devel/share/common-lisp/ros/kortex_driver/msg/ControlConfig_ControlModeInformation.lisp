; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude ControlConfig_ControlModeInformation.msg.html

(cl:defclass <ControlConfig_ControlModeInformation> (roslisp-msg-protocol:ros-message)
  ((control_mode
    :reader control_mode
    :initarg :control_mode
    :type cl:integer
    :initform 0))
)

(cl:defclass ControlConfig_ControlModeInformation (<ControlConfig_ControlModeInformation>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ControlConfig_ControlModeInformation>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ControlConfig_ControlModeInformation)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<ControlConfig_ControlModeInformation> is deprecated: use kortex_driver-msg:ControlConfig_ControlModeInformation instead.")))

(cl:ensure-generic-function 'control_mode-val :lambda-list '(m))
(cl:defmethod control_mode-val ((m <ControlConfig_ControlModeInformation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:control_mode-val is deprecated.  Use kortex_driver-msg:control_mode instead.")
  (control_mode m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ControlConfig_ControlModeInformation>) ostream)
  "Serializes a message object of type '<ControlConfig_ControlModeInformation>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'control_mode)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'control_mode)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'control_mode)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'control_mode)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ControlConfig_ControlModeInformation>) istream)
  "Deserializes a message object of type '<ControlConfig_ControlModeInformation>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'control_mode)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'control_mode)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'control_mode)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'control_mode)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ControlConfig_ControlModeInformation>)))
  "Returns string type for a message object of type '<ControlConfig_ControlModeInformation>"
  "kortex_driver/ControlConfig_ControlModeInformation")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ControlConfig_ControlModeInformation)))
  "Returns string type for a message object of type 'ControlConfig_ControlModeInformation"
  "kortex_driver/ControlConfig_ControlModeInformation")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ControlConfig_ControlModeInformation>)))
  "Returns md5sum for a message object of type '<ControlConfig_ControlModeInformation>"
  "c6c18afc4d9609a0ad8ceeb45431dcd5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ControlConfig_ControlModeInformation)))
  "Returns md5sum for a message object of type 'ControlConfig_ControlModeInformation"
  "c6c18afc4d9609a0ad8ceeb45431dcd5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ControlConfig_ControlModeInformation>)))
  "Returns full string definition for message of type '<ControlConfig_ControlModeInformation>"
  (cl:format cl:nil "~%uint32 control_mode~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ControlConfig_ControlModeInformation)))
  "Returns full string definition for message of type 'ControlConfig_ControlModeInformation"
  (cl:format cl:nil "~%uint32 control_mode~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ControlConfig_ControlModeInformation>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ControlConfig_ControlModeInformation>))
  "Converts a ROS message object to a list"
  (cl:list 'ControlConfig_ControlModeInformation
    (cl:cons ':control_mode (control_mode msg))
))

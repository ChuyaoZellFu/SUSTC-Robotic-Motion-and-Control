; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude Base_ControlModeInformation.msg.html

(cl:defclass <Base_ControlModeInformation> (roslisp-msg-protocol:ros-message)
  ((mode
    :reader mode
    :initarg :mode
    :type cl:integer
    :initform 0))
)

(cl:defclass Base_ControlModeInformation (<Base_ControlModeInformation>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Base_ControlModeInformation>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Base_ControlModeInformation)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<Base_ControlModeInformation> is deprecated: use kortex_driver-msg:Base_ControlModeInformation instead.")))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <Base_ControlModeInformation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:mode-val is deprecated.  Use kortex_driver-msg:mode instead.")
  (mode m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Base_ControlModeInformation>) ostream)
  "Serializes a message object of type '<Base_ControlModeInformation>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'mode)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'mode)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'mode)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Base_ControlModeInformation>) istream)
  "Deserializes a message object of type '<Base_ControlModeInformation>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'mode)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'mode)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'mode)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Base_ControlModeInformation>)))
  "Returns string type for a message object of type '<Base_ControlModeInformation>"
  "kortex_driver/Base_ControlModeInformation")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Base_ControlModeInformation)))
  "Returns string type for a message object of type 'Base_ControlModeInformation"
  "kortex_driver/Base_ControlModeInformation")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Base_ControlModeInformation>)))
  "Returns md5sum for a message object of type '<Base_ControlModeInformation>"
  "890d2fbf5b6e45074133a9b18a5f56b6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Base_ControlModeInformation)))
  "Returns md5sum for a message object of type 'Base_ControlModeInformation"
  "890d2fbf5b6e45074133a9b18a5f56b6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Base_ControlModeInformation>)))
  "Returns full string definition for message of type '<Base_ControlModeInformation>"
  (cl:format cl:nil "~%uint32 mode~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Base_ControlModeInformation)))
  "Returns full string definition for message of type 'Base_ControlModeInformation"
  (cl:format cl:nil "~%uint32 mode~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Base_ControlModeInformation>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Base_ControlModeInformation>))
  "Converts a ROS message object to a list"
  (cl:list 'Base_ControlModeInformation
    (cl:cons ':mode (mode msg))
))

; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude CommandModeInformation.msg.html

(cl:defclass <CommandModeInformation> (roslisp-msg-protocol:ros-message)
  ((command_mode
    :reader command_mode
    :initarg :command_mode
    :type cl:integer
    :initform 0))
)

(cl:defclass CommandModeInformation (<CommandModeInformation>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CommandModeInformation>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CommandModeInformation)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<CommandModeInformation> is deprecated: use kortex_driver-msg:CommandModeInformation instead.")))

(cl:ensure-generic-function 'command_mode-val :lambda-list '(m))
(cl:defmethod command_mode-val ((m <CommandModeInformation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:command_mode-val is deprecated.  Use kortex_driver-msg:command_mode instead.")
  (command_mode m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CommandModeInformation>) ostream)
  "Serializes a message object of type '<CommandModeInformation>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'command_mode)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'command_mode)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'command_mode)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'command_mode)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CommandModeInformation>) istream)
  "Deserializes a message object of type '<CommandModeInformation>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'command_mode)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'command_mode)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'command_mode)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'command_mode)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CommandModeInformation>)))
  "Returns string type for a message object of type '<CommandModeInformation>"
  "kortex_driver/CommandModeInformation")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CommandModeInformation)))
  "Returns string type for a message object of type 'CommandModeInformation"
  "kortex_driver/CommandModeInformation")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CommandModeInformation>)))
  "Returns md5sum for a message object of type '<CommandModeInformation>"
  "7d0d1fb206a87d77cdd9700eb23ea5cd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CommandModeInformation)))
  "Returns md5sum for a message object of type 'CommandModeInformation"
  "7d0d1fb206a87d77cdd9700eb23ea5cd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CommandModeInformation>)))
  "Returns full string definition for message of type '<CommandModeInformation>"
  (cl:format cl:nil "~%uint32 command_mode~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CommandModeInformation)))
  "Returns full string definition for message of type 'CommandModeInformation"
  (cl:format cl:nil "~%uint32 command_mode~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CommandModeInformation>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CommandModeInformation>))
  "Converts a ROS message object to a list"
  (cl:list 'CommandModeInformation
    (cl:cons ':command_mode (command_mode msg))
))

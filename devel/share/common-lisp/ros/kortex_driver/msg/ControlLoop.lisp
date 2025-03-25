; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude ControlLoop.msg.html

(cl:defclass <ControlLoop> (roslisp-msg-protocol:ros-message)
  ((control_loop
    :reader control_loop
    :initarg :control_loop
    :type cl:integer
    :initform 0))
)

(cl:defclass ControlLoop (<ControlLoop>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ControlLoop>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ControlLoop)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<ControlLoop> is deprecated: use kortex_driver-msg:ControlLoop instead.")))

(cl:ensure-generic-function 'control_loop-val :lambda-list '(m))
(cl:defmethod control_loop-val ((m <ControlLoop>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:control_loop-val is deprecated.  Use kortex_driver-msg:control_loop instead.")
  (control_loop m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ControlLoop>) ostream)
  "Serializes a message object of type '<ControlLoop>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'control_loop)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'control_loop)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'control_loop)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'control_loop)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ControlLoop>) istream)
  "Deserializes a message object of type '<ControlLoop>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'control_loop)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'control_loop)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'control_loop)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'control_loop)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ControlLoop>)))
  "Returns string type for a message object of type '<ControlLoop>"
  "kortex_driver/ControlLoop")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ControlLoop)))
  "Returns string type for a message object of type 'ControlLoop"
  "kortex_driver/ControlLoop")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ControlLoop>)))
  "Returns md5sum for a message object of type '<ControlLoop>"
  "26936d7a4ae5672d376ef90225be07d4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ControlLoop)))
  "Returns md5sum for a message object of type 'ControlLoop"
  "26936d7a4ae5672d376ef90225be07d4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ControlLoop>)))
  "Returns full string definition for message of type '<ControlLoop>"
  (cl:format cl:nil "~%uint32 control_loop~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ControlLoop)))
  "Returns full string definition for message of type 'ControlLoop"
  (cl:format cl:nil "~%uint32 control_loop~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ControlLoop>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ControlLoop>))
  "Converts a ROS message object to a list"
  (cl:list 'ControlLoop
    (cl:cons ':control_loop (control_loop msg))
))

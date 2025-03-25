; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude Delay.msg.html

(cl:defclass <Delay> (roslisp-msg-protocol:ros-message)
  ((duration
    :reader duration
    :initarg :duration
    :type cl:integer
    :initform 0))
)

(cl:defclass Delay (<Delay>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Delay>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Delay)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<Delay> is deprecated: use kortex_driver-msg:Delay instead.")))

(cl:ensure-generic-function 'duration-val :lambda-list '(m))
(cl:defmethod duration-val ((m <Delay>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:duration-val is deprecated.  Use kortex_driver-msg:duration instead.")
  (duration m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Delay>) ostream)
  "Serializes a message object of type '<Delay>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'duration)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'duration)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'duration)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'duration)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Delay>) istream)
  "Deserializes a message object of type '<Delay>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'duration)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'duration)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'duration)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'duration)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Delay>)))
  "Returns string type for a message object of type '<Delay>"
  "kortex_driver/Delay")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Delay)))
  "Returns string type for a message object of type 'Delay"
  "kortex_driver/Delay")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Delay>)))
  "Returns md5sum for a message object of type '<Delay>"
  "eea8d32b4658822ac77c4c94fbbb5f2b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Delay)))
  "Returns md5sum for a message object of type 'Delay"
  "eea8d32b4658822ac77c4c94fbbb5f2b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Delay>)))
  "Returns full string definition for message of type '<Delay>"
  (cl:format cl:nil "~%uint32 duration~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Delay)))
  "Returns full string definition for message of type 'Delay"
  (cl:format cl:nil "~%uint32 duration~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Delay>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Delay>))
  "Converts a ROS message object to a list"
  (cl:list 'Delay
    (cl:cons ':duration (duration msg))
))

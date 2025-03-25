; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude TwistCommand.msg.html

(cl:defclass <TwistCommand> (roslisp-msg-protocol:ros-message)
  ((reference_frame
    :reader reference_frame
    :initarg :reference_frame
    :type cl:integer
    :initform 0)
   (twist
    :reader twist
    :initarg :twist
    :type kortex_driver-msg:Twist
    :initform (cl:make-instance 'kortex_driver-msg:Twist))
   (duration
    :reader duration
    :initarg :duration
    :type cl:integer
    :initform 0))
)

(cl:defclass TwistCommand (<TwistCommand>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TwistCommand>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TwistCommand)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<TwistCommand> is deprecated: use kortex_driver-msg:TwistCommand instead.")))

(cl:ensure-generic-function 'reference_frame-val :lambda-list '(m))
(cl:defmethod reference_frame-val ((m <TwistCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:reference_frame-val is deprecated.  Use kortex_driver-msg:reference_frame instead.")
  (reference_frame m))

(cl:ensure-generic-function 'twist-val :lambda-list '(m))
(cl:defmethod twist-val ((m <TwistCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:twist-val is deprecated.  Use kortex_driver-msg:twist instead.")
  (twist m))

(cl:ensure-generic-function 'duration-val :lambda-list '(m))
(cl:defmethod duration-val ((m <TwistCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:duration-val is deprecated.  Use kortex_driver-msg:duration instead.")
  (duration m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TwistCommand>) ostream)
  "Serializes a message object of type '<TwistCommand>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'reference_frame)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'reference_frame)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'reference_frame)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'reference_frame)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'twist) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'duration)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'duration)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'duration)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'duration)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TwistCommand>) istream)
  "Deserializes a message object of type '<TwistCommand>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'reference_frame)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'reference_frame)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'reference_frame)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'reference_frame)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'twist) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'duration)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'duration)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'duration)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'duration)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TwistCommand>)))
  "Returns string type for a message object of type '<TwistCommand>"
  "kortex_driver/TwistCommand")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TwistCommand)))
  "Returns string type for a message object of type 'TwistCommand"
  "kortex_driver/TwistCommand")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TwistCommand>)))
  "Returns md5sum for a message object of type '<TwistCommand>"
  "16dcfd20a022a10eea1f05e5a9cbb18a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TwistCommand)))
  "Returns md5sum for a message object of type 'TwistCommand"
  "16dcfd20a022a10eea1f05e5a9cbb18a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TwistCommand>)))
  "Returns full string definition for message of type '<TwistCommand>"
  (cl:format cl:nil "~%uint32 reference_frame~%Twist twist~%uint32 duration~%================================================================================~%MSG: kortex_driver/Twist~%~%float32 linear_x~%float32 linear_y~%float32 linear_z~%float32 angular_x~%float32 angular_y~%float32 angular_z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TwistCommand)))
  "Returns full string definition for message of type 'TwistCommand"
  (cl:format cl:nil "~%uint32 reference_frame~%Twist twist~%uint32 duration~%================================================================================~%MSG: kortex_driver/Twist~%~%float32 linear_x~%float32 linear_y~%float32 linear_z~%float32 angular_x~%float32 angular_y~%float32 angular_z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TwistCommand>))
  (cl:+ 0
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'twist))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TwistCommand>))
  "Converts a ROS message object to a list"
  (cl:list 'TwistCommand
    (cl:cons ':reference_frame (reference_frame msg))
    (cl:cons ':twist (twist msg))
    (cl:cons ':duration (duration msg))
))

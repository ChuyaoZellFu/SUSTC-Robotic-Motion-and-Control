; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude LinearTwist.msg.html

(cl:defclass <LinearTwist> (roslisp-msg-protocol:ros-message)
  ((linear
    :reader linear
    :initarg :linear
    :type cl:float
    :initform 0.0))
)

(cl:defclass LinearTwist (<LinearTwist>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LinearTwist>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LinearTwist)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<LinearTwist> is deprecated: use kortex_driver-msg:LinearTwist instead.")))

(cl:ensure-generic-function 'linear-val :lambda-list '(m))
(cl:defmethod linear-val ((m <LinearTwist>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:linear-val is deprecated.  Use kortex_driver-msg:linear instead.")
  (linear m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LinearTwist>) ostream)
  "Serializes a message object of type '<LinearTwist>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'linear))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LinearTwist>) istream)
  "Deserializes a message object of type '<LinearTwist>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'linear) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LinearTwist>)))
  "Returns string type for a message object of type '<LinearTwist>"
  "kortex_driver/LinearTwist")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LinearTwist)))
  "Returns string type for a message object of type 'LinearTwist"
  "kortex_driver/LinearTwist")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LinearTwist>)))
  "Returns md5sum for a message object of type '<LinearTwist>"
  "aeedbad760b28cbccafc1668ae4ed22c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LinearTwist)))
  "Returns md5sum for a message object of type 'LinearTwist"
  "aeedbad760b28cbccafc1668ae4ed22c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LinearTwist>)))
  "Returns full string definition for message of type '<LinearTwist>"
  (cl:format cl:nil "~%float32 linear~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LinearTwist)))
  "Returns full string definition for message of type 'LinearTwist"
  (cl:format cl:nil "~%float32 linear~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LinearTwist>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LinearTwist>))
  "Converts a ROS message object to a list"
  (cl:list 'LinearTwist
    (cl:cons ':linear (linear msg))
))

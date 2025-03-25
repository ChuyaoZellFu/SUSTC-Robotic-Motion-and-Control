; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude AxisPosition.msg.html

(cl:defclass <AxisPosition> (roslisp-msg-protocol:ros-message)
  ((position
    :reader position
    :initarg :position
    :type cl:float
    :initform 0.0))
)

(cl:defclass AxisPosition (<AxisPosition>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AxisPosition>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AxisPosition)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<AxisPosition> is deprecated: use kortex_driver-msg:AxisPosition instead.")))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <AxisPosition>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:position-val is deprecated.  Use kortex_driver-msg:position instead.")
  (position m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AxisPosition>) ostream)
  "Serializes a message object of type '<AxisPosition>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'position))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AxisPosition>) istream)
  "Deserializes a message object of type '<AxisPosition>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'position) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AxisPosition>)))
  "Returns string type for a message object of type '<AxisPosition>"
  "kortex_driver/AxisPosition")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AxisPosition)))
  "Returns string type for a message object of type 'AxisPosition"
  "kortex_driver/AxisPosition")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AxisPosition>)))
  "Returns md5sum for a message object of type '<AxisPosition>"
  "e4e11b3af29ed247b2bd150d3f4540f9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AxisPosition)))
  "Returns md5sum for a message object of type 'AxisPosition"
  "e4e11b3af29ed247b2bd150d3f4540f9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AxisPosition>)))
  "Returns full string definition for message of type '<AxisPosition>"
  (cl:format cl:nil "~%float32 position~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AxisPosition)))
  "Returns full string definition for message of type 'AxisPosition"
  (cl:format cl:nil "~%float32 position~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AxisPosition>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AxisPosition>))
  "Converts a ROS message object to a list"
  (cl:list 'AxisPosition
    (cl:cons ':position (position msg))
))

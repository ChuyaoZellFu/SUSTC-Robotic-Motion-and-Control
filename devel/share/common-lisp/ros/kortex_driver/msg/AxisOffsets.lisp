; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude AxisOffsets.msg.html

(cl:defclass <AxisOffsets> (roslisp-msg-protocol:ros-message)
  ((absolute_offset
    :reader absolute_offset
    :initarg :absolute_offset
    :type cl:float
    :initform 0.0)
   (relative_offset
    :reader relative_offset
    :initarg :relative_offset
    :type cl:float
    :initform 0.0))
)

(cl:defclass AxisOffsets (<AxisOffsets>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AxisOffsets>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AxisOffsets)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<AxisOffsets> is deprecated: use kortex_driver-msg:AxisOffsets instead.")))

(cl:ensure-generic-function 'absolute_offset-val :lambda-list '(m))
(cl:defmethod absolute_offset-val ((m <AxisOffsets>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:absolute_offset-val is deprecated.  Use kortex_driver-msg:absolute_offset instead.")
  (absolute_offset m))

(cl:ensure-generic-function 'relative_offset-val :lambda-list '(m))
(cl:defmethod relative_offset-val ((m <AxisOffsets>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:relative_offset-val is deprecated.  Use kortex_driver-msg:relative_offset instead.")
  (relative_offset m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AxisOffsets>) ostream)
  "Serializes a message object of type '<AxisOffsets>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'absolute_offset))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'relative_offset))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AxisOffsets>) istream)
  "Deserializes a message object of type '<AxisOffsets>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'absolute_offset) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'relative_offset) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AxisOffsets>)))
  "Returns string type for a message object of type '<AxisOffsets>"
  "kortex_driver/AxisOffsets")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AxisOffsets)))
  "Returns string type for a message object of type 'AxisOffsets"
  "kortex_driver/AxisOffsets")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AxisOffsets>)))
  "Returns md5sum for a message object of type '<AxisOffsets>"
  "2cf10c1327fbdc40ecc0a83625568f5b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AxisOffsets)))
  "Returns md5sum for a message object of type 'AxisOffsets"
  "2cf10c1327fbdc40ecc0a83625568f5b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AxisOffsets>)))
  "Returns full string definition for message of type '<AxisOffsets>"
  (cl:format cl:nil "~%float32 absolute_offset~%float32 relative_offset~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AxisOffsets)))
  "Returns full string definition for message of type 'AxisOffsets"
  (cl:format cl:nil "~%float32 absolute_offset~%float32 relative_offset~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AxisOffsets>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AxisOffsets>))
  "Converts a ROS message object to a list"
  (cl:list 'AxisOffsets
    (cl:cons ':absolute_offset (absolute_offset msg))
    (cl:cons ':relative_offset (relative_offset msg))
))

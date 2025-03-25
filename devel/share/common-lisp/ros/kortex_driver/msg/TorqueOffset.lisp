; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude TorqueOffset.msg.html

(cl:defclass <TorqueOffset> (roslisp-msg-protocol:ros-message)
  ((torque_offset
    :reader torque_offset
    :initarg :torque_offset
    :type cl:float
    :initform 0.0))
)

(cl:defclass TorqueOffset (<TorqueOffset>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TorqueOffset>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TorqueOffset)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<TorqueOffset> is deprecated: use kortex_driver-msg:TorqueOffset instead.")))

(cl:ensure-generic-function 'torque_offset-val :lambda-list '(m))
(cl:defmethod torque_offset-val ((m <TorqueOffset>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:torque_offset-val is deprecated.  Use kortex_driver-msg:torque_offset instead.")
  (torque_offset m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TorqueOffset>) ostream)
  "Serializes a message object of type '<TorqueOffset>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'torque_offset))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TorqueOffset>) istream)
  "Deserializes a message object of type '<TorqueOffset>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'torque_offset) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TorqueOffset>)))
  "Returns string type for a message object of type '<TorqueOffset>"
  "kortex_driver/TorqueOffset")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TorqueOffset)))
  "Returns string type for a message object of type 'TorqueOffset"
  "kortex_driver/TorqueOffset")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TorqueOffset>)))
  "Returns md5sum for a message object of type '<TorqueOffset>"
  "0ac8f77f2604cb21bcfb1cb83c3c05e7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TorqueOffset)))
  "Returns md5sum for a message object of type 'TorqueOffset"
  "0ac8f77f2604cb21bcfb1cb83c3c05e7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TorqueOffset>)))
  "Returns full string definition for message of type '<TorqueOffset>"
  (cl:format cl:nil "~%float32 torque_offset~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TorqueOffset)))
  "Returns full string definition for message of type 'TorqueOffset"
  (cl:format cl:nil "~%float32 torque_offset~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TorqueOffset>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TorqueOffset>))
  "Converts a ROS message object to a list"
  (cl:list 'TorqueOffset
    (cl:cons ':torque_offset (torque_offset msg))
))

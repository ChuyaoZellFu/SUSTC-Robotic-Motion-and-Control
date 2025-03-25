; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude ChangeWrench.msg.html

(cl:defclass <ChangeWrench> (roslisp-msg-protocol:ros-message)
  ((force
    :reader force
    :initarg :force
    :type cl:float
    :initform 0.0)
   (torque
    :reader torque
    :initarg :torque
    :type cl:float
    :initform 0.0))
)

(cl:defclass ChangeWrench (<ChangeWrench>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ChangeWrench>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ChangeWrench)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<ChangeWrench> is deprecated: use kortex_driver-msg:ChangeWrench instead.")))

(cl:ensure-generic-function 'force-val :lambda-list '(m))
(cl:defmethod force-val ((m <ChangeWrench>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:force-val is deprecated.  Use kortex_driver-msg:force instead.")
  (force m))

(cl:ensure-generic-function 'torque-val :lambda-list '(m))
(cl:defmethod torque-val ((m <ChangeWrench>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:torque-val is deprecated.  Use kortex_driver-msg:torque instead.")
  (torque m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ChangeWrench>) ostream)
  "Serializes a message object of type '<ChangeWrench>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'force))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'torque))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ChangeWrench>) istream)
  "Deserializes a message object of type '<ChangeWrench>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'force) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'torque) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ChangeWrench>)))
  "Returns string type for a message object of type '<ChangeWrench>"
  "kortex_driver/ChangeWrench")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ChangeWrench)))
  "Returns string type for a message object of type 'ChangeWrench"
  "kortex_driver/ChangeWrench")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ChangeWrench>)))
  "Returns md5sum for a message object of type '<ChangeWrench>"
  "2afe6a3b4cba4f0de2ad91545187e9d1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ChangeWrench)))
  "Returns md5sum for a message object of type 'ChangeWrench"
  "2afe6a3b4cba4f0de2ad91545187e9d1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ChangeWrench>)))
  "Returns full string definition for message of type '<ChangeWrench>"
  (cl:format cl:nil "~%float32 force~%float32 torque~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ChangeWrench)))
  "Returns full string definition for message of type 'ChangeWrench"
  (cl:format cl:nil "~%float32 force~%float32 torque~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ChangeWrench>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ChangeWrench>))
  "Converts a ROS message object to a list"
  (cl:list 'ChangeWrench
    (cl:cons ':force (force msg))
    (cl:cons ':torque (torque msg))
))

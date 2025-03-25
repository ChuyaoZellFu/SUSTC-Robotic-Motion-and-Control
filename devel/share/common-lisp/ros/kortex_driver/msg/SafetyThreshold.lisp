; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude SafetyThreshold.msg.html

(cl:defclass <SafetyThreshold> (roslisp-msg-protocol:ros-message)
  ((handle
    :reader handle
    :initarg :handle
    :type kortex_driver-msg:SafetyHandle
    :initform (cl:make-instance 'kortex_driver-msg:SafetyHandle))
   (value
    :reader value
    :initarg :value
    :type cl:float
    :initform 0.0))
)

(cl:defclass SafetyThreshold (<SafetyThreshold>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SafetyThreshold>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SafetyThreshold)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<SafetyThreshold> is deprecated: use kortex_driver-msg:SafetyThreshold instead.")))

(cl:ensure-generic-function 'handle-val :lambda-list '(m))
(cl:defmethod handle-val ((m <SafetyThreshold>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:handle-val is deprecated.  Use kortex_driver-msg:handle instead.")
  (handle m))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <SafetyThreshold>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:value-val is deprecated.  Use kortex_driver-msg:value instead.")
  (value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SafetyThreshold>) ostream)
  "Serializes a message object of type '<SafetyThreshold>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'handle) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'value))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SafetyThreshold>) istream)
  "Deserializes a message object of type '<SafetyThreshold>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'handle) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'value) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SafetyThreshold>)))
  "Returns string type for a message object of type '<SafetyThreshold>"
  "kortex_driver/SafetyThreshold")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SafetyThreshold)))
  "Returns string type for a message object of type 'SafetyThreshold"
  "kortex_driver/SafetyThreshold")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SafetyThreshold>)))
  "Returns md5sum for a message object of type '<SafetyThreshold>"
  "6785d727fa30955d28c049a2ab6d6c82")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SafetyThreshold)))
  "Returns md5sum for a message object of type 'SafetyThreshold"
  "6785d727fa30955d28c049a2ab6d6c82")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SafetyThreshold>)))
  "Returns full string definition for message of type '<SafetyThreshold>"
  (cl:format cl:nil "~%SafetyHandle handle~%float32 value~%================================================================================~%MSG: kortex_driver/SafetyHandle~%~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SafetyThreshold)))
  "Returns full string definition for message of type 'SafetyThreshold"
  (cl:format cl:nil "~%SafetyHandle handle~%float32 value~%================================================================================~%MSG: kortex_driver/SafetyHandle~%~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SafetyThreshold>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'handle))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SafetyThreshold>))
  "Converts a ROS message object to a list"
  (cl:list 'SafetyThreshold
    (cl:cons ':handle (handle msg))
    (cl:cons ':value (value msg))
))

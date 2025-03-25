; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude PowerOnSelfTestResult.msg.html

(cl:defclass <PowerOnSelfTestResult> (roslisp-msg-protocol:ros-message)
  ((power_on_self_test_result
    :reader power_on_self_test_result
    :initarg :power_on_self_test_result
    :type cl:integer
    :initform 0))
)

(cl:defclass PowerOnSelfTestResult (<PowerOnSelfTestResult>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PowerOnSelfTestResult>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PowerOnSelfTestResult)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<PowerOnSelfTestResult> is deprecated: use kortex_driver-msg:PowerOnSelfTestResult instead.")))

(cl:ensure-generic-function 'power_on_self_test_result-val :lambda-list '(m))
(cl:defmethod power_on_self_test_result-val ((m <PowerOnSelfTestResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:power_on_self_test_result-val is deprecated.  Use kortex_driver-msg:power_on_self_test_result instead.")
  (power_on_self_test_result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PowerOnSelfTestResult>) ostream)
  "Serializes a message object of type '<PowerOnSelfTestResult>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'power_on_self_test_result)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'power_on_self_test_result)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'power_on_self_test_result)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'power_on_self_test_result)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PowerOnSelfTestResult>) istream)
  "Deserializes a message object of type '<PowerOnSelfTestResult>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'power_on_self_test_result)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'power_on_self_test_result)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'power_on_self_test_result)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'power_on_self_test_result)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PowerOnSelfTestResult>)))
  "Returns string type for a message object of type '<PowerOnSelfTestResult>"
  "kortex_driver/PowerOnSelfTestResult")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PowerOnSelfTestResult)))
  "Returns string type for a message object of type 'PowerOnSelfTestResult"
  "kortex_driver/PowerOnSelfTestResult")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PowerOnSelfTestResult>)))
  "Returns md5sum for a message object of type '<PowerOnSelfTestResult>"
  "927efe338b454b142f376d1936670eb0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PowerOnSelfTestResult)))
  "Returns md5sum for a message object of type 'PowerOnSelfTestResult"
  "927efe338b454b142f376d1936670eb0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PowerOnSelfTestResult>)))
  "Returns full string definition for message of type '<PowerOnSelfTestResult>"
  (cl:format cl:nil "~%uint32 power_on_self_test_result~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PowerOnSelfTestResult)))
  "Returns full string definition for message of type 'PowerOnSelfTestResult"
  (cl:format cl:nil "~%uint32 power_on_self_test_result~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PowerOnSelfTestResult>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PowerOnSelfTestResult>))
  "Converts a ROS message object to a list"
  (cl:list 'PowerOnSelfTestResult
    (cl:cons ':power_on_self_test_result (power_on_self_test_result msg))
))

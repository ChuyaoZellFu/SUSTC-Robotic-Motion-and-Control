; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude SafetyConfigurationList.msg.html

(cl:defclass <SafetyConfigurationList> (roslisp-msg-protocol:ros-message)
  ((configuration
    :reader configuration
    :initarg :configuration
    :type (cl:vector kortex_driver-msg:SafetyConfiguration)
   :initform (cl:make-array 0 :element-type 'kortex_driver-msg:SafetyConfiguration :initial-element (cl:make-instance 'kortex_driver-msg:SafetyConfiguration))))
)

(cl:defclass SafetyConfigurationList (<SafetyConfigurationList>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SafetyConfigurationList>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SafetyConfigurationList)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<SafetyConfigurationList> is deprecated: use kortex_driver-msg:SafetyConfigurationList instead.")))

(cl:ensure-generic-function 'configuration-val :lambda-list '(m))
(cl:defmethod configuration-val ((m <SafetyConfigurationList>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:configuration-val is deprecated.  Use kortex_driver-msg:configuration instead.")
  (configuration m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SafetyConfigurationList>) ostream)
  "Serializes a message object of type '<SafetyConfigurationList>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'configuration))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'configuration))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SafetyConfigurationList>) istream)
  "Deserializes a message object of type '<SafetyConfigurationList>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'configuration) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'configuration)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'kortex_driver-msg:SafetyConfiguration))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SafetyConfigurationList>)))
  "Returns string type for a message object of type '<SafetyConfigurationList>"
  "kortex_driver/SafetyConfigurationList")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SafetyConfigurationList)))
  "Returns string type for a message object of type 'SafetyConfigurationList"
  "kortex_driver/SafetyConfigurationList")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SafetyConfigurationList>)))
  "Returns md5sum for a message object of type '<SafetyConfigurationList>"
  "8178a2a82961f46646c77423c6510e4d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SafetyConfigurationList)))
  "Returns md5sum for a message object of type 'SafetyConfigurationList"
  "8178a2a82961f46646c77423c6510e4d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SafetyConfigurationList>)))
  "Returns full string definition for message of type '<SafetyConfigurationList>"
  (cl:format cl:nil "~%SafetyConfiguration[] configuration~%================================================================================~%MSG: kortex_driver/SafetyConfiguration~%~%SafetyHandle handle~%float32 error_threshold~%float32 warning_threshold~%SafetyEnable enable~%================================================================================~%MSG: kortex_driver/SafetyHandle~%~%uint32 identifier~%================================================================================~%MSG: kortex_driver/SafetyEnable~%~%SafetyHandle handle~%bool enable~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SafetyConfigurationList)))
  "Returns full string definition for message of type 'SafetyConfigurationList"
  (cl:format cl:nil "~%SafetyConfiguration[] configuration~%================================================================================~%MSG: kortex_driver/SafetyConfiguration~%~%SafetyHandle handle~%float32 error_threshold~%float32 warning_threshold~%SafetyEnable enable~%================================================================================~%MSG: kortex_driver/SafetyHandle~%~%uint32 identifier~%================================================================================~%MSG: kortex_driver/SafetyEnable~%~%SafetyHandle handle~%bool enable~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SafetyConfigurationList>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'configuration) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SafetyConfigurationList>))
  "Converts a ROS message object to a list"
  (cl:list 'SafetyConfigurationList
    (cl:cons ':configuration (configuration msg))
))

; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude SafetyInformationList.msg.html

(cl:defclass <SafetyInformationList> (roslisp-msg-protocol:ros-message)
  ((information
    :reader information
    :initarg :information
    :type (cl:vector kortex_driver-msg:SafetyInformation)
   :initform (cl:make-array 0 :element-type 'kortex_driver-msg:SafetyInformation :initial-element (cl:make-instance 'kortex_driver-msg:SafetyInformation))))
)

(cl:defclass SafetyInformationList (<SafetyInformationList>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SafetyInformationList>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SafetyInformationList)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<SafetyInformationList> is deprecated: use kortex_driver-msg:SafetyInformationList instead.")))

(cl:ensure-generic-function 'information-val :lambda-list '(m))
(cl:defmethod information-val ((m <SafetyInformationList>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:information-val is deprecated.  Use kortex_driver-msg:information instead.")
  (information m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SafetyInformationList>) ostream)
  "Serializes a message object of type '<SafetyInformationList>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'information))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'information))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SafetyInformationList>) istream)
  "Deserializes a message object of type '<SafetyInformationList>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'information) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'information)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'kortex_driver-msg:SafetyInformation))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SafetyInformationList>)))
  "Returns string type for a message object of type '<SafetyInformationList>"
  "kortex_driver/SafetyInformationList")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SafetyInformationList)))
  "Returns string type for a message object of type 'SafetyInformationList"
  "kortex_driver/SafetyInformationList")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SafetyInformationList>)))
  "Returns md5sum for a message object of type '<SafetyInformationList>"
  "c4049b5c5ed2e1815e6545bc46f6cf34")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SafetyInformationList)))
  "Returns md5sum for a message object of type 'SafetyInformationList"
  "c4049b5c5ed2e1815e6545bc46f6cf34")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SafetyInformationList>)))
  "Returns full string definition for message of type '<SafetyInformationList>"
  (cl:format cl:nil "~%SafetyInformation[] information~%================================================================================~%MSG: kortex_driver/SafetyInformation~%~%SafetyHandle handle~%bool can_change_safety_state~%bool has_warning_threshold~%bool has_error_threshold~%uint32 limit_type~%float32 default_warning_threshold~%float32 default_error_threshold~%float32 upper_hard_limit~%float32 lower_hard_limit~%uint32 status~%uint32 unit~%================================================================================~%MSG: kortex_driver/SafetyHandle~%~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SafetyInformationList)))
  "Returns full string definition for message of type 'SafetyInformationList"
  (cl:format cl:nil "~%SafetyInformation[] information~%================================================================================~%MSG: kortex_driver/SafetyInformation~%~%SafetyHandle handle~%bool can_change_safety_state~%bool has_warning_threshold~%bool has_error_threshold~%uint32 limit_type~%float32 default_warning_threshold~%float32 default_error_threshold~%float32 upper_hard_limit~%float32 lower_hard_limit~%uint32 status~%uint32 unit~%================================================================================~%MSG: kortex_driver/SafetyHandle~%~%uint32 identifier~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SafetyInformationList>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'information) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SafetyInformationList>))
  "Converts a ROS message object to a list"
  (cl:list 'SafetyInformationList
    (cl:cons ':information (information msg))
))

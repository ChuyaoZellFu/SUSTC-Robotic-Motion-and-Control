; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude ControllerConfigurationList.msg.html

(cl:defclass <ControllerConfigurationList> (roslisp-msg-protocol:ros-message)
  ((controller_configurations
    :reader controller_configurations
    :initarg :controller_configurations
    :type (cl:vector kortex_driver-msg:ControllerConfiguration)
   :initform (cl:make-array 0 :element-type 'kortex_driver-msg:ControllerConfiguration :initial-element (cl:make-instance 'kortex_driver-msg:ControllerConfiguration))))
)

(cl:defclass ControllerConfigurationList (<ControllerConfigurationList>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ControllerConfigurationList>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ControllerConfigurationList)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<ControllerConfigurationList> is deprecated: use kortex_driver-msg:ControllerConfigurationList instead.")))

(cl:ensure-generic-function 'controller_configurations-val :lambda-list '(m))
(cl:defmethod controller_configurations-val ((m <ControllerConfigurationList>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:controller_configurations-val is deprecated.  Use kortex_driver-msg:controller_configurations instead.")
  (controller_configurations m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ControllerConfigurationList>) ostream)
  "Serializes a message object of type '<ControllerConfigurationList>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'controller_configurations))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'controller_configurations))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ControllerConfigurationList>) istream)
  "Deserializes a message object of type '<ControllerConfigurationList>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'controller_configurations) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'controller_configurations)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'kortex_driver-msg:ControllerConfiguration))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ControllerConfigurationList>)))
  "Returns string type for a message object of type '<ControllerConfigurationList>"
  "kortex_driver/ControllerConfigurationList")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ControllerConfigurationList)))
  "Returns string type for a message object of type 'ControllerConfigurationList"
  "kortex_driver/ControllerConfigurationList")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ControllerConfigurationList>)))
  "Returns md5sum for a message object of type '<ControllerConfigurationList>"
  "4e505f81204befaff98df48e637201d5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ControllerConfigurationList)))
  "Returns md5sum for a message object of type 'ControllerConfigurationList"
  "4e505f81204befaff98df48e637201d5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ControllerConfigurationList>)))
  "Returns full string definition for message of type '<ControllerConfigurationList>"
  (cl:format cl:nil "~%ControllerConfiguration[] controller_configurations~%================================================================================~%MSG: kortex_driver/ControllerConfiguration~%~%ControllerHandle handle~%string name~%MappingHandle active_mapping_handle~%string analog_input_identifier_enum~%string digital_input_identifier_enum~%================================================================================~%MSG: kortex_driver/ControllerHandle~%~%uint32 type~%uint32 controller_identifier~%================================================================================~%MSG: kortex_driver/MappingHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ControllerConfigurationList)))
  "Returns full string definition for message of type 'ControllerConfigurationList"
  (cl:format cl:nil "~%ControllerConfiguration[] controller_configurations~%================================================================================~%MSG: kortex_driver/ControllerConfiguration~%~%ControllerHandle handle~%string name~%MappingHandle active_mapping_handle~%string analog_input_identifier_enum~%string digital_input_identifier_enum~%================================================================================~%MSG: kortex_driver/ControllerHandle~%~%uint32 type~%uint32 controller_identifier~%================================================================================~%MSG: kortex_driver/MappingHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ControllerConfigurationList>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'controller_configurations) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ControllerConfigurationList>))
  "Converts a ROS message object to a list"
  (cl:list 'ControllerConfigurationList
    (cl:cons ':controller_configurations (controller_configurations msg))
))

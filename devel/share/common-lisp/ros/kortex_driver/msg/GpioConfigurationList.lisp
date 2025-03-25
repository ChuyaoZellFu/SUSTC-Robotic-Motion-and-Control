; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude GpioConfigurationList.msg.html

(cl:defclass <GpioConfigurationList> (roslisp-msg-protocol:ros-message)
  ((port_configurations
    :reader port_configurations
    :initarg :port_configurations
    :type (cl:vector kortex_driver-msg:Base_GpioConfiguration)
   :initform (cl:make-array 0 :element-type 'kortex_driver-msg:Base_GpioConfiguration :initial-element (cl:make-instance 'kortex_driver-msg:Base_GpioConfiguration))))
)

(cl:defclass GpioConfigurationList (<GpioConfigurationList>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GpioConfigurationList>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GpioConfigurationList)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<GpioConfigurationList> is deprecated: use kortex_driver-msg:GpioConfigurationList instead.")))

(cl:ensure-generic-function 'port_configurations-val :lambda-list '(m))
(cl:defmethod port_configurations-val ((m <GpioConfigurationList>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:port_configurations-val is deprecated.  Use kortex_driver-msg:port_configurations instead.")
  (port_configurations m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GpioConfigurationList>) ostream)
  "Serializes a message object of type '<GpioConfigurationList>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'port_configurations))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'port_configurations))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GpioConfigurationList>) istream)
  "Deserializes a message object of type '<GpioConfigurationList>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'port_configurations) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'port_configurations)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'kortex_driver-msg:Base_GpioConfiguration))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GpioConfigurationList>)))
  "Returns string type for a message object of type '<GpioConfigurationList>"
  "kortex_driver/GpioConfigurationList")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GpioConfigurationList)))
  "Returns string type for a message object of type 'GpioConfigurationList"
  "kortex_driver/GpioConfigurationList")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GpioConfigurationList>)))
  "Returns md5sum for a message object of type '<GpioConfigurationList>"
  "2692838fc0bc85259f7645a61387ad92")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GpioConfigurationList)))
  "Returns md5sum for a message object of type 'GpioConfigurationList"
  "2692838fc0bc85259f7645a61387ad92")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GpioConfigurationList>)))
  "Returns full string definition for message of type '<GpioConfigurationList>"
  (cl:format cl:nil "~%Base_GpioConfiguration[] port_configurations~%================================================================================~%MSG: kortex_driver/Base_GpioConfiguration~%~%uint32 port_number~%GpioPinConfiguration[] pin_configurations~%================================================================================~%MSG: kortex_driver/GpioPinConfiguration~%~%uint32 pin_id~%uint32 pin_property~%bool output_enable~%bool default_output_value~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GpioConfigurationList)))
  "Returns full string definition for message of type 'GpioConfigurationList"
  (cl:format cl:nil "~%Base_GpioConfiguration[] port_configurations~%================================================================================~%MSG: kortex_driver/Base_GpioConfiguration~%~%uint32 port_number~%GpioPinConfiguration[] pin_configurations~%================================================================================~%MSG: kortex_driver/GpioPinConfiguration~%~%uint32 pin_id~%uint32 pin_property~%bool output_enable~%bool default_output_value~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GpioConfigurationList>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'port_configurations) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GpioConfigurationList>))
  "Converts a ROS message object to a list"
  (cl:list 'GpioConfigurationList
    (cl:cons ':port_configurations (port_configurations msg))
))

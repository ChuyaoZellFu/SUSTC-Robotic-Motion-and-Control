; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude Base_GpioConfiguration.msg.html

(cl:defclass <Base_GpioConfiguration> (roslisp-msg-protocol:ros-message)
  ((port_number
    :reader port_number
    :initarg :port_number
    :type cl:integer
    :initform 0)
   (pin_configurations
    :reader pin_configurations
    :initarg :pin_configurations
    :type (cl:vector kortex_driver-msg:GpioPinConfiguration)
   :initform (cl:make-array 0 :element-type 'kortex_driver-msg:GpioPinConfiguration :initial-element (cl:make-instance 'kortex_driver-msg:GpioPinConfiguration))))
)

(cl:defclass Base_GpioConfiguration (<Base_GpioConfiguration>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Base_GpioConfiguration>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Base_GpioConfiguration)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<Base_GpioConfiguration> is deprecated: use kortex_driver-msg:Base_GpioConfiguration instead.")))

(cl:ensure-generic-function 'port_number-val :lambda-list '(m))
(cl:defmethod port_number-val ((m <Base_GpioConfiguration>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:port_number-val is deprecated.  Use kortex_driver-msg:port_number instead.")
  (port_number m))

(cl:ensure-generic-function 'pin_configurations-val :lambda-list '(m))
(cl:defmethod pin_configurations-val ((m <Base_GpioConfiguration>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:pin_configurations-val is deprecated.  Use kortex_driver-msg:pin_configurations instead.")
  (pin_configurations m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Base_GpioConfiguration>) ostream)
  "Serializes a message object of type '<Base_GpioConfiguration>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'port_number)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'port_number)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'port_number)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'port_number)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'pin_configurations))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'pin_configurations))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Base_GpioConfiguration>) istream)
  "Deserializes a message object of type '<Base_GpioConfiguration>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'port_number)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'port_number)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'port_number)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'port_number)) (cl:read-byte istream))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'pin_configurations) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'pin_configurations)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'kortex_driver-msg:GpioPinConfiguration))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Base_GpioConfiguration>)))
  "Returns string type for a message object of type '<Base_GpioConfiguration>"
  "kortex_driver/Base_GpioConfiguration")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Base_GpioConfiguration)))
  "Returns string type for a message object of type 'Base_GpioConfiguration"
  "kortex_driver/Base_GpioConfiguration")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Base_GpioConfiguration>)))
  "Returns md5sum for a message object of type '<Base_GpioConfiguration>"
  "92f1de7cd4a1d8641179ab50f182b3f3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Base_GpioConfiguration)))
  "Returns md5sum for a message object of type 'Base_GpioConfiguration"
  "92f1de7cd4a1d8641179ab50f182b3f3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Base_GpioConfiguration>)))
  "Returns full string definition for message of type '<Base_GpioConfiguration>"
  (cl:format cl:nil "~%uint32 port_number~%GpioPinConfiguration[] pin_configurations~%================================================================================~%MSG: kortex_driver/GpioPinConfiguration~%~%uint32 pin_id~%uint32 pin_property~%bool output_enable~%bool default_output_value~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Base_GpioConfiguration)))
  "Returns full string definition for message of type 'Base_GpioConfiguration"
  (cl:format cl:nil "~%uint32 port_number~%GpioPinConfiguration[] pin_configurations~%================================================================================~%MSG: kortex_driver/GpioPinConfiguration~%~%uint32 pin_id~%uint32 pin_property~%bool output_enable~%bool default_output_value~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Base_GpioConfiguration>))
  (cl:+ 0
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'pin_configurations) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Base_GpioConfiguration>))
  "Converts a ROS message object to a list"
  (cl:list 'Base_GpioConfiguration
    (cl:cons ':port_number (port_number msg))
    (cl:cons ':pin_configurations (pin_configurations msg))
))

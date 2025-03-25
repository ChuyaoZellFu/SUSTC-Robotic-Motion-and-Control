; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude GpioPinConfiguration.msg.html

(cl:defclass <GpioPinConfiguration> (roslisp-msg-protocol:ros-message)
  ((pin_id
    :reader pin_id
    :initarg :pin_id
    :type cl:integer
    :initform 0)
   (pin_property
    :reader pin_property
    :initarg :pin_property
    :type cl:integer
    :initform 0)
   (output_enable
    :reader output_enable
    :initarg :output_enable
    :type cl:boolean
    :initform cl:nil)
   (default_output_value
    :reader default_output_value
    :initarg :default_output_value
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass GpioPinConfiguration (<GpioPinConfiguration>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GpioPinConfiguration>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GpioPinConfiguration)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<GpioPinConfiguration> is deprecated: use kortex_driver-msg:GpioPinConfiguration instead.")))

(cl:ensure-generic-function 'pin_id-val :lambda-list '(m))
(cl:defmethod pin_id-val ((m <GpioPinConfiguration>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:pin_id-val is deprecated.  Use kortex_driver-msg:pin_id instead.")
  (pin_id m))

(cl:ensure-generic-function 'pin_property-val :lambda-list '(m))
(cl:defmethod pin_property-val ((m <GpioPinConfiguration>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:pin_property-val is deprecated.  Use kortex_driver-msg:pin_property instead.")
  (pin_property m))

(cl:ensure-generic-function 'output_enable-val :lambda-list '(m))
(cl:defmethod output_enable-val ((m <GpioPinConfiguration>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:output_enable-val is deprecated.  Use kortex_driver-msg:output_enable instead.")
  (output_enable m))

(cl:ensure-generic-function 'default_output_value-val :lambda-list '(m))
(cl:defmethod default_output_value-val ((m <GpioPinConfiguration>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:default_output_value-val is deprecated.  Use kortex_driver-msg:default_output_value instead.")
  (default_output_value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GpioPinConfiguration>) ostream)
  "Serializes a message object of type '<GpioPinConfiguration>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'pin_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'pin_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'pin_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'pin_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'pin_property)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'pin_property)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'pin_property)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'pin_property)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'output_enable) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'default_output_value) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GpioPinConfiguration>) istream)
  "Deserializes a message object of type '<GpioPinConfiguration>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'pin_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'pin_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'pin_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'pin_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'pin_property)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'pin_property)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'pin_property)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'pin_property)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'output_enable) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'default_output_value) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GpioPinConfiguration>)))
  "Returns string type for a message object of type '<GpioPinConfiguration>"
  "kortex_driver/GpioPinConfiguration")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GpioPinConfiguration)))
  "Returns string type for a message object of type 'GpioPinConfiguration"
  "kortex_driver/GpioPinConfiguration")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GpioPinConfiguration>)))
  "Returns md5sum for a message object of type '<GpioPinConfiguration>"
  "4b71615f4756e2920864ba411102db09")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GpioPinConfiguration)))
  "Returns md5sum for a message object of type 'GpioPinConfiguration"
  "4b71615f4756e2920864ba411102db09")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GpioPinConfiguration>)))
  "Returns full string definition for message of type '<GpioPinConfiguration>"
  (cl:format cl:nil "~%uint32 pin_id~%uint32 pin_property~%bool output_enable~%bool default_output_value~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GpioPinConfiguration)))
  "Returns full string definition for message of type 'GpioPinConfiguration"
  (cl:format cl:nil "~%uint32 pin_id~%uint32 pin_property~%bool output_enable~%bool default_output_value~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GpioPinConfiguration>))
  (cl:+ 0
     4
     4
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GpioPinConfiguration>))
  "Converts a ROS message object to a list"
  (cl:list 'GpioPinConfiguration
    (cl:cons ':pin_id (pin_id msg))
    (cl:cons ':pin_property (pin_property msg))
    (cl:cons ':output_enable (output_enable msg))
    (cl:cons ':default_output_value (default_output_value msg))
))

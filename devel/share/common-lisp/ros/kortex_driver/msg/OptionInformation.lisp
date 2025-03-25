; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude OptionInformation.msg.html

(cl:defclass <OptionInformation> (roslisp-msg-protocol:ros-message)
  ((sensor
    :reader sensor
    :initarg :sensor
    :type cl:integer
    :initform 0)
   (option
    :reader option
    :initarg :option
    :type cl:integer
    :initform 0)
   (supported
    :reader supported
    :initarg :supported
    :type cl:boolean
    :initform cl:nil)
   (read_only
    :reader read_only
    :initarg :read_only
    :type cl:boolean
    :initform cl:nil)
   (minimum
    :reader minimum
    :initarg :minimum
    :type cl:float
    :initform 0.0)
   (maximum
    :reader maximum
    :initarg :maximum
    :type cl:float
    :initform 0.0)
   (step
    :reader step
    :initarg :step
    :type cl:float
    :initform 0.0)
   (default_value
    :reader default_value
    :initarg :default_value
    :type cl:float
    :initform 0.0))
)

(cl:defclass OptionInformation (<OptionInformation>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <OptionInformation>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'OptionInformation)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<OptionInformation> is deprecated: use kortex_driver-msg:OptionInformation instead.")))

(cl:ensure-generic-function 'sensor-val :lambda-list '(m))
(cl:defmethod sensor-val ((m <OptionInformation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:sensor-val is deprecated.  Use kortex_driver-msg:sensor instead.")
  (sensor m))

(cl:ensure-generic-function 'option-val :lambda-list '(m))
(cl:defmethod option-val ((m <OptionInformation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:option-val is deprecated.  Use kortex_driver-msg:option instead.")
  (option m))

(cl:ensure-generic-function 'supported-val :lambda-list '(m))
(cl:defmethod supported-val ((m <OptionInformation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:supported-val is deprecated.  Use kortex_driver-msg:supported instead.")
  (supported m))

(cl:ensure-generic-function 'read_only-val :lambda-list '(m))
(cl:defmethod read_only-val ((m <OptionInformation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:read_only-val is deprecated.  Use kortex_driver-msg:read_only instead.")
  (read_only m))

(cl:ensure-generic-function 'minimum-val :lambda-list '(m))
(cl:defmethod minimum-val ((m <OptionInformation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:minimum-val is deprecated.  Use kortex_driver-msg:minimum instead.")
  (minimum m))

(cl:ensure-generic-function 'maximum-val :lambda-list '(m))
(cl:defmethod maximum-val ((m <OptionInformation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:maximum-val is deprecated.  Use kortex_driver-msg:maximum instead.")
  (maximum m))

(cl:ensure-generic-function 'step-val :lambda-list '(m))
(cl:defmethod step-val ((m <OptionInformation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:step-val is deprecated.  Use kortex_driver-msg:step instead.")
  (step m))

(cl:ensure-generic-function 'default_value-val :lambda-list '(m))
(cl:defmethod default_value-val ((m <OptionInformation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:default_value-val is deprecated.  Use kortex_driver-msg:default_value instead.")
  (default_value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <OptionInformation>) ostream)
  "Serializes a message object of type '<OptionInformation>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sensor)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'sensor)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'sensor)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'sensor)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'option)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'option)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'option)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'option)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'supported) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'read_only) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'minimum))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'maximum))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'step))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'default_value))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <OptionInformation>) istream)
  "Deserializes a message object of type '<OptionInformation>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sensor)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'sensor)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'sensor)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'sensor)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'option)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'option)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'option)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'option)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'supported) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'read_only) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'minimum) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'maximum) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'step) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'default_value) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<OptionInformation>)))
  "Returns string type for a message object of type '<OptionInformation>"
  "kortex_driver/OptionInformation")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'OptionInformation)))
  "Returns string type for a message object of type 'OptionInformation"
  "kortex_driver/OptionInformation")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<OptionInformation>)))
  "Returns md5sum for a message object of type '<OptionInformation>"
  "4a9212bde4d95812b702827b151ff2e7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'OptionInformation)))
  "Returns md5sum for a message object of type 'OptionInformation"
  "4a9212bde4d95812b702827b151ff2e7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<OptionInformation>)))
  "Returns full string definition for message of type '<OptionInformation>"
  (cl:format cl:nil "~%uint32 sensor~%uint32 option~%bool supported~%bool read_only~%float32 minimum~%float32 maximum~%float32 step~%float32 default_value~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'OptionInformation)))
  "Returns full string definition for message of type 'OptionInformation"
  (cl:format cl:nil "~%uint32 sensor~%uint32 option~%bool supported~%bool read_only~%float32 minimum~%float32 maximum~%float32 step~%float32 default_value~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <OptionInformation>))
  (cl:+ 0
     4
     4
     1
     1
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <OptionInformation>))
  "Converts a ROS message object to a list"
  (cl:list 'OptionInformation
    (cl:cons ':sensor (sensor msg))
    (cl:cons ':option (option msg))
    (cl:cons ':supported (supported msg))
    (cl:cons ':read_only (read_only msg))
    (cl:cons ':minimum (minimum msg))
    (cl:cons ':maximum (maximum msg))
    (cl:cons ':step (step msg))
    (cl:cons ':default_value (default_value msg))
))

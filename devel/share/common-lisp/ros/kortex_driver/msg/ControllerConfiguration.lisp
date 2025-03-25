; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude ControllerConfiguration.msg.html

(cl:defclass <ControllerConfiguration> (roslisp-msg-protocol:ros-message)
  ((handle
    :reader handle
    :initarg :handle
    :type kortex_driver-msg:ControllerHandle
    :initform (cl:make-instance 'kortex_driver-msg:ControllerHandle))
   (name
    :reader name
    :initarg :name
    :type cl:string
    :initform "")
   (active_mapping_handle
    :reader active_mapping_handle
    :initarg :active_mapping_handle
    :type kortex_driver-msg:MappingHandle
    :initform (cl:make-instance 'kortex_driver-msg:MappingHandle))
   (analog_input_identifier_enum
    :reader analog_input_identifier_enum
    :initarg :analog_input_identifier_enum
    :type cl:string
    :initform "")
   (digital_input_identifier_enum
    :reader digital_input_identifier_enum
    :initarg :digital_input_identifier_enum
    :type cl:string
    :initform ""))
)

(cl:defclass ControllerConfiguration (<ControllerConfiguration>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ControllerConfiguration>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ControllerConfiguration)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<ControllerConfiguration> is deprecated: use kortex_driver-msg:ControllerConfiguration instead.")))

(cl:ensure-generic-function 'handle-val :lambda-list '(m))
(cl:defmethod handle-val ((m <ControllerConfiguration>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:handle-val is deprecated.  Use kortex_driver-msg:handle instead.")
  (handle m))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <ControllerConfiguration>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:name-val is deprecated.  Use kortex_driver-msg:name instead.")
  (name m))

(cl:ensure-generic-function 'active_mapping_handle-val :lambda-list '(m))
(cl:defmethod active_mapping_handle-val ((m <ControllerConfiguration>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:active_mapping_handle-val is deprecated.  Use kortex_driver-msg:active_mapping_handle instead.")
  (active_mapping_handle m))

(cl:ensure-generic-function 'analog_input_identifier_enum-val :lambda-list '(m))
(cl:defmethod analog_input_identifier_enum-val ((m <ControllerConfiguration>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:analog_input_identifier_enum-val is deprecated.  Use kortex_driver-msg:analog_input_identifier_enum instead.")
  (analog_input_identifier_enum m))

(cl:ensure-generic-function 'digital_input_identifier_enum-val :lambda-list '(m))
(cl:defmethod digital_input_identifier_enum-val ((m <ControllerConfiguration>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:digital_input_identifier_enum-val is deprecated.  Use kortex_driver-msg:digital_input_identifier_enum instead.")
  (digital_input_identifier_enum m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ControllerConfiguration>) ostream)
  "Serializes a message object of type '<ControllerConfiguration>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'handle) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'active_mapping_handle) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'analog_input_identifier_enum))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'analog_input_identifier_enum))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'digital_input_identifier_enum))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'digital_input_identifier_enum))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ControllerConfiguration>) istream)
  "Deserializes a message object of type '<ControllerConfiguration>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'handle) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'active_mapping_handle) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'analog_input_identifier_enum) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'analog_input_identifier_enum) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'digital_input_identifier_enum) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'digital_input_identifier_enum) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ControllerConfiguration>)))
  "Returns string type for a message object of type '<ControllerConfiguration>"
  "kortex_driver/ControllerConfiguration")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ControllerConfiguration)))
  "Returns string type for a message object of type 'ControllerConfiguration"
  "kortex_driver/ControllerConfiguration")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ControllerConfiguration>)))
  "Returns md5sum for a message object of type '<ControllerConfiguration>"
  "0a2e41d50fc9c491b9a0c9000a90ca85")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ControllerConfiguration)))
  "Returns md5sum for a message object of type 'ControllerConfiguration"
  "0a2e41d50fc9c491b9a0c9000a90ca85")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ControllerConfiguration>)))
  "Returns full string definition for message of type '<ControllerConfiguration>"
  (cl:format cl:nil "~%ControllerHandle handle~%string name~%MappingHandle active_mapping_handle~%string analog_input_identifier_enum~%string digital_input_identifier_enum~%================================================================================~%MSG: kortex_driver/ControllerHandle~%~%uint32 type~%uint32 controller_identifier~%================================================================================~%MSG: kortex_driver/MappingHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ControllerConfiguration)))
  "Returns full string definition for message of type 'ControllerConfiguration"
  (cl:format cl:nil "~%ControllerHandle handle~%string name~%MappingHandle active_mapping_handle~%string analog_input_identifier_enum~%string digital_input_identifier_enum~%================================================================================~%MSG: kortex_driver/ControllerHandle~%~%uint32 type~%uint32 controller_identifier~%================================================================================~%MSG: kortex_driver/MappingHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ControllerConfiguration>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'handle))
     4 (cl:length (cl:slot-value msg 'name))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'active_mapping_handle))
     4 (cl:length (cl:slot-value msg 'analog_input_identifier_enum))
     4 (cl:length (cl:slot-value msg 'digital_input_identifier_enum))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ControllerConfiguration>))
  "Converts a ROS message object to a list"
  (cl:list 'ControllerConfiguration
    (cl:cons ':handle (handle msg))
    (cl:cons ':name (name msg))
    (cl:cons ':active_mapping_handle (active_mapping_handle msg))
    (cl:cons ':analog_input_identifier_enum (analog_input_identifier_enum msg))
    (cl:cons ':digital_input_identifier_enum (digital_input_identifier_enum msg))
))

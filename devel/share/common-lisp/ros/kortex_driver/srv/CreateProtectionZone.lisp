; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude CreateProtectionZone-request.msg.html

(cl:defclass <CreateProtectionZone-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:ProtectionZone
    :initform (cl:make-instance 'kortex_driver-msg:ProtectionZone)))
)

(cl:defclass CreateProtectionZone-request (<CreateProtectionZone-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CreateProtectionZone-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CreateProtectionZone-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<CreateProtectionZone-request> is deprecated: use kortex_driver-srv:CreateProtectionZone-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <CreateProtectionZone-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CreateProtectionZone-request>) ostream)
  "Serializes a message object of type '<CreateProtectionZone-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CreateProtectionZone-request>) istream)
  "Deserializes a message object of type '<CreateProtectionZone-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CreateProtectionZone-request>)))
  "Returns string type for a service object of type '<CreateProtectionZone-request>"
  "kortex_driver/CreateProtectionZoneRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CreateProtectionZone-request)))
  "Returns string type for a service object of type 'CreateProtectionZone-request"
  "kortex_driver/CreateProtectionZoneRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CreateProtectionZone-request>)))
  "Returns md5sum for a message object of type '<CreateProtectionZone-request>"
  "e5380d610764b507278413b8ea5bd27f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CreateProtectionZone-request)))
  "Returns md5sum for a message object of type 'CreateProtectionZone-request"
  "e5380d610764b507278413b8ea5bd27f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CreateProtectionZone-request>)))
  "Returns full string definition for message of type '<CreateProtectionZone-request>"
  (cl:format cl:nil "ProtectionZone input~%~%================================================================================~%MSG: kortex_driver/ProtectionZone~%~%ProtectionZoneHandle handle~%string name~%string application_data~%bool is_enabled~%ZoneShape shape~%CartesianLimitation[] limitations~%CartesianLimitation[] envelope_limitations~%================================================================================~%MSG: kortex_driver/ProtectionZoneHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/ZoneShape~%~%uint32 shape_type~%Point origin~%Base_RotationMatrix orientation~%float32[] dimensions~%float32 envelope_thickness~%================================================================================~%MSG: kortex_driver/Point~%~%float32 x~%float32 y~%float32 z~%================================================================================~%MSG: kortex_driver/Base_RotationMatrix~%~%Base_RotationMatrixRow row1~%Base_RotationMatrixRow row2~%Base_RotationMatrixRow row3~%================================================================================~%MSG: kortex_driver/Base_RotationMatrixRow~%~%float32 column1~%float32 column2~%float32 column3~%================================================================================~%MSG: kortex_driver/CartesianLimitation~%~%uint32 type~%float32 translation~%float32 orientation~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CreateProtectionZone-request)))
  "Returns full string definition for message of type 'CreateProtectionZone-request"
  (cl:format cl:nil "ProtectionZone input~%~%================================================================================~%MSG: kortex_driver/ProtectionZone~%~%ProtectionZoneHandle handle~%string name~%string application_data~%bool is_enabled~%ZoneShape shape~%CartesianLimitation[] limitations~%CartesianLimitation[] envelope_limitations~%================================================================================~%MSG: kortex_driver/ProtectionZoneHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/ZoneShape~%~%uint32 shape_type~%Point origin~%Base_RotationMatrix orientation~%float32[] dimensions~%float32 envelope_thickness~%================================================================================~%MSG: kortex_driver/Point~%~%float32 x~%float32 y~%float32 z~%================================================================================~%MSG: kortex_driver/Base_RotationMatrix~%~%Base_RotationMatrixRow row1~%Base_RotationMatrixRow row2~%Base_RotationMatrixRow row3~%================================================================================~%MSG: kortex_driver/Base_RotationMatrixRow~%~%float32 column1~%float32 column2~%float32 column3~%================================================================================~%MSG: kortex_driver/CartesianLimitation~%~%uint32 type~%float32 translation~%float32 orientation~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CreateProtectionZone-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CreateProtectionZone-request>))
  "Converts a ROS message object to a list"
  (cl:list 'CreateProtectionZone-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude CreateProtectionZone-response.msg.html

(cl:defclass <CreateProtectionZone-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:ProtectionZoneHandle
    :initform (cl:make-instance 'kortex_driver-msg:ProtectionZoneHandle)))
)

(cl:defclass CreateProtectionZone-response (<CreateProtectionZone-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CreateProtectionZone-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CreateProtectionZone-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<CreateProtectionZone-response> is deprecated: use kortex_driver-srv:CreateProtectionZone-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <CreateProtectionZone-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CreateProtectionZone-response>) ostream)
  "Serializes a message object of type '<CreateProtectionZone-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CreateProtectionZone-response>) istream)
  "Deserializes a message object of type '<CreateProtectionZone-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CreateProtectionZone-response>)))
  "Returns string type for a service object of type '<CreateProtectionZone-response>"
  "kortex_driver/CreateProtectionZoneResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CreateProtectionZone-response)))
  "Returns string type for a service object of type 'CreateProtectionZone-response"
  "kortex_driver/CreateProtectionZoneResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CreateProtectionZone-response>)))
  "Returns md5sum for a message object of type '<CreateProtectionZone-response>"
  "e5380d610764b507278413b8ea5bd27f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CreateProtectionZone-response)))
  "Returns md5sum for a message object of type 'CreateProtectionZone-response"
  "e5380d610764b507278413b8ea5bd27f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CreateProtectionZone-response>)))
  "Returns full string definition for message of type '<CreateProtectionZone-response>"
  (cl:format cl:nil "ProtectionZoneHandle output~%~%================================================================================~%MSG: kortex_driver/ProtectionZoneHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CreateProtectionZone-response)))
  "Returns full string definition for message of type 'CreateProtectionZone-response"
  (cl:format cl:nil "ProtectionZoneHandle output~%~%================================================================================~%MSG: kortex_driver/ProtectionZoneHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CreateProtectionZone-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CreateProtectionZone-response>))
  "Converts a ROS message object to a list"
  (cl:list 'CreateProtectionZone-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'CreateProtectionZone)))
  'CreateProtectionZone-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'CreateProtectionZone)))
  'CreateProtectionZone-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CreateProtectionZone)))
  "Returns string type for a service object of type '<CreateProtectionZone>"
  "kortex_driver/CreateProtectionZone")
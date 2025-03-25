; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude ReadAllProtectionZones-request.msg.html

(cl:defclass <ReadAllProtectionZones-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass ReadAllProtectionZones-request (<ReadAllProtectionZones-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ReadAllProtectionZones-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ReadAllProtectionZones-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<ReadAllProtectionZones-request> is deprecated: use kortex_driver-srv:ReadAllProtectionZones-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <ReadAllProtectionZones-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ReadAllProtectionZones-request>) ostream)
  "Serializes a message object of type '<ReadAllProtectionZones-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ReadAllProtectionZones-request>) istream)
  "Deserializes a message object of type '<ReadAllProtectionZones-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ReadAllProtectionZones-request>)))
  "Returns string type for a service object of type '<ReadAllProtectionZones-request>"
  "kortex_driver/ReadAllProtectionZonesRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ReadAllProtectionZones-request)))
  "Returns string type for a service object of type 'ReadAllProtectionZones-request"
  "kortex_driver/ReadAllProtectionZonesRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ReadAllProtectionZones-request>)))
  "Returns md5sum for a message object of type '<ReadAllProtectionZones-request>"
  "26bc18c08c1b8389444ea1d92e6c8745")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ReadAllProtectionZones-request)))
  "Returns md5sum for a message object of type 'ReadAllProtectionZones-request"
  "26bc18c08c1b8389444ea1d92e6c8745")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ReadAllProtectionZones-request>)))
  "Returns full string definition for message of type '<ReadAllProtectionZones-request>"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ReadAllProtectionZones-request)))
  "Returns full string definition for message of type 'ReadAllProtectionZones-request"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ReadAllProtectionZones-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ReadAllProtectionZones-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ReadAllProtectionZones-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude ReadAllProtectionZones-response.msg.html

(cl:defclass <ReadAllProtectionZones-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:ProtectionZoneList
    :initform (cl:make-instance 'kortex_driver-msg:ProtectionZoneList)))
)

(cl:defclass ReadAllProtectionZones-response (<ReadAllProtectionZones-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ReadAllProtectionZones-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ReadAllProtectionZones-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<ReadAllProtectionZones-response> is deprecated: use kortex_driver-srv:ReadAllProtectionZones-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <ReadAllProtectionZones-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ReadAllProtectionZones-response>) ostream)
  "Serializes a message object of type '<ReadAllProtectionZones-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ReadAllProtectionZones-response>) istream)
  "Deserializes a message object of type '<ReadAllProtectionZones-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ReadAllProtectionZones-response>)))
  "Returns string type for a service object of type '<ReadAllProtectionZones-response>"
  "kortex_driver/ReadAllProtectionZonesResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ReadAllProtectionZones-response)))
  "Returns string type for a service object of type 'ReadAllProtectionZones-response"
  "kortex_driver/ReadAllProtectionZonesResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ReadAllProtectionZones-response>)))
  "Returns md5sum for a message object of type '<ReadAllProtectionZones-response>"
  "26bc18c08c1b8389444ea1d92e6c8745")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ReadAllProtectionZones-response)))
  "Returns md5sum for a message object of type 'ReadAllProtectionZones-response"
  "26bc18c08c1b8389444ea1d92e6c8745")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ReadAllProtectionZones-response>)))
  "Returns full string definition for message of type '<ReadAllProtectionZones-response>"
  (cl:format cl:nil "ProtectionZoneList output~%~%================================================================================~%MSG: kortex_driver/ProtectionZoneList~%~%ProtectionZone[] protection_zones~%================================================================================~%MSG: kortex_driver/ProtectionZone~%~%ProtectionZoneHandle handle~%string name~%string application_data~%bool is_enabled~%ZoneShape shape~%CartesianLimitation[] limitations~%CartesianLimitation[] envelope_limitations~%================================================================================~%MSG: kortex_driver/ProtectionZoneHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/ZoneShape~%~%uint32 shape_type~%Point origin~%Base_RotationMatrix orientation~%float32[] dimensions~%float32 envelope_thickness~%================================================================================~%MSG: kortex_driver/Point~%~%float32 x~%float32 y~%float32 z~%================================================================================~%MSG: kortex_driver/Base_RotationMatrix~%~%Base_RotationMatrixRow row1~%Base_RotationMatrixRow row2~%Base_RotationMatrixRow row3~%================================================================================~%MSG: kortex_driver/Base_RotationMatrixRow~%~%float32 column1~%float32 column2~%float32 column3~%================================================================================~%MSG: kortex_driver/CartesianLimitation~%~%uint32 type~%float32 translation~%float32 orientation~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ReadAllProtectionZones-response)))
  "Returns full string definition for message of type 'ReadAllProtectionZones-response"
  (cl:format cl:nil "ProtectionZoneList output~%~%================================================================================~%MSG: kortex_driver/ProtectionZoneList~%~%ProtectionZone[] protection_zones~%================================================================================~%MSG: kortex_driver/ProtectionZone~%~%ProtectionZoneHandle handle~%string name~%string application_data~%bool is_enabled~%ZoneShape shape~%CartesianLimitation[] limitations~%CartesianLimitation[] envelope_limitations~%================================================================================~%MSG: kortex_driver/ProtectionZoneHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/ZoneShape~%~%uint32 shape_type~%Point origin~%Base_RotationMatrix orientation~%float32[] dimensions~%float32 envelope_thickness~%================================================================================~%MSG: kortex_driver/Point~%~%float32 x~%float32 y~%float32 z~%================================================================================~%MSG: kortex_driver/Base_RotationMatrix~%~%Base_RotationMatrixRow row1~%Base_RotationMatrixRow row2~%Base_RotationMatrixRow row3~%================================================================================~%MSG: kortex_driver/Base_RotationMatrixRow~%~%float32 column1~%float32 column2~%float32 column3~%================================================================================~%MSG: kortex_driver/CartesianLimitation~%~%uint32 type~%float32 translation~%float32 orientation~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ReadAllProtectionZones-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ReadAllProtectionZones-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ReadAllProtectionZones-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ReadAllProtectionZones)))
  'ReadAllProtectionZones-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ReadAllProtectionZones)))
  'ReadAllProtectionZones-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ReadAllProtectionZones)))
  "Returns string type for a service object of type '<ReadAllProtectionZones>"
  "kortex_driver/ReadAllProtectionZones")
; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude ReadProtectionZone-request.msg.html

(cl:defclass <ReadProtectionZone-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:ProtectionZoneHandle
    :initform (cl:make-instance 'kortex_driver-msg:ProtectionZoneHandle)))
)

(cl:defclass ReadProtectionZone-request (<ReadProtectionZone-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ReadProtectionZone-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ReadProtectionZone-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<ReadProtectionZone-request> is deprecated: use kortex_driver-srv:ReadProtectionZone-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <ReadProtectionZone-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ReadProtectionZone-request>) ostream)
  "Serializes a message object of type '<ReadProtectionZone-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ReadProtectionZone-request>) istream)
  "Deserializes a message object of type '<ReadProtectionZone-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ReadProtectionZone-request>)))
  "Returns string type for a service object of type '<ReadProtectionZone-request>"
  "kortex_driver/ReadProtectionZoneRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ReadProtectionZone-request)))
  "Returns string type for a service object of type 'ReadProtectionZone-request"
  "kortex_driver/ReadProtectionZoneRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ReadProtectionZone-request>)))
  "Returns md5sum for a message object of type '<ReadProtectionZone-request>"
  "fd53b25b5f35cd926da1c19403c70bc5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ReadProtectionZone-request)))
  "Returns md5sum for a message object of type 'ReadProtectionZone-request"
  "fd53b25b5f35cd926da1c19403c70bc5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ReadProtectionZone-request>)))
  "Returns full string definition for message of type '<ReadProtectionZone-request>"
  (cl:format cl:nil "ProtectionZoneHandle input~%~%================================================================================~%MSG: kortex_driver/ProtectionZoneHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ReadProtectionZone-request)))
  "Returns full string definition for message of type 'ReadProtectionZone-request"
  (cl:format cl:nil "ProtectionZoneHandle input~%~%================================================================================~%MSG: kortex_driver/ProtectionZoneHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ReadProtectionZone-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ReadProtectionZone-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ReadProtectionZone-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude ReadProtectionZone-response.msg.html

(cl:defclass <ReadProtectionZone-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:ProtectionZone
    :initform (cl:make-instance 'kortex_driver-msg:ProtectionZone)))
)

(cl:defclass ReadProtectionZone-response (<ReadProtectionZone-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ReadProtectionZone-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ReadProtectionZone-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<ReadProtectionZone-response> is deprecated: use kortex_driver-srv:ReadProtectionZone-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <ReadProtectionZone-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ReadProtectionZone-response>) ostream)
  "Serializes a message object of type '<ReadProtectionZone-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ReadProtectionZone-response>) istream)
  "Deserializes a message object of type '<ReadProtectionZone-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ReadProtectionZone-response>)))
  "Returns string type for a service object of type '<ReadProtectionZone-response>"
  "kortex_driver/ReadProtectionZoneResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ReadProtectionZone-response)))
  "Returns string type for a service object of type 'ReadProtectionZone-response"
  "kortex_driver/ReadProtectionZoneResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ReadProtectionZone-response>)))
  "Returns md5sum for a message object of type '<ReadProtectionZone-response>"
  "fd53b25b5f35cd926da1c19403c70bc5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ReadProtectionZone-response)))
  "Returns md5sum for a message object of type 'ReadProtectionZone-response"
  "fd53b25b5f35cd926da1c19403c70bc5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ReadProtectionZone-response>)))
  "Returns full string definition for message of type '<ReadProtectionZone-response>"
  (cl:format cl:nil "ProtectionZone output~%~%================================================================================~%MSG: kortex_driver/ProtectionZone~%~%ProtectionZoneHandle handle~%string name~%string application_data~%bool is_enabled~%ZoneShape shape~%CartesianLimitation[] limitations~%CartesianLimitation[] envelope_limitations~%================================================================================~%MSG: kortex_driver/ProtectionZoneHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/ZoneShape~%~%uint32 shape_type~%Point origin~%Base_RotationMatrix orientation~%float32[] dimensions~%float32 envelope_thickness~%================================================================================~%MSG: kortex_driver/Point~%~%float32 x~%float32 y~%float32 z~%================================================================================~%MSG: kortex_driver/Base_RotationMatrix~%~%Base_RotationMatrixRow row1~%Base_RotationMatrixRow row2~%Base_RotationMatrixRow row3~%================================================================================~%MSG: kortex_driver/Base_RotationMatrixRow~%~%float32 column1~%float32 column2~%float32 column3~%================================================================================~%MSG: kortex_driver/CartesianLimitation~%~%uint32 type~%float32 translation~%float32 orientation~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ReadProtectionZone-response)))
  "Returns full string definition for message of type 'ReadProtectionZone-response"
  (cl:format cl:nil "ProtectionZone output~%~%================================================================================~%MSG: kortex_driver/ProtectionZone~%~%ProtectionZoneHandle handle~%string name~%string application_data~%bool is_enabled~%ZoneShape shape~%CartesianLimitation[] limitations~%CartesianLimitation[] envelope_limitations~%================================================================================~%MSG: kortex_driver/ProtectionZoneHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/ZoneShape~%~%uint32 shape_type~%Point origin~%Base_RotationMatrix orientation~%float32[] dimensions~%float32 envelope_thickness~%================================================================================~%MSG: kortex_driver/Point~%~%float32 x~%float32 y~%float32 z~%================================================================================~%MSG: kortex_driver/Base_RotationMatrix~%~%Base_RotationMatrixRow row1~%Base_RotationMatrixRow row2~%Base_RotationMatrixRow row3~%================================================================================~%MSG: kortex_driver/Base_RotationMatrixRow~%~%float32 column1~%float32 column2~%float32 column3~%================================================================================~%MSG: kortex_driver/CartesianLimitation~%~%uint32 type~%float32 translation~%float32 orientation~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ReadProtectionZone-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ReadProtectionZone-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ReadProtectionZone-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ReadProtectionZone)))
  'ReadProtectionZone-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ReadProtectionZone)))
  'ReadProtectionZone-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ReadProtectionZone)))
  "Returns string type for a service object of type '<ReadProtectionZone>"
  "kortex_driver/ReadProtectionZone")
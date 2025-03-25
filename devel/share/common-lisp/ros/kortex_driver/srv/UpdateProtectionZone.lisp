; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude UpdateProtectionZone-request.msg.html

(cl:defclass <UpdateProtectionZone-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:ProtectionZone
    :initform (cl:make-instance 'kortex_driver-msg:ProtectionZone)))
)

(cl:defclass UpdateProtectionZone-request (<UpdateProtectionZone-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <UpdateProtectionZone-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'UpdateProtectionZone-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<UpdateProtectionZone-request> is deprecated: use kortex_driver-srv:UpdateProtectionZone-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <UpdateProtectionZone-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <UpdateProtectionZone-request>) ostream)
  "Serializes a message object of type '<UpdateProtectionZone-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <UpdateProtectionZone-request>) istream)
  "Deserializes a message object of type '<UpdateProtectionZone-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<UpdateProtectionZone-request>)))
  "Returns string type for a service object of type '<UpdateProtectionZone-request>"
  "kortex_driver/UpdateProtectionZoneRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'UpdateProtectionZone-request)))
  "Returns string type for a service object of type 'UpdateProtectionZone-request"
  "kortex_driver/UpdateProtectionZoneRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<UpdateProtectionZone-request>)))
  "Returns md5sum for a message object of type '<UpdateProtectionZone-request>"
  "b1c7755249865d381b4b70557a125e12")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'UpdateProtectionZone-request)))
  "Returns md5sum for a message object of type 'UpdateProtectionZone-request"
  "b1c7755249865d381b4b70557a125e12")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<UpdateProtectionZone-request>)))
  "Returns full string definition for message of type '<UpdateProtectionZone-request>"
  (cl:format cl:nil "ProtectionZone input~%~%================================================================================~%MSG: kortex_driver/ProtectionZone~%~%ProtectionZoneHandle handle~%string name~%string application_data~%bool is_enabled~%ZoneShape shape~%CartesianLimitation[] limitations~%CartesianLimitation[] envelope_limitations~%================================================================================~%MSG: kortex_driver/ProtectionZoneHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/ZoneShape~%~%uint32 shape_type~%Point origin~%Base_RotationMatrix orientation~%float32[] dimensions~%float32 envelope_thickness~%================================================================================~%MSG: kortex_driver/Point~%~%float32 x~%float32 y~%float32 z~%================================================================================~%MSG: kortex_driver/Base_RotationMatrix~%~%Base_RotationMatrixRow row1~%Base_RotationMatrixRow row2~%Base_RotationMatrixRow row3~%================================================================================~%MSG: kortex_driver/Base_RotationMatrixRow~%~%float32 column1~%float32 column2~%float32 column3~%================================================================================~%MSG: kortex_driver/CartesianLimitation~%~%uint32 type~%float32 translation~%float32 orientation~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'UpdateProtectionZone-request)))
  "Returns full string definition for message of type 'UpdateProtectionZone-request"
  (cl:format cl:nil "ProtectionZone input~%~%================================================================================~%MSG: kortex_driver/ProtectionZone~%~%ProtectionZoneHandle handle~%string name~%string application_data~%bool is_enabled~%ZoneShape shape~%CartesianLimitation[] limitations~%CartesianLimitation[] envelope_limitations~%================================================================================~%MSG: kortex_driver/ProtectionZoneHandle~%~%uint32 identifier~%uint32 permission~%================================================================================~%MSG: kortex_driver/ZoneShape~%~%uint32 shape_type~%Point origin~%Base_RotationMatrix orientation~%float32[] dimensions~%float32 envelope_thickness~%================================================================================~%MSG: kortex_driver/Point~%~%float32 x~%float32 y~%float32 z~%================================================================================~%MSG: kortex_driver/Base_RotationMatrix~%~%Base_RotationMatrixRow row1~%Base_RotationMatrixRow row2~%Base_RotationMatrixRow row3~%================================================================================~%MSG: kortex_driver/Base_RotationMatrixRow~%~%float32 column1~%float32 column2~%float32 column3~%================================================================================~%MSG: kortex_driver/CartesianLimitation~%~%uint32 type~%float32 translation~%float32 orientation~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <UpdateProtectionZone-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <UpdateProtectionZone-request>))
  "Converts a ROS message object to a list"
  (cl:list 'UpdateProtectionZone-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude UpdateProtectionZone-response.msg.html

(cl:defclass <UpdateProtectionZone-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass UpdateProtectionZone-response (<UpdateProtectionZone-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <UpdateProtectionZone-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'UpdateProtectionZone-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<UpdateProtectionZone-response> is deprecated: use kortex_driver-srv:UpdateProtectionZone-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <UpdateProtectionZone-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <UpdateProtectionZone-response>) ostream)
  "Serializes a message object of type '<UpdateProtectionZone-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <UpdateProtectionZone-response>) istream)
  "Deserializes a message object of type '<UpdateProtectionZone-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<UpdateProtectionZone-response>)))
  "Returns string type for a service object of type '<UpdateProtectionZone-response>"
  "kortex_driver/UpdateProtectionZoneResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'UpdateProtectionZone-response)))
  "Returns string type for a service object of type 'UpdateProtectionZone-response"
  "kortex_driver/UpdateProtectionZoneResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<UpdateProtectionZone-response>)))
  "Returns md5sum for a message object of type '<UpdateProtectionZone-response>"
  "b1c7755249865d381b4b70557a125e12")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'UpdateProtectionZone-response)))
  "Returns md5sum for a message object of type 'UpdateProtectionZone-response"
  "b1c7755249865d381b4b70557a125e12")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<UpdateProtectionZone-response>)))
  "Returns full string definition for message of type '<UpdateProtectionZone-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'UpdateProtectionZone-response)))
  "Returns full string definition for message of type 'UpdateProtectionZone-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <UpdateProtectionZone-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <UpdateProtectionZone-response>))
  "Converts a ROS message object to a list"
  (cl:list 'UpdateProtectionZone-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'UpdateProtectionZone)))
  'UpdateProtectionZone-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'UpdateProtectionZone)))
  'UpdateProtectionZone-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'UpdateProtectionZone)))
  "Returns string type for a service object of type '<UpdateProtectionZone>"
  "kortex_driver/UpdateProtectionZone")
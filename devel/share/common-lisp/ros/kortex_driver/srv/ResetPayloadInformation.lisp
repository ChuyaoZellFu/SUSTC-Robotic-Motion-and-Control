; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude ResetPayloadInformation-request.msg.html

(cl:defclass <ResetPayloadInformation-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass ResetPayloadInformation-request (<ResetPayloadInformation-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ResetPayloadInformation-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ResetPayloadInformation-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<ResetPayloadInformation-request> is deprecated: use kortex_driver-srv:ResetPayloadInformation-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <ResetPayloadInformation-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ResetPayloadInformation-request>) ostream)
  "Serializes a message object of type '<ResetPayloadInformation-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ResetPayloadInformation-request>) istream)
  "Deserializes a message object of type '<ResetPayloadInformation-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ResetPayloadInformation-request>)))
  "Returns string type for a service object of type '<ResetPayloadInformation-request>"
  "kortex_driver/ResetPayloadInformationRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ResetPayloadInformation-request)))
  "Returns string type for a service object of type 'ResetPayloadInformation-request"
  "kortex_driver/ResetPayloadInformationRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ResetPayloadInformation-request>)))
  "Returns md5sum for a message object of type '<ResetPayloadInformation-request>"
  "38a744b19ddbb71fb4d7e8724de570f6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ResetPayloadInformation-request)))
  "Returns md5sum for a message object of type 'ResetPayloadInformation-request"
  "38a744b19ddbb71fb4d7e8724de570f6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ResetPayloadInformation-request>)))
  "Returns full string definition for message of type '<ResetPayloadInformation-request>"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ResetPayloadInformation-request)))
  "Returns full string definition for message of type 'ResetPayloadInformation-request"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ResetPayloadInformation-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ResetPayloadInformation-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ResetPayloadInformation-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude ResetPayloadInformation-response.msg.html

(cl:defclass <ResetPayloadInformation-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:PayloadInformation
    :initform (cl:make-instance 'kortex_driver-msg:PayloadInformation)))
)

(cl:defclass ResetPayloadInformation-response (<ResetPayloadInformation-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ResetPayloadInformation-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ResetPayloadInformation-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<ResetPayloadInformation-response> is deprecated: use kortex_driver-srv:ResetPayloadInformation-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <ResetPayloadInformation-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ResetPayloadInformation-response>) ostream)
  "Serializes a message object of type '<ResetPayloadInformation-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ResetPayloadInformation-response>) istream)
  "Deserializes a message object of type '<ResetPayloadInformation-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ResetPayloadInformation-response>)))
  "Returns string type for a service object of type '<ResetPayloadInformation-response>"
  "kortex_driver/ResetPayloadInformationResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ResetPayloadInformation-response)))
  "Returns string type for a service object of type 'ResetPayloadInformation-response"
  "kortex_driver/ResetPayloadInformationResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ResetPayloadInformation-response>)))
  "Returns md5sum for a message object of type '<ResetPayloadInformation-response>"
  "38a744b19ddbb71fb4d7e8724de570f6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ResetPayloadInformation-response)))
  "Returns md5sum for a message object of type 'ResetPayloadInformation-response"
  "38a744b19ddbb71fb4d7e8724de570f6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ResetPayloadInformation-response>)))
  "Returns full string definition for message of type '<ResetPayloadInformation-response>"
  (cl:format cl:nil "PayloadInformation output~%~%================================================================================~%MSG: kortex_driver/PayloadInformation~%~%float32 payload_mass~%ControlConfig_Position payload_mass_center~%================================================================================~%MSG: kortex_driver/ControlConfig_Position~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ResetPayloadInformation-response)))
  "Returns full string definition for message of type 'ResetPayloadInformation-response"
  (cl:format cl:nil "PayloadInformation output~%~%================================================================================~%MSG: kortex_driver/PayloadInformation~%~%float32 payload_mass~%ControlConfig_Position payload_mass_center~%================================================================================~%MSG: kortex_driver/ControlConfig_Position~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ResetPayloadInformation-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ResetPayloadInformation-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ResetPayloadInformation-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ResetPayloadInformation)))
  'ResetPayloadInformation-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ResetPayloadInformation)))
  'ResetPayloadInformation-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ResetPayloadInformation)))
  "Returns string type for a service object of type '<ResetPayloadInformation>"
  "kortex_driver/ResetPayloadInformation")
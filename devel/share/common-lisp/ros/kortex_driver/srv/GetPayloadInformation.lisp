; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude GetPayloadInformation-request.msg.html

(cl:defclass <GetPayloadInformation-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass GetPayloadInformation-request (<GetPayloadInformation-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetPayloadInformation-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetPayloadInformation-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetPayloadInformation-request> is deprecated: use kortex_driver-srv:GetPayloadInformation-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <GetPayloadInformation-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetPayloadInformation-request>) ostream)
  "Serializes a message object of type '<GetPayloadInformation-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetPayloadInformation-request>) istream)
  "Deserializes a message object of type '<GetPayloadInformation-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetPayloadInformation-request>)))
  "Returns string type for a service object of type '<GetPayloadInformation-request>"
  "kortex_driver/GetPayloadInformationRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetPayloadInformation-request)))
  "Returns string type for a service object of type 'GetPayloadInformation-request"
  "kortex_driver/GetPayloadInformationRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetPayloadInformation-request>)))
  "Returns md5sum for a message object of type '<GetPayloadInformation-request>"
  "38a744b19ddbb71fb4d7e8724de570f6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetPayloadInformation-request)))
  "Returns md5sum for a message object of type 'GetPayloadInformation-request"
  "38a744b19ddbb71fb4d7e8724de570f6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetPayloadInformation-request>)))
  "Returns full string definition for message of type '<GetPayloadInformation-request>"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetPayloadInformation-request)))
  "Returns full string definition for message of type 'GetPayloadInformation-request"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetPayloadInformation-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetPayloadInformation-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetPayloadInformation-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude GetPayloadInformation-response.msg.html

(cl:defclass <GetPayloadInformation-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:PayloadInformation
    :initform (cl:make-instance 'kortex_driver-msg:PayloadInformation)))
)

(cl:defclass GetPayloadInformation-response (<GetPayloadInformation-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetPayloadInformation-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetPayloadInformation-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetPayloadInformation-response> is deprecated: use kortex_driver-srv:GetPayloadInformation-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <GetPayloadInformation-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetPayloadInformation-response>) ostream)
  "Serializes a message object of type '<GetPayloadInformation-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetPayloadInformation-response>) istream)
  "Deserializes a message object of type '<GetPayloadInformation-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetPayloadInformation-response>)))
  "Returns string type for a service object of type '<GetPayloadInformation-response>"
  "kortex_driver/GetPayloadInformationResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetPayloadInformation-response)))
  "Returns string type for a service object of type 'GetPayloadInformation-response"
  "kortex_driver/GetPayloadInformationResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetPayloadInformation-response>)))
  "Returns md5sum for a message object of type '<GetPayloadInformation-response>"
  "38a744b19ddbb71fb4d7e8724de570f6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetPayloadInformation-response)))
  "Returns md5sum for a message object of type 'GetPayloadInformation-response"
  "38a744b19ddbb71fb4d7e8724de570f6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetPayloadInformation-response>)))
  "Returns full string definition for message of type '<GetPayloadInformation-response>"
  (cl:format cl:nil "PayloadInformation output~%~%================================================================================~%MSG: kortex_driver/PayloadInformation~%~%float32 payload_mass~%ControlConfig_Position payload_mass_center~%================================================================================~%MSG: kortex_driver/ControlConfig_Position~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetPayloadInformation-response)))
  "Returns full string definition for message of type 'GetPayloadInformation-response"
  (cl:format cl:nil "PayloadInformation output~%~%================================================================================~%MSG: kortex_driver/PayloadInformation~%~%float32 payload_mass~%ControlConfig_Position payload_mass_center~%================================================================================~%MSG: kortex_driver/ControlConfig_Position~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetPayloadInformation-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetPayloadInformation-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetPayloadInformation-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetPayloadInformation)))
  'GetPayloadInformation-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetPayloadInformation)))
  'GetPayloadInformation-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetPayloadInformation)))
  "Returns string type for a service object of type '<GetPayloadInformation>"
  "kortex_driver/GetPayloadInformation")
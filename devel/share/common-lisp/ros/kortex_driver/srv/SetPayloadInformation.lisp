; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude SetPayloadInformation-request.msg.html

(cl:defclass <SetPayloadInformation-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:PayloadInformation
    :initform (cl:make-instance 'kortex_driver-msg:PayloadInformation)))
)

(cl:defclass SetPayloadInformation-request (<SetPayloadInformation-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetPayloadInformation-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetPayloadInformation-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SetPayloadInformation-request> is deprecated: use kortex_driver-srv:SetPayloadInformation-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <SetPayloadInformation-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetPayloadInformation-request>) ostream)
  "Serializes a message object of type '<SetPayloadInformation-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetPayloadInformation-request>) istream)
  "Deserializes a message object of type '<SetPayloadInformation-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetPayloadInformation-request>)))
  "Returns string type for a service object of type '<SetPayloadInformation-request>"
  "kortex_driver/SetPayloadInformationRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetPayloadInformation-request)))
  "Returns string type for a service object of type 'SetPayloadInformation-request"
  "kortex_driver/SetPayloadInformationRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetPayloadInformation-request>)))
  "Returns md5sum for a message object of type '<SetPayloadInformation-request>"
  "132bdc38d85ff3aec89e76f693ad5e54")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetPayloadInformation-request)))
  "Returns md5sum for a message object of type 'SetPayloadInformation-request"
  "132bdc38d85ff3aec89e76f693ad5e54")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetPayloadInformation-request>)))
  "Returns full string definition for message of type '<SetPayloadInformation-request>"
  (cl:format cl:nil "PayloadInformation input~%~%================================================================================~%MSG: kortex_driver/PayloadInformation~%~%float32 payload_mass~%ControlConfig_Position payload_mass_center~%================================================================================~%MSG: kortex_driver/ControlConfig_Position~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetPayloadInformation-request)))
  "Returns full string definition for message of type 'SetPayloadInformation-request"
  (cl:format cl:nil "PayloadInformation input~%~%================================================================================~%MSG: kortex_driver/PayloadInformation~%~%float32 payload_mass~%ControlConfig_Position payload_mass_center~%================================================================================~%MSG: kortex_driver/ControlConfig_Position~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetPayloadInformation-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetPayloadInformation-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetPayloadInformation-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude SetPayloadInformation-response.msg.html

(cl:defclass <SetPayloadInformation-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass SetPayloadInformation-response (<SetPayloadInformation-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetPayloadInformation-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetPayloadInformation-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SetPayloadInformation-response> is deprecated: use kortex_driver-srv:SetPayloadInformation-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <SetPayloadInformation-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetPayloadInformation-response>) ostream)
  "Serializes a message object of type '<SetPayloadInformation-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetPayloadInformation-response>) istream)
  "Deserializes a message object of type '<SetPayloadInformation-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetPayloadInformation-response>)))
  "Returns string type for a service object of type '<SetPayloadInformation-response>"
  "kortex_driver/SetPayloadInformationResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetPayloadInformation-response)))
  "Returns string type for a service object of type 'SetPayloadInformation-response"
  "kortex_driver/SetPayloadInformationResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetPayloadInformation-response>)))
  "Returns md5sum for a message object of type '<SetPayloadInformation-response>"
  "132bdc38d85ff3aec89e76f693ad5e54")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetPayloadInformation-response)))
  "Returns md5sum for a message object of type 'SetPayloadInformation-response"
  "132bdc38d85ff3aec89e76f693ad5e54")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetPayloadInformation-response>)))
  "Returns full string definition for message of type '<SetPayloadInformation-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetPayloadInformation-response)))
  "Returns full string definition for message of type 'SetPayloadInformation-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetPayloadInformation-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetPayloadInformation-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetPayloadInformation-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetPayloadInformation)))
  'SetPayloadInformation-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetPayloadInformation)))
  'SetPayloadInformation-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetPayloadInformation)))
  "Returns string type for a service object of type '<SetPayloadInformation>"
  "kortex_driver/SetPayloadInformation")
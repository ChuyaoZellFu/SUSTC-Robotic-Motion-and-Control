; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude GetOptionInformation-request.msg.html

(cl:defclass <GetOptionInformation-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:OptionIdentifier
    :initform (cl:make-instance 'kortex_driver-msg:OptionIdentifier)))
)

(cl:defclass GetOptionInformation-request (<GetOptionInformation-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetOptionInformation-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetOptionInformation-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetOptionInformation-request> is deprecated: use kortex_driver-srv:GetOptionInformation-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <GetOptionInformation-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetOptionInformation-request>) ostream)
  "Serializes a message object of type '<GetOptionInformation-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetOptionInformation-request>) istream)
  "Deserializes a message object of type '<GetOptionInformation-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetOptionInformation-request>)))
  "Returns string type for a service object of type '<GetOptionInformation-request>"
  "kortex_driver/GetOptionInformationRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetOptionInformation-request)))
  "Returns string type for a service object of type 'GetOptionInformation-request"
  "kortex_driver/GetOptionInformationRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetOptionInformation-request>)))
  "Returns md5sum for a message object of type '<GetOptionInformation-request>"
  "0af2e58d9822a8c26f4c5a66d0d6015b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetOptionInformation-request)))
  "Returns md5sum for a message object of type 'GetOptionInformation-request"
  "0af2e58d9822a8c26f4c5a66d0d6015b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetOptionInformation-request>)))
  "Returns full string definition for message of type '<GetOptionInformation-request>"
  (cl:format cl:nil "OptionIdentifier input~%~%================================================================================~%MSG: kortex_driver/OptionIdentifier~%~%uint32 sensor~%uint32 option~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetOptionInformation-request)))
  "Returns full string definition for message of type 'GetOptionInformation-request"
  (cl:format cl:nil "OptionIdentifier input~%~%================================================================================~%MSG: kortex_driver/OptionIdentifier~%~%uint32 sensor~%uint32 option~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetOptionInformation-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetOptionInformation-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetOptionInformation-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude GetOptionInformation-response.msg.html

(cl:defclass <GetOptionInformation-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:OptionInformation
    :initform (cl:make-instance 'kortex_driver-msg:OptionInformation)))
)

(cl:defclass GetOptionInformation-response (<GetOptionInformation-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetOptionInformation-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetOptionInformation-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetOptionInformation-response> is deprecated: use kortex_driver-srv:GetOptionInformation-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <GetOptionInformation-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetOptionInformation-response>) ostream)
  "Serializes a message object of type '<GetOptionInformation-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetOptionInformation-response>) istream)
  "Deserializes a message object of type '<GetOptionInformation-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetOptionInformation-response>)))
  "Returns string type for a service object of type '<GetOptionInformation-response>"
  "kortex_driver/GetOptionInformationResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetOptionInformation-response)))
  "Returns string type for a service object of type 'GetOptionInformation-response"
  "kortex_driver/GetOptionInformationResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetOptionInformation-response>)))
  "Returns md5sum for a message object of type '<GetOptionInformation-response>"
  "0af2e58d9822a8c26f4c5a66d0d6015b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetOptionInformation-response)))
  "Returns md5sum for a message object of type 'GetOptionInformation-response"
  "0af2e58d9822a8c26f4c5a66d0d6015b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetOptionInformation-response>)))
  "Returns full string definition for message of type '<GetOptionInformation-response>"
  (cl:format cl:nil "OptionInformation output~%~%================================================================================~%MSG: kortex_driver/OptionInformation~%~%uint32 sensor~%uint32 option~%bool supported~%bool read_only~%float32 minimum~%float32 maximum~%float32 step~%float32 default_value~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetOptionInformation-response)))
  "Returns full string definition for message of type 'GetOptionInformation-response"
  (cl:format cl:nil "OptionInformation output~%~%================================================================================~%MSG: kortex_driver/OptionInformation~%~%uint32 sensor~%uint32 option~%bool supported~%bool read_only~%float32 minimum~%float32 maximum~%float32 step~%float32 default_value~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetOptionInformation-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetOptionInformation-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetOptionInformation-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetOptionInformation)))
  'GetOptionInformation-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetOptionInformation)))
  'GetOptionInformation-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetOptionInformation)))
  "Returns string type for a service object of type '<GetOptionInformation>"
  "kortex_driver/GetOptionInformation")
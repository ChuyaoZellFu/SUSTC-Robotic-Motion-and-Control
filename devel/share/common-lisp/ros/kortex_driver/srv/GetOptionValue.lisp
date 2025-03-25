; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude GetOptionValue-request.msg.html

(cl:defclass <GetOptionValue-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:OptionIdentifier
    :initform (cl:make-instance 'kortex_driver-msg:OptionIdentifier)))
)

(cl:defclass GetOptionValue-request (<GetOptionValue-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetOptionValue-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetOptionValue-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetOptionValue-request> is deprecated: use kortex_driver-srv:GetOptionValue-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <GetOptionValue-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetOptionValue-request>) ostream)
  "Serializes a message object of type '<GetOptionValue-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetOptionValue-request>) istream)
  "Deserializes a message object of type '<GetOptionValue-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetOptionValue-request>)))
  "Returns string type for a service object of type '<GetOptionValue-request>"
  "kortex_driver/GetOptionValueRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetOptionValue-request)))
  "Returns string type for a service object of type 'GetOptionValue-request"
  "kortex_driver/GetOptionValueRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetOptionValue-request>)))
  "Returns md5sum for a message object of type '<GetOptionValue-request>"
  "e414d686b57953b5748ab5bedaec5577")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetOptionValue-request)))
  "Returns md5sum for a message object of type 'GetOptionValue-request"
  "e414d686b57953b5748ab5bedaec5577")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetOptionValue-request>)))
  "Returns full string definition for message of type '<GetOptionValue-request>"
  (cl:format cl:nil "OptionIdentifier input~%~%================================================================================~%MSG: kortex_driver/OptionIdentifier~%~%uint32 sensor~%uint32 option~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetOptionValue-request)))
  "Returns full string definition for message of type 'GetOptionValue-request"
  (cl:format cl:nil "OptionIdentifier input~%~%================================================================================~%MSG: kortex_driver/OptionIdentifier~%~%uint32 sensor~%uint32 option~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetOptionValue-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetOptionValue-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetOptionValue-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude GetOptionValue-response.msg.html

(cl:defclass <GetOptionValue-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:OptionValue
    :initform (cl:make-instance 'kortex_driver-msg:OptionValue)))
)

(cl:defclass GetOptionValue-response (<GetOptionValue-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetOptionValue-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetOptionValue-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetOptionValue-response> is deprecated: use kortex_driver-srv:GetOptionValue-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <GetOptionValue-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetOptionValue-response>) ostream)
  "Serializes a message object of type '<GetOptionValue-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetOptionValue-response>) istream)
  "Deserializes a message object of type '<GetOptionValue-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetOptionValue-response>)))
  "Returns string type for a service object of type '<GetOptionValue-response>"
  "kortex_driver/GetOptionValueResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetOptionValue-response)))
  "Returns string type for a service object of type 'GetOptionValue-response"
  "kortex_driver/GetOptionValueResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetOptionValue-response>)))
  "Returns md5sum for a message object of type '<GetOptionValue-response>"
  "e414d686b57953b5748ab5bedaec5577")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetOptionValue-response)))
  "Returns md5sum for a message object of type 'GetOptionValue-response"
  "e414d686b57953b5748ab5bedaec5577")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetOptionValue-response>)))
  "Returns full string definition for message of type '<GetOptionValue-response>"
  (cl:format cl:nil "OptionValue output~%~%================================================================================~%MSG: kortex_driver/OptionValue~%~%uint32 sensor~%uint32 option~%float32 value~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetOptionValue-response)))
  "Returns full string definition for message of type 'GetOptionValue-response"
  (cl:format cl:nil "OptionValue output~%~%================================================================================~%MSG: kortex_driver/OptionValue~%~%uint32 sensor~%uint32 option~%float32 value~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetOptionValue-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetOptionValue-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetOptionValue-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetOptionValue)))
  'GetOptionValue-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetOptionValue)))
  'GetOptionValue-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetOptionValue)))
  "Returns string type for a service object of type '<GetOptionValue>"
  "kortex_driver/GetOptionValue")
; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude GetSerialNumber-request.msg.html

(cl:defclass <GetSerialNumber-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass GetSerialNumber-request (<GetSerialNumber-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetSerialNumber-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetSerialNumber-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetSerialNumber-request> is deprecated: use kortex_driver-srv:GetSerialNumber-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <GetSerialNumber-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetSerialNumber-request>) ostream)
  "Serializes a message object of type '<GetSerialNumber-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetSerialNumber-request>) istream)
  "Deserializes a message object of type '<GetSerialNumber-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetSerialNumber-request>)))
  "Returns string type for a service object of type '<GetSerialNumber-request>"
  "kortex_driver/GetSerialNumberRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetSerialNumber-request)))
  "Returns string type for a service object of type 'GetSerialNumber-request"
  "kortex_driver/GetSerialNumberRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetSerialNumber-request>)))
  "Returns md5sum for a message object of type '<GetSerialNumber-request>"
  "0249587230c33b79820737b46150559e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetSerialNumber-request)))
  "Returns md5sum for a message object of type 'GetSerialNumber-request"
  "0249587230c33b79820737b46150559e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetSerialNumber-request>)))
  "Returns full string definition for message of type '<GetSerialNumber-request>"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetSerialNumber-request)))
  "Returns full string definition for message of type 'GetSerialNumber-request"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetSerialNumber-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetSerialNumber-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetSerialNumber-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude GetSerialNumber-response.msg.html

(cl:defclass <GetSerialNumber-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:SerialNumber
    :initform (cl:make-instance 'kortex_driver-msg:SerialNumber)))
)

(cl:defclass GetSerialNumber-response (<GetSerialNumber-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetSerialNumber-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetSerialNumber-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetSerialNumber-response> is deprecated: use kortex_driver-srv:GetSerialNumber-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <GetSerialNumber-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetSerialNumber-response>) ostream)
  "Serializes a message object of type '<GetSerialNumber-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetSerialNumber-response>) istream)
  "Deserializes a message object of type '<GetSerialNumber-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetSerialNumber-response>)))
  "Returns string type for a service object of type '<GetSerialNumber-response>"
  "kortex_driver/GetSerialNumberResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetSerialNumber-response)))
  "Returns string type for a service object of type 'GetSerialNumber-response"
  "kortex_driver/GetSerialNumberResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetSerialNumber-response>)))
  "Returns md5sum for a message object of type '<GetSerialNumber-response>"
  "0249587230c33b79820737b46150559e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetSerialNumber-response)))
  "Returns md5sum for a message object of type 'GetSerialNumber-response"
  "0249587230c33b79820737b46150559e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetSerialNumber-response>)))
  "Returns full string definition for message of type '<GetSerialNumber-response>"
  (cl:format cl:nil "SerialNumber output~%~%================================================================================~%MSG: kortex_driver/SerialNumber~%~%string serial_number~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetSerialNumber-response)))
  "Returns full string definition for message of type 'GetSerialNumber-response"
  (cl:format cl:nil "SerialNumber output~%~%================================================================================~%MSG: kortex_driver/SerialNumber~%~%string serial_number~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetSerialNumber-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetSerialNumber-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetSerialNumber-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetSerialNumber)))
  'GetSerialNumber-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetSerialNumber)))
  'GetSerialNumber-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetSerialNumber)))
  "Returns string type for a service object of type '<GetSerialNumber>"
  "kortex_driver/GetSerialNumber")
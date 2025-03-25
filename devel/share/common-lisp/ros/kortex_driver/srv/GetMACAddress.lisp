; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude GetMACAddress-request.msg.html

(cl:defclass <GetMACAddress-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass GetMACAddress-request (<GetMACAddress-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetMACAddress-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetMACAddress-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetMACAddress-request> is deprecated: use kortex_driver-srv:GetMACAddress-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <GetMACAddress-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetMACAddress-request>) ostream)
  "Serializes a message object of type '<GetMACAddress-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetMACAddress-request>) istream)
  "Deserializes a message object of type '<GetMACAddress-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetMACAddress-request>)))
  "Returns string type for a service object of type '<GetMACAddress-request>"
  "kortex_driver/GetMACAddressRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetMACAddress-request)))
  "Returns string type for a service object of type 'GetMACAddress-request"
  "kortex_driver/GetMACAddressRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetMACAddress-request>)))
  "Returns md5sum for a message object of type '<GetMACAddress-request>"
  "bf0ce987b74a5e470bf1deeae1772263")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetMACAddress-request)))
  "Returns md5sum for a message object of type 'GetMACAddress-request"
  "bf0ce987b74a5e470bf1deeae1772263")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetMACAddress-request>)))
  "Returns full string definition for message of type '<GetMACAddress-request>"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetMACAddress-request)))
  "Returns full string definition for message of type 'GetMACAddress-request"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetMACAddress-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetMACAddress-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetMACAddress-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude GetMACAddress-response.msg.html

(cl:defclass <GetMACAddress-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:MACAddress
    :initform (cl:make-instance 'kortex_driver-msg:MACAddress)))
)

(cl:defclass GetMACAddress-response (<GetMACAddress-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetMACAddress-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetMACAddress-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetMACAddress-response> is deprecated: use kortex_driver-srv:GetMACAddress-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <GetMACAddress-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetMACAddress-response>) ostream)
  "Serializes a message object of type '<GetMACAddress-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetMACAddress-response>) istream)
  "Deserializes a message object of type '<GetMACAddress-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetMACAddress-response>)))
  "Returns string type for a service object of type '<GetMACAddress-response>"
  "kortex_driver/GetMACAddressResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetMACAddress-response)))
  "Returns string type for a service object of type 'GetMACAddress-response"
  "kortex_driver/GetMACAddressResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetMACAddress-response>)))
  "Returns md5sum for a message object of type '<GetMACAddress-response>"
  "bf0ce987b74a5e470bf1deeae1772263")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetMACAddress-response)))
  "Returns md5sum for a message object of type 'GetMACAddress-response"
  "bf0ce987b74a5e470bf1deeae1772263")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetMACAddress-response>)))
  "Returns full string definition for message of type '<GetMACAddress-response>"
  (cl:format cl:nil "MACAddress output~%~%================================================================================~%MSG: kortex_driver/MACAddress~%~%uint8[] mac_address~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetMACAddress-response)))
  "Returns full string definition for message of type 'GetMACAddress-response"
  (cl:format cl:nil "MACAddress output~%~%================================================================================~%MSG: kortex_driver/MACAddress~%~%uint8[] mac_address~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetMACAddress-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetMACAddress-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetMACAddress-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetMACAddress)))
  'GetMACAddress-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetMACAddress)))
  'GetMACAddress-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetMACAddress)))
  "Returns string type for a service object of type '<GetMACAddress>"
  "kortex_driver/GetMACAddress")
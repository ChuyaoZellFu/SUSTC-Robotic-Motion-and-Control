; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude GetPartNumber-request.msg.html

(cl:defclass <GetPartNumber-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass GetPartNumber-request (<GetPartNumber-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetPartNumber-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetPartNumber-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetPartNumber-request> is deprecated: use kortex_driver-srv:GetPartNumber-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <GetPartNumber-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetPartNumber-request>) ostream)
  "Serializes a message object of type '<GetPartNumber-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetPartNumber-request>) istream)
  "Deserializes a message object of type '<GetPartNumber-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetPartNumber-request>)))
  "Returns string type for a service object of type '<GetPartNumber-request>"
  "kortex_driver/GetPartNumberRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetPartNumber-request)))
  "Returns string type for a service object of type 'GetPartNumber-request"
  "kortex_driver/GetPartNumberRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetPartNumber-request>)))
  "Returns md5sum for a message object of type '<GetPartNumber-request>"
  "5989c9475bfe9b2d805af628c388fd87")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetPartNumber-request)))
  "Returns md5sum for a message object of type 'GetPartNumber-request"
  "5989c9475bfe9b2d805af628c388fd87")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetPartNumber-request>)))
  "Returns full string definition for message of type '<GetPartNumber-request>"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetPartNumber-request)))
  "Returns full string definition for message of type 'GetPartNumber-request"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetPartNumber-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetPartNumber-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetPartNumber-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude GetPartNumber-response.msg.html

(cl:defclass <GetPartNumber-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:PartNumber
    :initform (cl:make-instance 'kortex_driver-msg:PartNumber)))
)

(cl:defclass GetPartNumber-response (<GetPartNumber-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetPartNumber-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetPartNumber-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetPartNumber-response> is deprecated: use kortex_driver-srv:GetPartNumber-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <GetPartNumber-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetPartNumber-response>) ostream)
  "Serializes a message object of type '<GetPartNumber-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetPartNumber-response>) istream)
  "Deserializes a message object of type '<GetPartNumber-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetPartNumber-response>)))
  "Returns string type for a service object of type '<GetPartNumber-response>"
  "kortex_driver/GetPartNumberResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetPartNumber-response)))
  "Returns string type for a service object of type 'GetPartNumber-response"
  "kortex_driver/GetPartNumberResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetPartNumber-response>)))
  "Returns md5sum for a message object of type '<GetPartNumber-response>"
  "5989c9475bfe9b2d805af628c388fd87")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetPartNumber-response)))
  "Returns md5sum for a message object of type 'GetPartNumber-response"
  "5989c9475bfe9b2d805af628c388fd87")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetPartNumber-response>)))
  "Returns full string definition for message of type '<GetPartNumber-response>"
  (cl:format cl:nil "PartNumber output~%~%================================================================================~%MSG: kortex_driver/PartNumber~%~%string part_number~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetPartNumber-response)))
  "Returns full string definition for message of type 'GetPartNumber-response"
  (cl:format cl:nil "PartNumber output~%~%================================================================================~%MSG: kortex_driver/PartNumber~%~%string part_number~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetPartNumber-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetPartNumber-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetPartNumber-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetPartNumber)))
  'GetPartNumber-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetPartNumber)))
  'GetPartNumber-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetPartNumber)))
  "Returns string type for a service object of type '<GetPartNumber>"
  "kortex_driver/GetPartNumber")
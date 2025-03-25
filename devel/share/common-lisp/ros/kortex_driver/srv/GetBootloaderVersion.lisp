; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude GetBootloaderVersion-request.msg.html

(cl:defclass <GetBootloaderVersion-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass GetBootloaderVersion-request (<GetBootloaderVersion-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetBootloaderVersion-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetBootloaderVersion-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetBootloaderVersion-request> is deprecated: use kortex_driver-srv:GetBootloaderVersion-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <GetBootloaderVersion-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetBootloaderVersion-request>) ostream)
  "Serializes a message object of type '<GetBootloaderVersion-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetBootloaderVersion-request>) istream)
  "Deserializes a message object of type '<GetBootloaderVersion-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetBootloaderVersion-request>)))
  "Returns string type for a service object of type '<GetBootloaderVersion-request>"
  "kortex_driver/GetBootloaderVersionRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetBootloaderVersion-request)))
  "Returns string type for a service object of type 'GetBootloaderVersion-request"
  "kortex_driver/GetBootloaderVersionRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetBootloaderVersion-request>)))
  "Returns md5sum for a message object of type '<GetBootloaderVersion-request>"
  "e674f344ecc5f8e12f3233bf3b7cbeaa")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetBootloaderVersion-request)))
  "Returns md5sum for a message object of type 'GetBootloaderVersion-request"
  "e674f344ecc5f8e12f3233bf3b7cbeaa")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetBootloaderVersion-request>)))
  "Returns full string definition for message of type '<GetBootloaderVersion-request>"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetBootloaderVersion-request)))
  "Returns full string definition for message of type 'GetBootloaderVersion-request"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetBootloaderVersion-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetBootloaderVersion-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetBootloaderVersion-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude GetBootloaderVersion-response.msg.html

(cl:defclass <GetBootloaderVersion-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:BootloaderVersion
    :initform (cl:make-instance 'kortex_driver-msg:BootloaderVersion)))
)

(cl:defclass GetBootloaderVersion-response (<GetBootloaderVersion-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetBootloaderVersion-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetBootloaderVersion-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetBootloaderVersion-response> is deprecated: use kortex_driver-srv:GetBootloaderVersion-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <GetBootloaderVersion-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetBootloaderVersion-response>) ostream)
  "Serializes a message object of type '<GetBootloaderVersion-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetBootloaderVersion-response>) istream)
  "Deserializes a message object of type '<GetBootloaderVersion-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetBootloaderVersion-response>)))
  "Returns string type for a service object of type '<GetBootloaderVersion-response>"
  "kortex_driver/GetBootloaderVersionResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetBootloaderVersion-response)))
  "Returns string type for a service object of type 'GetBootloaderVersion-response"
  "kortex_driver/GetBootloaderVersionResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetBootloaderVersion-response>)))
  "Returns md5sum for a message object of type '<GetBootloaderVersion-response>"
  "e674f344ecc5f8e12f3233bf3b7cbeaa")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetBootloaderVersion-response)))
  "Returns md5sum for a message object of type 'GetBootloaderVersion-response"
  "e674f344ecc5f8e12f3233bf3b7cbeaa")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetBootloaderVersion-response>)))
  "Returns full string definition for message of type '<GetBootloaderVersion-response>"
  (cl:format cl:nil "BootloaderVersion output~%~%================================================================================~%MSG: kortex_driver/BootloaderVersion~%~%uint32 bootloader_version~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetBootloaderVersion-response)))
  "Returns full string definition for message of type 'GetBootloaderVersion-response"
  (cl:format cl:nil "BootloaderVersion output~%~%================================================================================~%MSG: kortex_driver/BootloaderVersion~%~%uint32 bootloader_version~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetBootloaderVersion-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetBootloaderVersion-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetBootloaderVersion-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetBootloaderVersion)))
  'GetBootloaderVersion-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetBootloaderVersion)))
  'GetBootloaderVersion-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetBootloaderVersion)))
  "Returns string type for a service object of type '<GetBootloaderVersion>"
  "kortex_driver/GetBootloaderVersion")
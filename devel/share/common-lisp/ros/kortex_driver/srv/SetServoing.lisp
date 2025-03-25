; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude SetServoing-request.msg.html

(cl:defclass <SetServoing-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:Servoing
    :initform (cl:make-instance 'kortex_driver-msg:Servoing)))
)

(cl:defclass SetServoing-request (<SetServoing-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetServoing-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetServoing-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SetServoing-request> is deprecated: use kortex_driver-srv:SetServoing-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <SetServoing-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetServoing-request>) ostream)
  "Serializes a message object of type '<SetServoing-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetServoing-request>) istream)
  "Deserializes a message object of type '<SetServoing-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetServoing-request>)))
  "Returns string type for a service object of type '<SetServoing-request>"
  "kortex_driver/SetServoingRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetServoing-request)))
  "Returns string type for a service object of type 'SetServoing-request"
  "kortex_driver/SetServoingRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetServoing-request>)))
  "Returns md5sum for a message object of type '<SetServoing-request>"
  "7ccd6a6c56b1642433541444d0e2ab2f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetServoing-request)))
  "Returns md5sum for a message object of type 'SetServoing-request"
  "7ccd6a6c56b1642433541444d0e2ab2f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetServoing-request>)))
  "Returns full string definition for message of type '<SetServoing-request>"
  (cl:format cl:nil "Servoing input~%~%================================================================================~%MSG: kortex_driver/Servoing~%~%bool enabled~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetServoing-request)))
  "Returns full string definition for message of type 'SetServoing-request"
  (cl:format cl:nil "Servoing input~%~%================================================================================~%MSG: kortex_driver/Servoing~%~%bool enabled~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetServoing-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetServoing-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetServoing-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude SetServoing-response.msg.html

(cl:defclass <SetServoing-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass SetServoing-response (<SetServoing-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetServoing-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetServoing-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SetServoing-response> is deprecated: use kortex_driver-srv:SetServoing-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <SetServoing-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetServoing-response>) ostream)
  "Serializes a message object of type '<SetServoing-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetServoing-response>) istream)
  "Deserializes a message object of type '<SetServoing-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetServoing-response>)))
  "Returns string type for a service object of type '<SetServoing-response>"
  "kortex_driver/SetServoingResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetServoing-response)))
  "Returns string type for a service object of type 'SetServoing-response"
  "kortex_driver/SetServoingResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetServoing-response>)))
  "Returns md5sum for a message object of type '<SetServoing-response>"
  "7ccd6a6c56b1642433541444d0e2ab2f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetServoing-response)))
  "Returns md5sum for a message object of type 'SetServoing-response"
  "7ccd6a6c56b1642433541444d0e2ab2f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetServoing-response>)))
  "Returns full string definition for message of type '<SetServoing-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetServoing-response)))
  "Returns full string definition for message of type 'SetServoing-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetServoing-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetServoing-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetServoing-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetServoing)))
  'SetServoing-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetServoing)))
  'SetServoing-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetServoing)))
  "Returns string type for a service object of type '<SetServoing>"
  "kortex_driver/SetServoing")
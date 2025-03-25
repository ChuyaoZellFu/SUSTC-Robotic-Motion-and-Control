; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude RestoreFactoryProductConfiguration-request.msg.html

(cl:defclass <RestoreFactoryProductConfiguration-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass RestoreFactoryProductConfiguration-request (<RestoreFactoryProductConfiguration-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RestoreFactoryProductConfiguration-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RestoreFactoryProductConfiguration-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<RestoreFactoryProductConfiguration-request> is deprecated: use kortex_driver-srv:RestoreFactoryProductConfiguration-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <RestoreFactoryProductConfiguration-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RestoreFactoryProductConfiguration-request>) ostream)
  "Serializes a message object of type '<RestoreFactoryProductConfiguration-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RestoreFactoryProductConfiguration-request>) istream)
  "Deserializes a message object of type '<RestoreFactoryProductConfiguration-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RestoreFactoryProductConfiguration-request>)))
  "Returns string type for a service object of type '<RestoreFactoryProductConfiguration-request>"
  "kortex_driver/RestoreFactoryProductConfigurationRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RestoreFactoryProductConfiguration-request)))
  "Returns string type for a service object of type 'RestoreFactoryProductConfiguration-request"
  "kortex_driver/RestoreFactoryProductConfigurationRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RestoreFactoryProductConfiguration-request>)))
  "Returns md5sum for a message object of type '<RestoreFactoryProductConfiguration-request>"
  "f335b819dc59099fe3124b36f140ad07")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RestoreFactoryProductConfiguration-request)))
  "Returns md5sum for a message object of type 'RestoreFactoryProductConfiguration-request"
  "f335b819dc59099fe3124b36f140ad07")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RestoreFactoryProductConfiguration-request>)))
  "Returns full string definition for message of type '<RestoreFactoryProductConfiguration-request>"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RestoreFactoryProductConfiguration-request)))
  "Returns full string definition for message of type 'RestoreFactoryProductConfiguration-request"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RestoreFactoryProductConfiguration-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RestoreFactoryProductConfiguration-request>))
  "Converts a ROS message object to a list"
  (cl:list 'RestoreFactoryProductConfiguration-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude RestoreFactoryProductConfiguration-response.msg.html

(cl:defclass <RestoreFactoryProductConfiguration-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass RestoreFactoryProductConfiguration-response (<RestoreFactoryProductConfiguration-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RestoreFactoryProductConfiguration-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RestoreFactoryProductConfiguration-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<RestoreFactoryProductConfiguration-response> is deprecated: use kortex_driver-srv:RestoreFactoryProductConfiguration-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <RestoreFactoryProductConfiguration-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RestoreFactoryProductConfiguration-response>) ostream)
  "Serializes a message object of type '<RestoreFactoryProductConfiguration-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RestoreFactoryProductConfiguration-response>) istream)
  "Deserializes a message object of type '<RestoreFactoryProductConfiguration-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RestoreFactoryProductConfiguration-response>)))
  "Returns string type for a service object of type '<RestoreFactoryProductConfiguration-response>"
  "kortex_driver/RestoreFactoryProductConfigurationResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RestoreFactoryProductConfiguration-response)))
  "Returns string type for a service object of type 'RestoreFactoryProductConfiguration-response"
  "kortex_driver/RestoreFactoryProductConfigurationResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RestoreFactoryProductConfiguration-response>)))
  "Returns md5sum for a message object of type '<RestoreFactoryProductConfiguration-response>"
  "f335b819dc59099fe3124b36f140ad07")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RestoreFactoryProductConfiguration-response)))
  "Returns md5sum for a message object of type 'RestoreFactoryProductConfiguration-response"
  "f335b819dc59099fe3124b36f140ad07")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RestoreFactoryProductConfiguration-response>)))
  "Returns full string definition for message of type '<RestoreFactoryProductConfiguration-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RestoreFactoryProductConfiguration-response)))
  "Returns full string definition for message of type 'RestoreFactoryProductConfiguration-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RestoreFactoryProductConfiguration-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RestoreFactoryProductConfiguration-response>))
  "Converts a ROS message object to a list"
  (cl:list 'RestoreFactoryProductConfiguration-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'RestoreFactoryProductConfiguration)))
  'RestoreFactoryProductConfiguration-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'RestoreFactoryProductConfiguration)))
  'RestoreFactoryProductConfiguration-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RestoreFactoryProductConfiguration)))
  "Returns string type for a service object of type '<RestoreFactoryProductConfiguration>"
  "kortex_driver/RestoreFactoryProductConfiguration")
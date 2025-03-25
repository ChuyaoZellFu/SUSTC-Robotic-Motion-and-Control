; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude Base_ClearFaults-request.msg.html

(cl:defclass <Base_ClearFaults-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass Base_ClearFaults-request (<Base_ClearFaults-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Base_ClearFaults-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Base_ClearFaults-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<Base_ClearFaults-request> is deprecated: use kortex_driver-srv:Base_ClearFaults-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <Base_ClearFaults-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Base_ClearFaults-request>) ostream)
  "Serializes a message object of type '<Base_ClearFaults-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Base_ClearFaults-request>) istream)
  "Deserializes a message object of type '<Base_ClearFaults-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Base_ClearFaults-request>)))
  "Returns string type for a service object of type '<Base_ClearFaults-request>"
  "kortex_driver/Base_ClearFaultsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Base_ClearFaults-request)))
  "Returns string type for a service object of type 'Base_ClearFaults-request"
  "kortex_driver/Base_ClearFaultsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Base_ClearFaults-request>)))
  "Returns md5sum for a message object of type '<Base_ClearFaults-request>"
  "f335b819dc59099fe3124b36f140ad07")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Base_ClearFaults-request)))
  "Returns md5sum for a message object of type 'Base_ClearFaults-request"
  "f335b819dc59099fe3124b36f140ad07")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Base_ClearFaults-request>)))
  "Returns full string definition for message of type '<Base_ClearFaults-request>"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Base_ClearFaults-request)))
  "Returns full string definition for message of type 'Base_ClearFaults-request"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Base_ClearFaults-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Base_ClearFaults-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Base_ClearFaults-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude Base_ClearFaults-response.msg.html

(cl:defclass <Base_ClearFaults-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass Base_ClearFaults-response (<Base_ClearFaults-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Base_ClearFaults-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Base_ClearFaults-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<Base_ClearFaults-response> is deprecated: use kortex_driver-srv:Base_ClearFaults-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <Base_ClearFaults-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Base_ClearFaults-response>) ostream)
  "Serializes a message object of type '<Base_ClearFaults-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Base_ClearFaults-response>) istream)
  "Deserializes a message object of type '<Base_ClearFaults-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Base_ClearFaults-response>)))
  "Returns string type for a service object of type '<Base_ClearFaults-response>"
  "kortex_driver/Base_ClearFaultsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Base_ClearFaults-response)))
  "Returns string type for a service object of type 'Base_ClearFaults-response"
  "kortex_driver/Base_ClearFaultsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Base_ClearFaults-response>)))
  "Returns md5sum for a message object of type '<Base_ClearFaults-response>"
  "f335b819dc59099fe3124b36f140ad07")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Base_ClearFaults-response)))
  "Returns md5sum for a message object of type 'Base_ClearFaults-response"
  "f335b819dc59099fe3124b36f140ad07")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Base_ClearFaults-response>)))
  "Returns full string definition for message of type '<Base_ClearFaults-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Base_ClearFaults-response)))
  "Returns full string definition for message of type 'Base_ClearFaults-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Base_ClearFaults-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Base_ClearFaults-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Base_ClearFaults-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Base_ClearFaults)))
  'Base_ClearFaults-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Base_ClearFaults)))
  'Base_ClearFaults-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Base_ClearFaults)))
  "Returns string type for a service object of type '<Base_ClearFaults>"
  "kortex_driver/Base_ClearFaults")
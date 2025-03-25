; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude ClearAllSafetyStatus-request.msg.html

(cl:defclass <ClearAllSafetyStatus-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass ClearAllSafetyStatus-request (<ClearAllSafetyStatus-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ClearAllSafetyStatus-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ClearAllSafetyStatus-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<ClearAllSafetyStatus-request> is deprecated: use kortex_driver-srv:ClearAllSafetyStatus-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <ClearAllSafetyStatus-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ClearAllSafetyStatus-request>) ostream)
  "Serializes a message object of type '<ClearAllSafetyStatus-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ClearAllSafetyStatus-request>) istream)
  "Deserializes a message object of type '<ClearAllSafetyStatus-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ClearAllSafetyStatus-request>)))
  "Returns string type for a service object of type '<ClearAllSafetyStatus-request>"
  "kortex_driver/ClearAllSafetyStatusRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ClearAllSafetyStatus-request)))
  "Returns string type for a service object of type 'ClearAllSafetyStatus-request"
  "kortex_driver/ClearAllSafetyStatusRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ClearAllSafetyStatus-request>)))
  "Returns md5sum for a message object of type '<ClearAllSafetyStatus-request>"
  "f335b819dc59099fe3124b36f140ad07")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ClearAllSafetyStatus-request)))
  "Returns md5sum for a message object of type 'ClearAllSafetyStatus-request"
  "f335b819dc59099fe3124b36f140ad07")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ClearAllSafetyStatus-request>)))
  "Returns full string definition for message of type '<ClearAllSafetyStatus-request>"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ClearAllSafetyStatus-request)))
  "Returns full string definition for message of type 'ClearAllSafetyStatus-request"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ClearAllSafetyStatus-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ClearAllSafetyStatus-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ClearAllSafetyStatus-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude ClearAllSafetyStatus-response.msg.html

(cl:defclass <ClearAllSafetyStatus-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass ClearAllSafetyStatus-response (<ClearAllSafetyStatus-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ClearAllSafetyStatus-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ClearAllSafetyStatus-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<ClearAllSafetyStatus-response> is deprecated: use kortex_driver-srv:ClearAllSafetyStatus-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <ClearAllSafetyStatus-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ClearAllSafetyStatus-response>) ostream)
  "Serializes a message object of type '<ClearAllSafetyStatus-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ClearAllSafetyStatus-response>) istream)
  "Deserializes a message object of type '<ClearAllSafetyStatus-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ClearAllSafetyStatus-response>)))
  "Returns string type for a service object of type '<ClearAllSafetyStatus-response>"
  "kortex_driver/ClearAllSafetyStatusResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ClearAllSafetyStatus-response)))
  "Returns string type for a service object of type 'ClearAllSafetyStatus-response"
  "kortex_driver/ClearAllSafetyStatusResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ClearAllSafetyStatus-response>)))
  "Returns md5sum for a message object of type '<ClearAllSafetyStatus-response>"
  "f335b819dc59099fe3124b36f140ad07")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ClearAllSafetyStatus-response)))
  "Returns md5sum for a message object of type 'ClearAllSafetyStatus-response"
  "f335b819dc59099fe3124b36f140ad07")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ClearAllSafetyStatus-response>)))
  "Returns full string definition for message of type '<ClearAllSafetyStatus-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ClearAllSafetyStatus-response)))
  "Returns full string definition for message of type 'ClearAllSafetyStatus-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ClearAllSafetyStatus-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ClearAllSafetyStatus-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ClearAllSafetyStatus-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ClearAllSafetyStatus)))
  'ClearAllSafetyStatus-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ClearAllSafetyStatus)))
  'ClearAllSafetyStatus-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ClearAllSafetyStatus)))
  "Returns string type for a service object of type '<ClearAllSafetyStatus>"
  "kortex_driver/ClearAllSafetyStatus")
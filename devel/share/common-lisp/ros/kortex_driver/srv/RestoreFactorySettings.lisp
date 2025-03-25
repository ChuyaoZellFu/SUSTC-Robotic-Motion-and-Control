; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude RestoreFactorySettings-request.msg.html

(cl:defclass <RestoreFactorySettings-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass RestoreFactorySettings-request (<RestoreFactorySettings-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RestoreFactorySettings-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RestoreFactorySettings-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<RestoreFactorySettings-request> is deprecated: use kortex_driver-srv:RestoreFactorySettings-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <RestoreFactorySettings-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RestoreFactorySettings-request>) ostream)
  "Serializes a message object of type '<RestoreFactorySettings-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RestoreFactorySettings-request>) istream)
  "Deserializes a message object of type '<RestoreFactorySettings-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RestoreFactorySettings-request>)))
  "Returns string type for a service object of type '<RestoreFactorySettings-request>"
  "kortex_driver/RestoreFactorySettingsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RestoreFactorySettings-request)))
  "Returns string type for a service object of type 'RestoreFactorySettings-request"
  "kortex_driver/RestoreFactorySettingsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RestoreFactorySettings-request>)))
  "Returns md5sum for a message object of type '<RestoreFactorySettings-request>"
  "f335b819dc59099fe3124b36f140ad07")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RestoreFactorySettings-request)))
  "Returns md5sum for a message object of type 'RestoreFactorySettings-request"
  "f335b819dc59099fe3124b36f140ad07")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RestoreFactorySettings-request>)))
  "Returns full string definition for message of type '<RestoreFactorySettings-request>"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RestoreFactorySettings-request)))
  "Returns full string definition for message of type 'RestoreFactorySettings-request"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RestoreFactorySettings-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RestoreFactorySettings-request>))
  "Converts a ROS message object to a list"
  (cl:list 'RestoreFactorySettings-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude RestoreFactorySettings-response.msg.html

(cl:defclass <RestoreFactorySettings-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass RestoreFactorySettings-response (<RestoreFactorySettings-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RestoreFactorySettings-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RestoreFactorySettings-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<RestoreFactorySettings-response> is deprecated: use kortex_driver-srv:RestoreFactorySettings-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <RestoreFactorySettings-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RestoreFactorySettings-response>) ostream)
  "Serializes a message object of type '<RestoreFactorySettings-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RestoreFactorySettings-response>) istream)
  "Deserializes a message object of type '<RestoreFactorySettings-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RestoreFactorySettings-response>)))
  "Returns string type for a service object of type '<RestoreFactorySettings-response>"
  "kortex_driver/RestoreFactorySettingsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RestoreFactorySettings-response)))
  "Returns string type for a service object of type 'RestoreFactorySettings-response"
  "kortex_driver/RestoreFactorySettingsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RestoreFactorySettings-response>)))
  "Returns md5sum for a message object of type '<RestoreFactorySettings-response>"
  "f335b819dc59099fe3124b36f140ad07")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RestoreFactorySettings-response)))
  "Returns md5sum for a message object of type 'RestoreFactorySettings-response"
  "f335b819dc59099fe3124b36f140ad07")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RestoreFactorySettings-response>)))
  "Returns full string definition for message of type '<RestoreFactorySettings-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RestoreFactorySettings-response)))
  "Returns full string definition for message of type 'RestoreFactorySettings-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RestoreFactorySettings-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RestoreFactorySettings-response>))
  "Converts a ROS message object to a list"
  (cl:list 'RestoreFactorySettings-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'RestoreFactorySettings)))
  'RestoreFactorySettings-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'RestoreFactorySettings)))
  'RestoreFactorySettings-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RestoreFactorySettings)))
  "Returns string type for a service object of type '<RestoreFactorySettings>"
  "kortex_driver/RestoreFactorySettings")
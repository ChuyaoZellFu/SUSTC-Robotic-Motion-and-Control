; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude RebootRequest-request.msg.html

(cl:defclass <RebootRequest-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:RebootRqst
    :initform (cl:make-instance 'kortex_driver-msg:RebootRqst)))
)

(cl:defclass RebootRequest-request (<RebootRequest-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RebootRequest-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RebootRequest-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<RebootRequest-request> is deprecated: use kortex_driver-srv:RebootRequest-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <RebootRequest-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RebootRequest-request>) ostream)
  "Serializes a message object of type '<RebootRequest-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RebootRequest-request>) istream)
  "Deserializes a message object of type '<RebootRequest-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RebootRequest-request>)))
  "Returns string type for a service object of type '<RebootRequest-request>"
  "kortex_driver/RebootRequestRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RebootRequest-request)))
  "Returns string type for a service object of type 'RebootRequest-request"
  "kortex_driver/RebootRequestRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RebootRequest-request>)))
  "Returns md5sum for a message object of type '<RebootRequest-request>"
  "7587d0460f4d1dff902eaf7ca6589a94")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RebootRequest-request)))
  "Returns md5sum for a message object of type 'RebootRequest-request"
  "7587d0460f4d1dff902eaf7ca6589a94")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RebootRequest-request>)))
  "Returns full string definition for message of type '<RebootRequest-request>"
  (cl:format cl:nil "RebootRqst input~%~%================================================================================~%MSG: kortex_driver/RebootRqst~%~%uint32 delay~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RebootRequest-request)))
  "Returns full string definition for message of type 'RebootRequest-request"
  (cl:format cl:nil "RebootRqst input~%~%================================================================================~%MSG: kortex_driver/RebootRqst~%~%uint32 delay~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RebootRequest-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RebootRequest-request>))
  "Converts a ROS message object to a list"
  (cl:list 'RebootRequest-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude RebootRequest-response.msg.html

(cl:defclass <RebootRequest-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass RebootRequest-response (<RebootRequest-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RebootRequest-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RebootRequest-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<RebootRequest-response> is deprecated: use kortex_driver-srv:RebootRequest-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <RebootRequest-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RebootRequest-response>) ostream)
  "Serializes a message object of type '<RebootRequest-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RebootRequest-response>) istream)
  "Deserializes a message object of type '<RebootRequest-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RebootRequest-response>)))
  "Returns string type for a service object of type '<RebootRequest-response>"
  "kortex_driver/RebootRequestResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RebootRequest-response)))
  "Returns string type for a service object of type 'RebootRequest-response"
  "kortex_driver/RebootRequestResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RebootRequest-response>)))
  "Returns md5sum for a message object of type '<RebootRequest-response>"
  "7587d0460f4d1dff902eaf7ca6589a94")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RebootRequest-response)))
  "Returns md5sum for a message object of type 'RebootRequest-response"
  "7587d0460f4d1dff902eaf7ca6589a94")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RebootRequest-response>)))
  "Returns full string definition for message of type '<RebootRequest-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RebootRequest-response)))
  "Returns full string definition for message of type 'RebootRequest-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RebootRequest-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RebootRequest-response>))
  "Converts a ROS message object to a list"
  (cl:list 'RebootRequest-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'RebootRequest)))
  'RebootRequest-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'RebootRequest)))
  'RebootRequest-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RebootRequest)))
  "Returns string type for a service object of type '<RebootRequest>"
  "kortex_driver/RebootRequest")
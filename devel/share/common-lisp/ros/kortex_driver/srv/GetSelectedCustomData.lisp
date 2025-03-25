; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude GetSelectedCustomData-request.msg.html

(cl:defclass <GetSelectedCustomData-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass GetSelectedCustomData-request (<GetSelectedCustomData-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetSelectedCustomData-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetSelectedCustomData-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetSelectedCustomData-request> is deprecated: use kortex_driver-srv:GetSelectedCustomData-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <GetSelectedCustomData-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetSelectedCustomData-request>) ostream)
  "Serializes a message object of type '<GetSelectedCustomData-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetSelectedCustomData-request>) istream)
  "Deserializes a message object of type '<GetSelectedCustomData-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetSelectedCustomData-request>)))
  "Returns string type for a service object of type '<GetSelectedCustomData-request>"
  "kortex_driver/GetSelectedCustomDataRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetSelectedCustomData-request)))
  "Returns string type for a service object of type 'GetSelectedCustomData-request"
  "kortex_driver/GetSelectedCustomDataRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetSelectedCustomData-request>)))
  "Returns md5sum for a message object of type '<GetSelectedCustomData-request>"
  "d4420814b14d58afc7b7b98f4fdd7907")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetSelectedCustomData-request)))
  "Returns md5sum for a message object of type 'GetSelectedCustomData-request"
  "d4420814b14d58afc7b7b98f4fdd7907")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetSelectedCustomData-request>)))
  "Returns full string definition for message of type '<GetSelectedCustomData-request>"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetSelectedCustomData-request)))
  "Returns full string definition for message of type 'GetSelectedCustomData-request"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetSelectedCustomData-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetSelectedCustomData-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetSelectedCustomData-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude GetSelectedCustomData-response.msg.html

(cl:defclass <GetSelectedCustomData-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:CustomDataSelection
    :initform (cl:make-instance 'kortex_driver-msg:CustomDataSelection)))
)

(cl:defclass GetSelectedCustomData-response (<GetSelectedCustomData-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetSelectedCustomData-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetSelectedCustomData-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetSelectedCustomData-response> is deprecated: use kortex_driver-srv:GetSelectedCustomData-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <GetSelectedCustomData-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetSelectedCustomData-response>) ostream)
  "Serializes a message object of type '<GetSelectedCustomData-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetSelectedCustomData-response>) istream)
  "Deserializes a message object of type '<GetSelectedCustomData-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetSelectedCustomData-response>)))
  "Returns string type for a service object of type '<GetSelectedCustomData-response>"
  "kortex_driver/GetSelectedCustomDataResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetSelectedCustomData-response)))
  "Returns string type for a service object of type 'GetSelectedCustomData-response"
  "kortex_driver/GetSelectedCustomDataResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetSelectedCustomData-response>)))
  "Returns md5sum for a message object of type '<GetSelectedCustomData-response>"
  "d4420814b14d58afc7b7b98f4fdd7907")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetSelectedCustomData-response)))
  "Returns md5sum for a message object of type 'GetSelectedCustomData-response"
  "d4420814b14d58afc7b7b98f4fdd7907")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetSelectedCustomData-response>)))
  "Returns full string definition for message of type '<GetSelectedCustomData-response>"
  (cl:format cl:nil "CustomDataSelection output~%~%================================================================================~%MSG: kortex_driver/CustomDataSelection~%~%uint32[] channel~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetSelectedCustomData-response)))
  "Returns full string definition for message of type 'GetSelectedCustomData-response"
  (cl:format cl:nil "CustomDataSelection output~%~%================================================================================~%MSG: kortex_driver/CustomDataSelection~%~%uint32[] channel~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetSelectedCustomData-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetSelectedCustomData-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetSelectedCustomData-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetSelectedCustomData)))
  'GetSelectedCustomData-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetSelectedCustomData)))
  'GetSelectedCustomData-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetSelectedCustomData)))
  "Returns string type for a service object of type '<GetSelectedCustomData>"
  "kortex_driver/GetSelectedCustomData")
; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude SelectCustomData-request.msg.html

(cl:defclass <SelectCustomData-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:CustomDataSelection
    :initform (cl:make-instance 'kortex_driver-msg:CustomDataSelection)))
)

(cl:defclass SelectCustomData-request (<SelectCustomData-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SelectCustomData-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SelectCustomData-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SelectCustomData-request> is deprecated: use kortex_driver-srv:SelectCustomData-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <SelectCustomData-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SelectCustomData-request>) ostream)
  "Serializes a message object of type '<SelectCustomData-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SelectCustomData-request>) istream)
  "Deserializes a message object of type '<SelectCustomData-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SelectCustomData-request>)))
  "Returns string type for a service object of type '<SelectCustomData-request>"
  "kortex_driver/SelectCustomDataRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SelectCustomData-request)))
  "Returns string type for a service object of type 'SelectCustomData-request"
  "kortex_driver/SelectCustomDataRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SelectCustomData-request>)))
  "Returns md5sum for a message object of type '<SelectCustomData-request>"
  "6fbafd6efc00928eb53e0c769fef2624")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SelectCustomData-request)))
  "Returns md5sum for a message object of type 'SelectCustomData-request"
  "6fbafd6efc00928eb53e0c769fef2624")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SelectCustomData-request>)))
  "Returns full string definition for message of type '<SelectCustomData-request>"
  (cl:format cl:nil "CustomDataSelection input~%~%================================================================================~%MSG: kortex_driver/CustomDataSelection~%~%uint32[] channel~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SelectCustomData-request)))
  "Returns full string definition for message of type 'SelectCustomData-request"
  (cl:format cl:nil "CustomDataSelection input~%~%================================================================================~%MSG: kortex_driver/CustomDataSelection~%~%uint32[] channel~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SelectCustomData-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SelectCustomData-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SelectCustomData-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude SelectCustomData-response.msg.html

(cl:defclass <SelectCustomData-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass SelectCustomData-response (<SelectCustomData-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SelectCustomData-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SelectCustomData-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SelectCustomData-response> is deprecated: use kortex_driver-srv:SelectCustomData-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <SelectCustomData-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SelectCustomData-response>) ostream)
  "Serializes a message object of type '<SelectCustomData-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SelectCustomData-response>) istream)
  "Deserializes a message object of type '<SelectCustomData-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SelectCustomData-response>)))
  "Returns string type for a service object of type '<SelectCustomData-response>"
  "kortex_driver/SelectCustomDataResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SelectCustomData-response)))
  "Returns string type for a service object of type 'SelectCustomData-response"
  "kortex_driver/SelectCustomDataResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SelectCustomData-response>)))
  "Returns md5sum for a message object of type '<SelectCustomData-response>"
  "6fbafd6efc00928eb53e0c769fef2624")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SelectCustomData-response)))
  "Returns md5sum for a message object of type 'SelectCustomData-response"
  "6fbafd6efc00928eb53e0c769fef2624")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SelectCustomData-response>)))
  "Returns full string definition for message of type '<SelectCustomData-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SelectCustomData-response)))
  "Returns full string definition for message of type 'SelectCustomData-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SelectCustomData-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SelectCustomData-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SelectCustomData-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SelectCustomData)))
  'SelectCustomData-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SelectCustomData)))
  'SelectCustomData-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SelectCustomData)))
  "Returns string type for a service object of type '<SelectCustomData>"
  "kortex_driver/SelectCustomData")
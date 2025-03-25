; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude GetPartNumberRevision-request.msg.html

(cl:defclass <GetPartNumberRevision-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass GetPartNumberRevision-request (<GetPartNumberRevision-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetPartNumberRevision-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetPartNumberRevision-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetPartNumberRevision-request> is deprecated: use kortex_driver-srv:GetPartNumberRevision-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <GetPartNumberRevision-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetPartNumberRevision-request>) ostream)
  "Serializes a message object of type '<GetPartNumberRevision-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetPartNumberRevision-request>) istream)
  "Deserializes a message object of type '<GetPartNumberRevision-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetPartNumberRevision-request>)))
  "Returns string type for a service object of type '<GetPartNumberRevision-request>"
  "kortex_driver/GetPartNumberRevisionRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetPartNumberRevision-request)))
  "Returns string type for a service object of type 'GetPartNumberRevision-request"
  "kortex_driver/GetPartNumberRevisionRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetPartNumberRevision-request>)))
  "Returns md5sum for a message object of type '<GetPartNumberRevision-request>"
  "a25670b2402eb62859a04b7bd6a88b8c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetPartNumberRevision-request)))
  "Returns md5sum for a message object of type 'GetPartNumberRevision-request"
  "a25670b2402eb62859a04b7bd6a88b8c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetPartNumberRevision-request>)))
  "Returns full string definition for message of type '<GetPartNumberRevision-request>"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetPartNumberRevision-request)))
  "Returns full string definition for message of type 'GetPartNumberRevision-request"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetPartNumberRevision-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetPartNumberRevision-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetPartNumberRevision-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude GetPartNumberRevision-response.msg.html

(cl:defclass <GetPartNumberRevision-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:PartNumberRevision
    :initform (cl:make-instance 'kortex_driver-msg:PartNumberRevision)))
)

(cl:defclass GetPartNumberRevision-response (<GetPartNumberRevision-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetPartNumberRevision-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetPartNumberRevision-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetPartNumberRevision-response> is deprecated: use kortex_driver-srv:GetPartNumberRevision-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <GetPartNumberRevision-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetPartNumberRevision-response>) ostream)
  "Serializes a message object of type '<GetPartNumberRevision-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetPartNumberRevision-response>) istream)
  "Deserializes a message object of type '<GetPartNumberRevision-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetPartNumberRevision-response>)))
  "Returns string type for a service object of type '<GetPartNumberRevision-response>"
  "kortex_driver/GetPartNumberRevisionResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetPartNumberRevision-response)))
  "Returns string type for a service object of type 'GetPartNumberRevision-response"
  "kortex_driver/GetPartNumberRevisionResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetPartNumberRevision-response>)))
  "Returns md5sum for a message object of type '<GetPartNumberRevision-response>"
  "a25670b2402eb62859a04b7bd6a88b8c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetPartNumberRevision-response)))
  "Returns md5sum for a message object of type 'GetPartNumberRevision-response"
  "a25670b2402eb62859a04b7bd6a88b8c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetPartNumberRevision-response>)))
  "Returns full string definition for message of type '<GetPartNumberRevision-response>"
  (cl:format cl:nil "PartNumberRevision output~%~%================================================================================~%MSG: kortex_driver/PartNumberRevision~%~%string part_number_revision~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetPartNumberRevision-response)))
  "Returns full string definition for message of type 'GetPartNumberRevision-response"
  (cl:format cl:nil "PartNumberRevision output~%~%================================================================================~%MSG: kortex_driver/PartNumberRevision~%~%string part_number_revision~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetPartNumberRevision-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetPartNumberRevision-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetPartNumberRevision-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetPartNumberRevision)))
  'GetPartNumberRevision-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetPartNumberRevision)))
  'GetPartNumberRevision-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetPartNumberRevision)))
  "Returns string type for a service object of type '<GetPartNumberRevision>"
  "kortex_driver/GetPartNumberRevision")
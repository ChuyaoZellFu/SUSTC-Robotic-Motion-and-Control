; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude GetCartesianReferenceFrame-request.msg.html

(cl:defclass <GetCartesianReferenceFrame-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass GetCartesianReferenceFrame-request (<GetCartesianReferenceFrame-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetCartesianReferenceFrame-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetCartesianReferenceFrame-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetCartesianReferenceFrame-request> is deprecated: use kortex_driver-srv:GetCartesianReferenceFrame-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <GetCartesianReferenceFrame-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetCartesianReferenceFrame-request>) ostream)
  "Serializes a message object of type '<GetCartesianReferenceFrame-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetCartesianReferenceFrame-request>) istream)
  "Deserializes a message object of type '<GetCartesianReferenceFrame-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetCartesianReferenceFrame-request>)))
  "Returns string type for a service object of type '<GetCartesianReferenceFrame-request>"
  "kortex_driver/GetCartesianReferenceFrameRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetCartesianReferenceFrame-request)))
  "Returns string type for a service object of type 'GetCartesianReferenceFrame-request"
  "kortex_driver/GetCartesianReferenceFrameRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetCartesianReferenceFrame-request>)))
  "Returns md5sum for a message object of type '<GetCartesianReferenceFrame-request>"
  "fd1f94f7574ce13875b4bba8e911f051")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetCartesianReferenceFrame-request)))
  "Returns md5sum for a message object of type 'GetCartesianReferenceFrame-request"
  "fd1f94f7574ce13875b4bba8e911f051")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetCartesianReferenceFrame-request>)))
  "Returns full string definition for message of type '<GetCartesianReferenceFrame-request>"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetCartesianReferenceFrame-request)))
  "Returns full string definition for message of type 'GetCartesianReferenceFrame-request"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetCartesianReferenceFrame-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetCartesianReferenceFrame-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetCartesianReferenceFrame-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude GetCartesianReferenceFrame-response.msg.html

(cl:defclass <GetCartesianReferenceFrame-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:CartesianReferenceFrameInfo
    :initform (cl:make-instance 'kortex_driver-msg:CartesianReferenceFrameInfo)))
)

(cl:defclass GetCartesianReferenceFrame-response (<GetCartesianReferenceFrame-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetCartesianReferenceFrame-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetCartesianReferenceFrame-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetCartesianReferenceFrame-response> is deprecated: use kortex_driver-srv:GetCartesianReferenceFrame-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <GetCartesianReferenceFrame-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetCartesianReferenceFrame-response>) ostream)
  "Serializes a message object of type '<GetCartesianReferenceFrame-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetCartesianReferenceFrame-response>) istream)
  "Deserializes a message object of type '<GetCartesianReferenceFrame-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetCartesianReferenceFrame-response>)))
  "Returns string type for a service object of type '<GetCartesianReferenceFrame-response>"
  "kortex_driver/GetCartesianReferenceFrameResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetCartesianReferenceFrame-response)))
  "Returns string type for a service object of type 'GetCartesianReferenceFrame-response"
  "kortex_driver/GetCartesianReferenceFrameResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetCartesianReferenceFrame-response>)))
  "Returns md5sum for a message object of type '<GetCartesianReferenceFrame-response>"
  "fd1f94f7574ce13875b4bba8e911f051")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetCartesianReferenceFrame-response)))
  "Returns md5sum for a message object of type 'GetCartesianReferenceFrame-response"
  "fd1f94f7574ce13875b4bba8e911f051")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetCartesianReferenceFrame-response>)))
  "Returns full string definition for message of type '<GetCartesianReferenceFrame-response>"
  (cl:format cl:nil "CartesianReferenceFrameInfo output~%~%================================================================================~%MSG: kortex_driver/CartesianReferenceFrameInfo~%~%uint32 reference_frame~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetCartesianReferenceFrame-response)))
  "Returns full string definition for message of type 'GetCartesianReferenceFrame-response"
  (cl:format cl:nil "CartesianReferenceFrameInfo output~%~%================================================================================~%MSG: kortex_driver/CartesianReferenceFrameInfo~%~%uint32 reference_frame~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetCartesianReferenceFrame-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetCartesianReferenceFrame-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetCartesianReferenceFrame-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetCartesianReferenceFrame)))
  'GetCartesianReferenceFrame-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetCartesianReferenceFrame)))
  'GetCartesianReferenceFrame-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetCartesianReferenceFrame)))
  "Returns string type for a service object of type '<GetCartesianReferenceFrame>"
  "kortex_driver/GetCartesianReferenceFrame")
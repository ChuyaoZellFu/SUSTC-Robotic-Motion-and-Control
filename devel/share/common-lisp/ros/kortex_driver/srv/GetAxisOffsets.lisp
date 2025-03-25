; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude GetAxisOffsets-request.msg.html

(cl:defclass <GetAxisOffsets-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass GetAxisOffsets-request (<GetAxisOffsets-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetAxisOffsets-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetAxisOffsets-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetAxisOffsets-request> is deprecated: use kortex_driver-srv:GetAxisOffsets-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <GetAxisOffsets-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetAxisOffsets-request>) ostream)
  "Serializes a message object of type '<GetAxisOffsets-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetAxisOffsets-request>) istream)
  "Deserializes a message object of type '<GetAxisOffsets-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetAxisOffsets-request>)))
  "Returns string type for a service object of type '<GetAxisOffsets-request>"
  "kortex_driver/GetAxisOffsetsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetAxisOffsets-request)))
  "Returns string type for a service object of type 'GetAxisOffsets-request"
  "kortex_driver/GetAxisOffsetsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetAxisOffsets-request>)))
  "Returns md5sum for a message object of type '<GetAxisOffsets-request>"
  "e0bbfa31b1895cb9c1c9ec4bbac51a98")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetAxisOffsets-request)))
  "Returns md5sum for a message object of type 'GetAxisOffsets-request"
  "e0bbfa31b1895cb9c1c9ec4bbac51a98")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetAxisOffsets-request>)))
  "Returns full string definition for message of type '<GetAxisOffsets-request>"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetAxisOffsets-request)))
  "Returns full string definition for message of type 'GetAxisOffsets-request"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetAxisOffsets-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetAxisOffsets-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetAxisOffsets-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude GetAxisOffsets-response.msg.html

(cl:defclass <GetAxisOffsets-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:AxisOffsets
    :initform (cl:make-instance 'kortex_driver-msg:AxisOffsets)))
)

(cl:defclass GetAxisOffsets-response (<GetAxisOffsets-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetAxisOffsets-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetAxisOffsets-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetAxisOffsets-response> is deprecated: use kortex_driver-srv:GetAxisOffsets-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <GetAxisOffsets-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetAxisOffsets-response>) ostream)
  "Serializes a message object of type '<GetAxisOffsets-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetAxisOffsets-response>) istream)
  "Deserializes a message object of type '<GetAxisOffsets-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetAxisOffsets-response>)))
  "Returns string type for a service object of type '<GetAxisOffsets-response>"
  "kortex_driver/GetAxisOffsetsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetAxisOffsets-response)))
  "Returns string type for a service object of type 'GetAxisOffsets-response"
  "kortex_driver/GetAxisOffsetsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetAxisOffsets-response>)))
  "Returns md5sum for a message object of type '<GetAxisOffsets-response>"
  "e0bbfa31b1895cb9c1c9ec4bbac51a98")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetAxisOffsets-response)))
  "Returns md5sum for a message object of type 'GetAxisOffsets-response"
  "e0bbfa31b1895cb9c1c9ec4bbac51a98")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetAxisOffsets-response>)))
  "Returns full string definition for message of type '<GetAxisOffsets-response>"
  (cl:format cl:nil "AxisOffsets output~%~%================================================================================~%MSG: kortex_driver/AxisOffsets~%~%float32 absolute_offset~%float32 relative_offset~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetAxisOffsets-response)))
  "Returns full string definition for message of type 'GetAxisOffsets-response"
  (cl:format cl:nil "AxisOffsets output~%~%================================================================================~%MSG: kortex_driver/AxisOffsets~%~%float32 absolute_offset~%float32 relative_offset~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetAxisOffsets-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetAxisOffsets-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetAxisOffsets-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetAxisOffsets)))
  'GetAxisOffsets-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetAxisOffsets)))
  'GetAxisOffsets-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetAxisOffsets)))
  "Returns string type for a service object of type '<GetAxisOffsets>"
  "kortex_driver/GetAxisOffsets")
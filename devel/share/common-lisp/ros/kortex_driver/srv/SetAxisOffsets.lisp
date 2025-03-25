; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude SetAxisOffsets-request.msg.html

(cl:defclass <SetAxisOffsets-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:AxisPosition
    :initform (cl:make-instance 'kortex_driver-msg:AxisPosition)))
)

(cl:defclass SetAxisOffsets-request (<SetAxisOffsets-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetAxisOffsets-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetAxisOffsets-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SetAxisOffsets-request> is deprecated: use kortex_driver-srv:SetAxisOffsets-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <SetAxisOffsets-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetAxisOffsets-request>) ostream)
  "Serializes a message object of type '<SetAxisOffsets-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetAxisOffsets-request>) istream)
  "Deserializes a message object of type '<SetAxisOffsets-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetAxisOffsets-request>)))
  "Returns string type for a service object of type '<SetAxisOffsets-request>"
  "kortex_driver/SetAxisOffsetsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetAxisOffsets-request)))
  "Returns string type for a service object of type 'SetAxisOffsets-request"
  "kortex_driver/SetAxisOffsetsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetAxisOffsets-request>)))
  "Returns md5sum for a message object of type '<SetAxisOffsets-request>"
  "b3df8f041f59058a98542ed76394a7b4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetAxisOffsets-request)))
  "Returns md5sum for a message object of type 'SetAxisOffsets-request"
  "b3df8f041f59058a98542ed76394a7b4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetAxisOffsets-request>)))
  "Returns full string definition for message of type '<SetAxisOffsets-request>"
  (cl:format cl:nil "AxisPosition input~%~%================================================================================~%MSG: kortex_driver/AxisPosition~%~%float32 position~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetAxisOffsets-request)))
  "Returns full string definition for message of type 'SetAxisOffsets-request"
  (cl:format cl:nil "AxisPosition input~%~%================================================================================~%MSG: kortex_driver/AxisPosition~%~%float32 position~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetAxisOffsets-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetAxisOffsets-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetAxisOffsets-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude SetAxisOffsets-response.msg.html

(cl:defclass <SetAxisOffsets-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass SetAxisOffsets-response (<SetAxisOffsets-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetAxisOffsets-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetAxisOffsets-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SetAxisOffsets-response> is deprecated: use kortex_driver-srv:SetAxisOffsets-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <SetAxisOffsets-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetAxisOffsets-response>) ostream)
  "Serializes a message object of type '<SetAxisOffsets-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetAxisOffsets-response>) istream)
  "Deserializes a message object of type '<SetAxisOffsets-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetAxisOffsets-response>)))
  "Returns string type for a service object of type '<SetAxisOffsets-response>"
  "kortex_driver/SetAxisOffsetsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetAxisOffsets-response)))
  "Returns string type for a service object of type 'SetAxisOffsets-response"
  "kortex_driver/SetAxisOffsetsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetAxisOffsets-response>)))
  "Returns md5sum for a message object of type '<SetAxisOffsets-response>"
  "b3df8f041f59058a98542ed76394a7b4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetAxisOffsets-response)))
  "Returns md5sum for a message object of type 'SetAxisOffsets-response"
  "b3df8f041f59058a98542ed76394a7b4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetAxisOffsets-response>)))
  "Returns full string definition for message of type '<SetAxisOffsets-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetAxisOffsets-response)))
  "Returns full string definition for message of type 'SetAxisOffsets-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetAxisOffsets-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetAxisOffsets-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetAxisOffsets-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetAxisOffsets)))
  'SetAxisOffsets-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetAxisOffsets)))
  'SetAxisOffsets-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetAxisOffsets)))
  "Returns string type for a service object of type '<SetAxisOffsets>"
  "kortex_driver/SetAxisOffsets")
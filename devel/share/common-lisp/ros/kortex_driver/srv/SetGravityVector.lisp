; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude SetGravityVector-request.msg.html

(cl:defclass <SetGravityVector-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:GravityVector
    :initform (cl:make-instance 'kortex_driver-msg:GravityVector)))
)

(cl:defclass SetGravityVector-request (<SetGravityVector-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetGravityVector-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetGravityVector-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SetGravityVector-request> is deprecated: use kortex_driver-srv:SetGravityVector-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <SetGravityVector-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetGravityVector-request>) ostream)
  "Serializes a message object of type '<SetGravityVector-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetGravityVector-request>) istream)
  "Deserializes a message object of type '<SetGravityVector-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetGravityVector-request>)))
  "Returns string type for a service object of type '<SetGravityVector-request>"
  "kortex_driver/SetGravityVectorRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetGravityVector-request)))
  "Returns string type for a service object of type 'SetGravityVector-request"
  "kortex_driver/SetGravityVectorRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetGravityVector-request>)))
  "Returns md5sum for a message object of type '<SetGravityVector-request>"
  "0af2ee5f9fbb095072dc9d9e57b9318b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetGravityVector-request)))
  "Returns md5sum for a message object of type 'SetGravityVector-request"
  "0af2ee5f9fbb095072dc9d9e57b9318b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetGravityVector-request>)))
  "Returns full string definition for message of type '<SetGravityVector-request>"
  (cl:format cl:nil "GravityVector input~%~%================================================================================~%MSG: kortex_driver/GravityVector~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetGravityVector-request)))
  "Returns full string definition for message of type 'SetGravityVector-request"
  (cl:format cl:nil "GravityVector input~%~%================================================================================~%MSG: kortex_driver/GravityVector~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetGravityVector-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetGravityVector-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetGravityVector-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude SetGravityVector-response.msg.html

(cl:defclass <SetGravityVector-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass SetGravityVector-response (<SetGravityVector-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetGravityVector-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetGravityVector-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SetGravityVector-response> is deprecated: use kortex_driver-srv:SetGravityVector-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <SetGravityVector-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetGravityVector-response>) ostream)
  "Serializes a message object of type '<SetGravityVector-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetGravityVector-response>) istream)
  "Deserializes a message object of type '<SetGravityVector-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetGravityVector-response>)))
  "Returns string type for a service object of type '<SetGravityVector-response>"
  "kortex_driver/SetGravityVectorResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetGravityVector-response)))
  "Returns string type for a service object of type 'SetGravityVector-response"
  "kortex_driver/SetGravityVectorResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetGravityVector-response>)))
  "Returns md5sum for a message object of type '<SetGravityVector-response>"
  "0af2ee5f9fbb095072dc9d9e57b9318b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetGravityVector-response)))
  "Returns md5sum for a message object of type 'SetGravityVector-response"
  "0af2ee5f9fbb095072dc9d9e57b9318b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetGravityVector-response>)))
  "Returns full string definition for message of type '<SetGravityVector-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetGravityVector-response)))
  "Returns full string definition for message of type 'SetGravityVector-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetGravityVector-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetGravityVector-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetGravityVector-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetGravityVector)))
  'SetGravityVector-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetGravityVector)))
  'SetGravityVector-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetGravityVector)))
  "Returns string type for a service object of type '<SetGravityVector>"
  "kortex_driver/SetGravityVector")
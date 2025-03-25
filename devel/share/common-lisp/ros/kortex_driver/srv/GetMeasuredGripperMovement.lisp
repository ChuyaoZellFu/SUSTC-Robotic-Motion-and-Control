; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude GetMeasuredGripperMovement-request.msg.html

(cl:defclass <GetMeasuredGripperMovement-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:GripperRequest
    :initform (cl:make-instance 'kortex_driver-msg:GripperRequest)))
)

(cl:defclass GetMeasuredGripperMovement-request (<GetMeasuredGripperMovement-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetMeasuredGripperMovement-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetMeasuredGripperMovement-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetMeasuredGripperMovement-request> is deprecated: use kortex_driver-srv:GetMeasuredGripperMovement-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <GetMeasuredGripperMovement-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetMeasuredGripperMovement-request>) ostream)
  "Serializes a message object of type '<GetMeasuredGripperMovement-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetMeasuredGripperMovement-request>) istream)
  "Deserializes a message object of type '<GetMeasuredGripperMovement-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetMeasuredGripperMovement-request>)))
  "Returns string type for a service object of type '<GetMeasuredGripperMovement-request>"
  "kortex_driver/GetMeasuredGripperMovementRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetMeasuredGripperMovement-request)))
  "Returns string type for a service object of type 'GetMeasuredGripperMovement-request"
  "kortex_driver/GetMeasuredGripperMovementRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetMeasuredGripperMovement-request>)))
  "Returns md5sum for a message object of type '<GetMeasuredGripperMovement-request>"
  "8a811dcc01240e06c24ef1514e8bc606")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetMeasuredGripperMovement-request)))
  "Returns md5sum for a message object of type 'GetMeasuredGripperMovement-request"
  "8a811dcc01240e06c24ef1514e8bc606")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetMeasuredGripperMovement-request>)))
  "Returns full string definition for message of type '<GetMeasuredGripperMovement-request>"
  (cl:format cl:nil "GripperRequest input~%~%================================================================================~%MSG: kortex_driver/GripperRequest~%~%uint32 mode~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetMeasuredGripperMovement-request)))
  "Returns full string definition for message of type 'GetMeasuredGripperMovement-request"
  (cl:format cl:nil "GripperRequest input~%~%================================================================================~%MSG: kortex_driver/GripperRequest~%~%uint32 mode~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetMeasuredGripperMovement-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetMeasuredGripperMovement-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetMeasuredGripperMovement-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude GetMeasuredGripperMovement-response.msg.html

(cl:defclass <GetMeasuredGripperMovement-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Gripper
    :initform (cl:make-instance 'kortex_driver-msg:Gripper)))
)

(cl:defclass GetMeasuredGripperMovement-response (<GetMeasuredGripperMovement-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetMeasuredGripperMovement-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetMeasuredGripperMovement-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetMeasuredGripperMovement-response> is deprecated: use kortex_driver-srv:GetMeasuredGripperMovement-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <GetMeasuredGripperMovement-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetMeasuredGripperMovement-response>) ostream)
  "Serializes a message object of type '<GetMeasuredGripperMovement-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetMeasuredGripperMovement-response>) istream)
  "Deserializes a message object of type '<GetMeasuredGripperMovement-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetMeasuredGripperMovement-response>)))
  "Returns string type for a service object of type '<GetMeasuredGripperMovement-response>"
  "kortex_driver/GetMeasuredGripperMovementResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetMeasuredGripperMovement-response)))
  "Returns string type for a service object of type 'GetMeasuredGripperMovement-response"
  "kortex_driver/GetMeasuredGripperMovementResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetMeasuredGripperMovement-response>)))
  "Returns md5sum for a message object of type '<GetMeasuredGripperMovement-response>"
  "8a811dcc01240e06c24ef1514e8bc606")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetMeasuredGripperMovement-response)))
  "Returns md5sum for a message object of type 'GetMeasuredGripperMovement-response"
  "8a811dcc01240e06c24ef1514e8bc606")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetMeasuredGripperMovement-response>)))
  "Returns full string definition for message of type '<GetMeasuredGripperMovement-response>"
  (cl:format cl:nil "Gripper output~%~%================================================================================~%MSG: kortex_driver/Gripper~%~%Finger[] finger~%================================================================================~%MSG: kortex_driver/Finger~%~%uint32 finger_identifier~%float32 value~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetMeasuredGripperMovement-response)))
  "Returns full string definition for message of type 'GetMeasuredGripperMovement-response"
  (cl:format cl:nil "Gripper output~%~%================================================================================~%MSG: kortex_driver/Gripper~%~%Finger[] finger~%================================================================================~%MSG: kortex_driver/Finger~%~%uint32 finger_identifier~%float32 value~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetMeasuredGripperMovement-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetMeasuredGripperMovement-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetMeasuredGripperMovement-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetMeasuredGripperMovement)))
  'GetMeasuredGripperMovement-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetMeasuredGripperMovement)))
  'GetMeasuredGripperMovement-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetMeasuredGripperMovement)))
  "Returns string type for a service object of type '<GetMeasuredGripperMovement>"
  "kortex_driver/GetMeasuredGripperMovement")
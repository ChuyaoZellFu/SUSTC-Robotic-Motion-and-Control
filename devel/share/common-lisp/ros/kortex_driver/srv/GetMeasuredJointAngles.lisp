; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude GetMeasuredJointAngles-request.msg.html

(cl:defclass <GetMeasuredJointAngles-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass GetMeasuredJointAngles-request (<GetMeasuredJointAngles-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetMeasuredJointAngles-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetMeasuredJointAngles-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetMeasuredJointAngles-request> is deprecated: use kortex_driver-srv:GetMeasuredJointAngles-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <GetMeasuredJointAngles-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetMeasuredJointAngles-request>) ostream)
  "Serializes a message object of type '<GetMeasuredJointAngles-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetMeasuredJointAngles-request>) istream)
  "Deserializes a message object of type '<GetMeasuredJointAngles-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetMeasuredJointAngles-request>)))
  "Returns string type for a service object of type '<GetMeasuredJointAngles-request>"
  "kortex_driver/GetMeasuredJointAnglesRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetMeasuredJointAngles-request)))
  "Returns string type for a service object of type 'GetMeasuredJointAngles-request"
  "kortex_driver/GetMeasuredJointAnglesRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetMeasuredJointAngles-request>)))
  "Returns md5sum for a message object of type '<GetMeasuredJointAngles-request>"
  "a8cbc5dedf438645c485057a0e9b03e4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetMeasuredJointAngles-request)))
  "Returns md5sum for a message object of type 'GetMeasuredJointAngles-request"
  "a8cbc5dedf438645c485057a0e9b03e4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetMeasuredJointAngles-request>)))
  "Returns full string definition for message of type '<GetMeasuredJointAngles-request>"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetMeasuredJointAngles-request)))
  "Returns full string definition for message of type 'GetMeasuredJointAngles-request"
  (cl:format cl:nil "Empty input~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetMeasuredJointAngles-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetMeasuredJointAngles-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetMeasuredJointAngles-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude GetMeasuredJointAngles-response.msg.html

(cl:defclass <GetMeasuredJointAngles-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:JointAngles
    :initform (cl:make-instance 'kortex_driver-msg:JointAngles)))
)

(cl:defclass GetMeasuredJointAngles-response (<GetMeasuredJointAngles-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetMeasuredJointAngles-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetMeasuredJointAngles-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<GetMeasuredJointAngles-response> is deprecated: use kortex_driver-srv:GetMeasuredJointAngles-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <GetMeasuredJointAngles-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetMeasuredJointAngles-response>) ostream)
  "Serializes a message object of type '<GetMeasuredJointAngles-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetMeasuredJointAngles-response>) istream)
  "Deserializes a message object of type '<GetMeasuredJointAngles-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetMeasuredJointAngles-response>)))
  "Returns string type for a service object of type '<GetMeasuredJointAngles-response>"
  "kortex_driver/GetMeasuredJointAnglesResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetMeasuredJointAngles-response)))
  "Returns string type for a service object of type 'GetMeasuredJointAngles-response"
  "kortex_driver/GetMeasuredJointAnglesResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetMeasuredJointAngles-response>)))
  "Returns md5sum for a message object of type '<GetMeasuredJointAngles-response>"
  "a8cbc5dedf438645c485057a0e9b03e4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetMeasuredJointAngles-response)))
  "Returns md5sum for a message object of type 'GetMeasuredJointAngles-response"
  "a8cbc5dedf438645c485057a0e9b03e4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetMeasuredJointAngles-response>)))
  "Returns full string definition for message of type '<GetMeasuredJointAngles-response>"
  (cl:format cl:nil "JointAngles output~%~%================================================================================~%MSG: kortex_driver/JointAngles~%~%JointAngle[] joint_angles~%================================================================================~%MSG: kortex_driver/JointAngle~%~%uint32 joint_identifier~%float32 value~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetMeasuredJointAngles-response)))
  "Returns full string definition for message of type 'GetMeasuredJointAngles-response"
  (cl:format cl:nil "JointAngles output~%~%================================================================================~%MSG: kortex_driver/JointAngles~%~%JointAngle[] joint_angles~%================================================================================~%MSG: kortex_driver/JointAngle~%~%uint32 joint_identifier~%float32 value~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetMeasuredJointAngles-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetMeasuredJointAngles-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetMeasuredJointAngles-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetMeasuredJointAngles)))
  'GetMeasuredJointAngles-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetMeasuredJointAngles)))
  'GetMeasuredJointAngles-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetMeasuredJointAngles)))
  "Returns string type for a service object of type '<GetMeasuredJointAngles>"
  "kortex_driver/GetMeasuredJointAngles")
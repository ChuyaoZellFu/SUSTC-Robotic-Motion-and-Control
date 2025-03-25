; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude PlayJointTrajectory-request.msg.html

(cl:defclass <PlayJointTrajectory-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:ConstrainedJointAngles
    :initform (cl:make-instance 'kortex_driver-msg:ConstrainedJointAngles)))
)

(cl:defclass PlayJointTrajectory-request (<PlayJointTrajectory-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PlayJointTrajectory-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PlayJointTrajectory-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<PlayJointTrajectory-request> is deprecated: use kortex_driver-srv:PlayJointTrajectory-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <PlayJointTrajectory-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PlayJointTrajectory-request>) ostream)
  "Serializes a message object of type '<PlayJointTrajectory-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PlayJointTrajectory-request>) istream)
  "Deserializes a message object of type '<PlayJointTrajectory-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PlayJointTrajectory-request>)))
  "Returns string type for a service object of type '<PlayJointTrajectory-request>"
  "kortex_driver/PlayJointTrajectoryRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PlayJointTrajectory-request)))
  "Returns string type for a service object of type 'PlayJointTrajectory-request"
  "kortex_driver/PlayJointTrajectoryRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PlayJointTrajectory-request>)))
  "Returns md5sum for a message object of type '<PlayJointTrajectory-request>"
  "f04458839c6d778db05eb68c1fd72ac4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PlayJointTrajectory-request)))
  "Returns md5sum for a message object of type 'PlayJointTrajectory-request"
  "f04458839c6d778db05eb68c1fd72ac4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PlayJointTrajectory-request>)))
  "Returns full string definition for message of type '<PlayJointTrajectory-request>"
  (cl:format cl:nil "ConstrainedJointAngles input~%~%================================================================================~%MSG: kortex_driver/ConstrainedJointAngles~%~%JointAngles joint_angles~%JointTrajectoryConstraint constraint~%================================================================================~%MSG: kortex_driver/JointAngles~%~%JointAngle[] joint_angles~%================================================================================~%MSG: kortex_driver/JointAngle~%~%uint32 joint_identifier~%float32 value~%================================================================================~%MSG: kortex_driver/JointTrajectoryConstraint~%~%uint32 type~%float32 value~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PlayJointTrajectory-request)))
  "Returns full string definition for message of type 'PlayJointTrajectory-request"
  (cl:format cl:nil "ConstrainedJointAngles input~%~%================================================================================~%MSG: kortex_driver/ConstrainedJointAngles~%~%JointAngles joint_angles~%JointTrajectoryConstraint constraint~%================================================================================~%MSG: kortex_driver/JointAngles~%~%JointAngle[] joint_angles~%================================================================================~%MSG: kortex_driver/JointAngle~%~%uint32 joint_identifier~%float32 value~%================================================================================~%MSG: kortex_driver/JointTrajectoryConstraint~%~%uint32 type~%float32 value~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PlayJointTrajectory-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PlayJointTrajectory-request>))
  "Converts a ROS message object to a list"
  (cl:list 'PlayJointTrajectory-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude PlayJointTrajectory-response.msg.html

(cl:defclass <PlayJointTrajectory-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass PlayJointTrajectory-response (<PlayJointTrajectory-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PlayJointTrajectory-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PlayJointTrajectory-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<PlayJointTrajectory-response> is deprecated: use kortex_driver-srv:PlayJointTrajectory-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <PlayJointTrajectory-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PlayJointTrajectory-response>) ostream)
  "Serializes a message object of type '<PlayJointTrajectory-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PlayJointTrajectory-response>) istream)
  "Deserializes a message object of type '<PlayJointTrajectory-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PlayJointTrajectory-response>)))
  "Returns string type for a service object of type '<PlayJointTrajectory-response>"
  "kortex_driver/PlayJointTrajectoryResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PlayJointTrajectory-response)))
  "Returns string type for a service object of type 'PlayJointTrajectory-response"
  "kortex_driver/PlayJointTrajectoryResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PlayJointTrajectory-response>)))
  "Returns md5sum for a message object of type '<PlayJointTrajectory-response>"
  "f04458839c6d778db05eb68c1fd72ac4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PlayJointTrajectory-response)))
  "Returns md5sum for a message object of type 'PlayJointTrajectory-response"
  "f04458839c6d778db05eb68c1fd72ac4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PlayJointTrajectory-response>)))
  "Returns full string definition for message of type '<PlayJointTrajectory-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PlayJointTrajectory-response)))
  "Returns full string definition for message of type 'PlayJointTrajectory-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PlayJointTrajectory-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PlayJointTrajectory-response>))
  "Converts a ROS message object to a list"
  (cl:list 'PlayJointTrajectory-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'PlayJointTrajectory)))
  'PlayJointTrajectory-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'PlayJointTrajectory)))
  'PlayJointTrajectory-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PlayJointTrajectory)))
  "Returns string type for a service object of type '<PlayJointTrajectory>"
  "kortex_driver/PlayJointTrajectory")
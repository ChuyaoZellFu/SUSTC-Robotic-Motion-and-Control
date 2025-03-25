; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude PlaySelectedJointTrajectory-request.msg.html

(cl:defclass <PlaySelectedJointTrajectory-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:ConstrainedJointAngle
    :initform (cl:make-instance 'kortex_driver-msg:ConstrainedJointAngle)))
)

(cl:defclass PlaySelectedJointTrajectory-request (<PlaySelectedJointTrajectory-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PlaySelectedJointTrajectory-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PlaySelectedJointTrajectory-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<PlaySelectedJointTrajectory-request> is deprecated: use kortex_driver-srv:PlaySelectedJointTrajectory-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <PlaySelectedJointTrajectory-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PlaySelectedJointTrajectory-request>) ostream)
  "Serializes a message object of type '<PlaySelectedJointTrajectory-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PlaySelectedJointTrajectory-request>) istream)
  "Deserializes a message object of type '<PlaySelectedJointTrajectory-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PlaySelectedJointTrajectory-request>)))
  "Returns string type for a service object of type '<PlaySelectedJointTrajectory-request>"
  "kortex_driver/PlaySelectedJointTrajectoryRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PlaySelectedJointTrajectory-request)))
  "Returns string type for a service object of type 'PlaySelectedJointTrajectory-request"
  "kortex_driver/PlaySelectedJointTrajectoryRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PlaySelectedJointTrajectory-request>)))
  "Returns md5sum for a message object of type '<PlaySelectedJointTrajectory-request>"
  "83e0097177e42b394a0d659ed427f041")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PlaySelectedJointTrajectory-request)))
  "Returns md5sum for a message object of type 'PlaySelectedJointTrajectory-request"
  "83e0097177e42b394a0d659ed427f041")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PlaySelectedJointTrajectory-request>)))
  "Returns full string definition for message of type '<PlaySelectedJointTrajectory-request>"
  (cl:format cl:nil "ConstrainedJointAngle input~%~%================================================================================~%MSG: kortex_driver/ConstrainedJointAngle~%~%uint32 joint_identifier~%float32 value~%JointTrajectoryConstraint constraint~%================================================================================~%MSG: kortex_driver/JointTrajectoryConstraint~%~%uint32 type~%float32 value~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PlaySelectedJointTrajectory-request)))
  "Returns full string definition for message of type 'PlaySelectedJointTrajectory-request"
  (cl:format cl:nil "ConstrainedJointAngle input~%~%================================================================================~%MSG: kortex_driver/ConstrainedJointAngle~%~%uint32 joint_identifier~%float32 value~%JointTrajectoryConstraint constraint~%================================================================================~%MSG: kortex_driver/JointTrajectoryConstraint~%~%uint32 type~%float32 value~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PlaySelectedJointTrajectory-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PlaySelectedJointTrajectory-request>))
  "Converts a ROS message object to a list"
  (cl:list 'PlaySelectedJointTrajectory-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude PlaySelectedJointTrajectory-response.msg.html

(cl:defclass <PlaySelectedJointTrajectory-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Empty
    :initform (cl:make-instance 'kortex_driver-msg:Empty)))
)

(cl:defclass PlaySelectedJointTrajectory-response (<PlaySelectedJointTrajectory-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PlaySelectedJointTrajectory-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PlaySelectedJointTrajectory-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<PlaySelectedJointTrajectory-response> is deprecated: use kortex_driver-srv:PlaySelectedJointTrajectory-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <PlaySelectedJointTrajectory-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PlaySelectedJointTrajectory-response>) ostream)
  "Serializes a message object of type '<PlaySelectedJointTrajectory-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PlaySelectedJointTrajectory-response>) istream)
  "Deserializes a message object of type '<PlaySelectedJointTrajectory-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PlaySelectedJointTrajectory-response>)))
  "Returns string type for a service object of type '<PlaySelectedJointTrajectory-response>"
  "kortex_driver/PlaySelectedJointTrajectoryResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PlaySelectedJointTrajectory-response)))
  "Returns string type for a service object of type 'PlaySelectedJointTrajectory-response"
  "kortex_driver/PlaySelectedJointTrajectoryResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PlaySelectedJointTrajectory-response>)))
  "Returns md5sum for a message object of type '<PlaySelectedJointTrajectory-response>"
  "83e0097177e42b394a0d659ed427f041")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PlaySelectedJointTrajectory-response)))
  "Returns md5sum for a message object of type 'PlaySelectedJointTrajectory-response"
  "83e0097177e42b394a0d659ed427f041")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PlaySelectedJointTrajectory-response>)))
  "Returns full string definition for message of type '<PlaySelectedJointTrajectory-response>"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PlaySelectedJointTrajectory-response)))
  "Returns full string definition for message of type 'PlaySelectedJointTrajectory-response"
  (cl:format cl:nil "Empty output~%~%================================================================================~%MSG: kortex_driver/Empty~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PlaySelectedJointTrajectory-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PlaySelectedJointTrajectory-response>))
  "Converts a ROS message object to a list"
  (cl:list 'PlaySelectedJointTrajectory-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'PlaySelectedJointTrajectory)))
  'PlaySelectedJointTrajectory-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'PlaySelectedJointTrajectory)))
  'PlaySelectedJointTrajectory-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PlaySelectedJointTrajectory)))
  "Returns string type for a service object of type '<PlaySelectedJointTrajectory>"
  "kortex_driver/PlaySelectedJointTrajectory")
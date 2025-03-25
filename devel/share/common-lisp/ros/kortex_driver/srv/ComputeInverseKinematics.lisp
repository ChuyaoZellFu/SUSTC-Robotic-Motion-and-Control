; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude ComputeInverseKinematics-request.msg.html

(cl:defclass <ComputeInverseKinematics-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:IKData
    :initform (cl:make-instance 'kortex_driver-msg:IKData)))
)

(cl:defclass ComputeInverseKinematics-request (<ComputeInverseKinematics-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ComputeInverseKinematics-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ComputeInverseKinematics-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<ComputeInverseKinematics-request> is deprecated: use kortex_driver-srv:ComputeInverseKinematics-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <ComputeInverseKinematics-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ComputeInverseKinematics-request>) ostream)
  "Serializes a message object of type '<ComputeInverseKinematics-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ComputeInverseKinematics-request>) istream)
  "Deserializes a message object of type '<ComputeInverseKinematics-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ComputeInverseKinematics-request>)))
  "Returns string type for a service object of type '<ComputeInverseKinematics-request>"
  "kortex_driver/ComputeInverseKinematicsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ComputeInverseKinematics-request)))
  "Returns string type for a service object of type 'ComputeInverseKinematics-request"
  "kortex_driver/ComputeInverseKinematicsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ComputeInverseKinematics-request>)))
  "Returns md5sum for a message object of type '<ComputeInverseKinematics-request>"
  "290825a1e5de199dc18075d261a5fee3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ComputeInverseKinematics-request)))
  "Returns md5sum for a message object of type 'ComputeInverseKinematics-request"
  "290825a1e5de199dc18075d261a5fee3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ComputeInverseKinematics-request>)))
  "Returns full string definition for message of type '<ComputeInverseKinematics-request>"
  (cl:format cl:nil "IKData input~%~%================================================================================~%MSG: kortex_driver/IKData~%~%Pose cartesian_pose~%JointAngles guess~%================================================================================~%MSG: kortex_driver/Pose~%~%float32 x~%float32 y~%float32 z~%float32 theta_x~%float32 theta_y~%float32 theta_z~%================================================================================~%MSG: kortex_driver/JointAngles~%~%JointAngle[] joint_angles~%================================================================================~%MSG: kortex_driver/JointAngle~%~%uint32 joint_identifier~%float32 value~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ComputeInverseKinematics-request)))
  "Returns full string definition for message of type 'ComputeInverseKinematics-request"
  (cl:format cl:nil "IKData input~%~%================================================================================~%MSG: kortex_driver/IKData~%~%Pose cartesian_pose~%JointAngles guess~%================================================================================~%MSG: kortex_driver/Pose~%~%float32 x~%float32 y~%float32 z~%float32 theta_x~%float32 theta_y~%float32 theta_z~%================================================================================~%MSG: kortex_driver/JointAngles~%~%JointAngle[] joint_angles~%================================================================================~%MSG: kortex_driver/JointAngle~%~%uint32 joint_identifier~%float32 value~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ComputeInverseKinematics-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ComputeInverseKinematics-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ComputeInverseKinematics-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude ComputeInverseKinematics-response.msg.html

(cl:defclass <ComputeInverseKinematics-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:JointAngles
    :initform (cl:make-instance 'kortex_driver-msg:JointAngles)))
)

(cl:defclass ComputeInverseKinematics-response (<ComputeInverseKinematics-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ComputeInverseKinematics-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ComputeInverseKinematics-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<ComputeInverseKinematics-response> is deprecated: use kortex_driver-srv:ComputeInverseKinematics-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <ComputeInverseKinematics-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ComputeInverseKinematics-response>) ostream)
  "Serializes a message object of type '<ComputeInverseKinematics-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ComputeInverseKinematics-response>) istream)
  "Deserializes a message object of type '<ComputeInverseKinematics-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ComputeInverseKinematics-response>)))
  "Returns string type for a service object of type '<ComputeInverseKinematics-response>"
  "kortex_driver/ComputeInverseKinematicsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ComputeInverseKinematics-response)))
  "Returns string type for a service object of type 'ComputeInverseKinematics-response"
  "kortex_driver/ComputeInverseKinematicsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ComputeInverseKinematics-response>)))
  "Returns md5sum for a message object of type '<ComputeInverseKinematics-response>"
  "290825a1e5de199dc18075d261a5fee3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ComputeInverseKinematics-response)))
  "Returns md5sum for a message object of type 'ComputeInverseKinematics-response"
  "290825a1e5de199dc18075d261a5fee3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ComputeInverseKinematics-response>)))
  "Returns full string definition for message of type '<ComputeInverseKinematics-response>"
  (cl:format cl:nil "JointAngles output~%~%================================================================================~%MSG: kortex_driver/JointAngles~%~%JointAngle[] joint_angles~%================================================================================~%MSG: kortex_driver/JointAngle~%~%uint32 joint_identifier~%float32 value~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ComputeInverseKinematics-response)))
  "Returns full string definition for message of type 'ComputeInverseKinematics-response"
  (cl:format cl:nil "JointAngles output~%~%================================================================================~%MSG: kortex_driver/JointAngles~%~%JointAngle[] joint_angles~%================================================================================~%MSG: kortex_driver/JointAngle~%~%uint32 joint_identifier~%float32 value~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ComputeInverseKinematics-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ComputeInverseKinematics-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ComputeInverseKinematics-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ComputeInverseKinematics)))
  'ComputeInverseKinematics-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ComputeInverseKinematics)))
  'ComputeInverseKinematics-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ComputeInverseKinematics)))
  "Returns string type for a service object of type '<ComputeInverseKinematics>"
  "kortex_driver/ComputeInverseKinematics")
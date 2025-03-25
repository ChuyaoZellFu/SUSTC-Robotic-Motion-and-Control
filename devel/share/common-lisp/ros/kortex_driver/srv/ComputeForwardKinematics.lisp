; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude ComputeForwardKinematics-request.msg.html

(cl:defclass <ComputeForwardKinematics-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type kortex_driver-msg:JointAngles
    :initform (cl:make-instance 'kortex_driver-msg:JointAngles)))
)

(cl:defclass ComputeForwardKinematics-request (<ComputeForwardKinematics-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ComputeForwardKinematics-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ComputeForwardKinematics-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<ComputeForwardKinematics-request> is deprecated: use kortex_driver-srv:ComputeForwardKinematics-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <ComputeForwardKinematics-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:input-val is deprecated.  Use kortex_driver-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ComputeForwardKinematics-request>) ostream)
  "Serializes a message object of type '<ComputeForwardKinematics-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'input) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ComputeForwardKinematics-request>) istream)
  "Deserializes a message object of type '<ComputeForwardKinematics-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'input) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ComputeForwardKinematics-request>)))
  "Returns string type for a service object of type '<ComputeForwardKinematics-request>"
  "kortex_driver/ComputeForwardKinematicsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ComputeForwardKinematics-request)))
  "Returns string type for a service object of type 'ComputeForwardKinematics-request"
  "kortex_driver/ComputeForwardKinematicsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ComputeForwardKinematics-request>)))
  "Returns md5sum for a message object of type '<ComputeForwardKinematics-request>"
  "2406133d7d6bcd723f8f11457d1f2636")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ComputeForwardKinematics-request)))
  "Returns md5sum for a message object of type 'ComputeForwardKinematics-request"
  "2406133d7d6bcd723f8f11457d1f2636")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ComputeForwardKinematics-request>)))
  "Returns full string definition for message of type '<ComputeForwardKinematics-request>"
  (cl:format cl:nil "JointAngles input~%~%================================================================================~%MSG: kortex_driver/JointAngles~%~%JointAngle[] joint_angles~%================================================================================~%MSG: kortex_driver/JointAngle~%~%uint32 joint_identifier~%float32 value~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ComputeForwardKinematics-request)))
  "Returns full string definition for message of type 'ComputeForwardKinematics-request"
  (cl:format cl:nil "JointAngles input~%~%================================================================================~%MSG: kortex_driver/JointAngles~%~%JointAngle[] joint_angles~%================================================================================~%MSG: kortex_driver/JointAngle~%~%uint32 joint_identifier~%float32 value~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ComputeForwardKinematics-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ComputeForwardKinematics-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ComputeForwardKinematics-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude ComputeForwardKinematics-response.msg.html

(cl:defclass <ComputeForwardKinematics-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type kortex_driver-msg:Pose
    :initform (cl:make-instance 'kortex_driver-msg:Pose)))
)

(cl:defclass ComputeForwardKinematics-response (<ComputeForwardKinematics-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ComputeForwardKinematics-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ComputeForwardKinematics-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<ComputeForwardKinematics-response> is deprecated: use kortex_driver-srv:ComputeForwardKinematics-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <ComputeForwardKinematics-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:output-val is deprecated.  Use kortex_driver-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ComputeForwardKinematics-response>) ostream)
  "Serializes a message object of type '<ComputeForwardKinematics-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ComputeForwardKinematics-response>) istream)
  "Deserializes a message object of type '<ComputeForwardKinematics-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ComputeForwardKinematics-response>)))
  "Returns string type for a service object of type '<ComputeForwardKinematics-response>"
  "kortex_driver/ComputeForwardKinematicsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ComputeForwardKinematics-response)))
  "Returns string type for a service object of type 'ComputeForwardKinematics-response"
  "kortex_driver/ComputeForwardKinematicsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ComputeForwardKinematics-response>)))
  "Returns md5sum for a message object of type '<ComputeForwardKinematics-response>"
  "2406133d7d6bcd723f8f11457d1f2636")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ComputeForwardKinematics-response)))
  "Returns md5sum for a message object of type 'ComputeForwardKinematics-response"
  "2406133d7d6bcd723f8f11457d1f2636")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ComputeForwardKinematics-response>)))
  "Returns full string definition for message of type '<ComputeForwardKinematics-response>"
  (cl:format cl:nil "Pose output~%~%================================================================================~%MSG: kortex_driver/Pose~%~%float32 x~%float32 y~%float32 z~%float32 theta_x~%float32 theta_y~%float32 theta_z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ComputeForwardKinematics-response)))
  "Returns full string definition for message of type 'ComputeForwardKinematics-response"
  (cl:format cl:nil "Pose output~%~%================================================================================~%MSG: kortex_driver/Pose~%~%float32 x~%float32 y~%float32 z~%float32 theta_x~%float32 theta_y~%float32 theta_z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ComputeForwardKinematics-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ComputeForwardKinematics-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ComputeForwardKinematics-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ComputeForwardKinematics)))
  'ComputeForwardKinematics-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ComputeForwardKinematics)))
  'ComputeForwardKinematics-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ComputeForwardKinematics)))
  "Returns string type for a service object of type '<ComputeForwardKinematics>"
  "kortex_driver/ComputeForwardKinematics")
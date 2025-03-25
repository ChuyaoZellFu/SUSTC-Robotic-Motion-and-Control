; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude IKData.msg.html

(cl:defclass <IKData> (roslisp-msg-protocol:ros-message)
  ((cartesian_pose
    :reader cartesian_pose
    :initarg :cartesian_pose
    :type kortex_driver-msg:Pose
    :initform (cl:make-instance 'kortex_driver-msg:Pose))
   (guess
    :reader guess
    :initarg :guess
    :type kortex_driver-msg:JointAngles
    :initform (cl:make-instance 'kortex_driver-msg:JointAngles)))
)

(cl:defclass IKData (<IKData>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <IKData>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'IKData)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<IKData> is deprecated: use kortex_driver-msg:IKData instead.")))

(cl:ensure-generic-function 'cartesian_pose-val :lambda-list '(m))
(cl:defmethod cartesian_pose-val ((m <IKData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:cartesian_pose-val is deprecated.  Use kortex_driver-msg:cartesian_pose instead.")
  (cartesian_pose m))

(cl:ensure-generic-function 'guess-val :lambda-list '(m))
(cl:defmethod guess-val ((m <IKData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:guess-val is deprecated.  Use kortex_driver-msg:guess instead.")
  (guess m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <IKData>) ostream)
  "Serializes a message object of type '<IKData>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'cartesian_pose) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'guess) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <IKData>) istream)
  "Deserializes a message object of type '<IKData>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'cartesian_pose) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'guess) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<IKData>)))
  "Returns string type for a message object of type '<IKData>"
  "kortex_driver/IKData")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'IKData)))
  "Returns string type for a message object of type 'IKData"
  "kortex_driver/IKData")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<IKData>)))
  "Returns md5sum for a message object of type '<IKData>"
  "29f05c9210572828af7df145fee29d3b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'IKData)))
  "Returns md5sum for a message object of type 'IKData"
  "29f05c9210572828af7df145fee29d3b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<IKData>)))
  "Returns full string definition for message of type '<IKData>"
  (cl:format cl:nil "~%Pose cartesian_pose~%JointAngles guess~%================================================================================~%MSG: kortex_driver/Pose~%~%float32 x~%float32 y~%float32 z~%float32 theta_x~%float32 theta_y~%float32 theta_z~%================================================================================~%MSG: kortex_driver/JointAngles~%~%JointAngle[] joint_angles~%================================================================================~%MSG: kortex_driver/JointAngle~%~%uint32 joint_identifier~%float32 value~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'IKData)))
  "Returns full string definition for message of type 'IKData"
  (cl:format cl:nil "~%Pose cartesian_pose~%JointAngles guess~%================================================================================~%MSG: kortex_driver/Pose~%~%float32 x~%float32 y~%float32 z~%float32 theta_x~%float32 theta_y~%float32 theta_z~%================================================================================~%MSG: kortex_driver/JointAngles~%~%JointAngle[] joint_angles~%================================================================================~%MSG: kortex_driver/JointAngle~%~%uint32 joint_identifier~%float32 value~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <IKData>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'cartesian_pose))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'guess))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <IKData>))
  "Converts a ROS message object to a list"
  (cl:list 'IKData
    (cl:cons ':cartesian_pose (cartesian_pose msg))
    (cl:cons ':guess (guess msg))
))

; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude ChangeJointSpeeds.msg.html

(cl:defclass <ChangeJointSpeeds> (roslisp-msg-protocol:ros-message)
  ((joint_speeds
    :reader joint_speeds
    :initarg :joint_speeds
    :type kortex_driver-msg:Base_JointSpeeds
    :initform (cl:make-instance 'kortex_driver-msg:Base_JointSpeeds)))
)

(cl:defclass ChangeJointSpeeds (<ChangeJointSpeeds>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ChangeJointSpeeds>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ChangeJointSpeeds)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<ChangeJointSpeeds> is deprecated: use kortex_driver-msg:ChangeJointSpeeds instead.")))

(cl:ensure-generic-function 'joint_speeds-val :lambda-list '(m))
(cl:defmethod joint_speeds-val ((m <ChangeJointSpeeds>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:joint_speeds-val is deprecated.  Use kortex_driver-msg:joint_speeds instead.")
  (joint_speeds m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ChangeJointSpeeds>) ostream)
  "Serializes a message object of type '<ChangeJointSpeeds>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'joint_speeds) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ChangeJointSpeeds>) istream)
  "Deserializes a message object of type '<ChangeJointSpeeds>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'joint_speeds) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ChangeJointSpeeds>)))
  "Returns string type for a message object of type '<ChangeJointSpeeds>"
  "kortex_driver/ChangeJointSpeeds")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ChangeJointSpeeds)))
  "Returns string type for a message object of type 'ChangeJointSpeeds"
  "kortex_driver/ChangeJointSpeeds")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ChangeJointSpeeds>)))
  "Returns md5sum for a message object of type '<ChangeJointSpeeds>"
  "d6e510b2965f87e14832d5332810f679")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ChangeJointSpeeds)))
  "Returns md5sum for a message object of type 'ChangeJointSpeeds"
  "d6e510b2965f87e14832d5332810f679")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ChangeJointSpeeds>)))
  "Returns full string definition for message of type '<ChangeJointSpeeds>"
  (cl:format cl:nil "~%Base_JointSpeeds joint_speeds~%================================================================================~%MSG: kortex_driver/Base_JointSpeeds~%~%JointSpeed[] joint_speeds~%uint32 duration~%================================================================================~%MSG: kortex_driver/JointSpeed~%~%uint32 joint_identifier~%float32 value~%uint32 duration~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ChangeJointSpeeds)))
  "Returns full string definition for message of type 'ChangeJointSpeeds"
  (cl:format cl:nil "~%Base_JointSpeeds joint_speeds~%================================================================================~%MSG: kortex_driver/Base_JointSpeeds~%~%JointSpeed[] joint_speeds~%uint32 duration~%================================================================================~%MSG: kortex_driver/JointSpeed~%~%uint32 joint_identifier~%float32 value~%uint32 duration~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ChangeJointSpeeds>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'joint_speeds))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ChangeJointSpeeds>))
  "Converts a ROS message object to a list"
  (cl:list 'ChangeJointSpeeds
    (cl:cons ':joint_speeds (joint_speeds msg))
))

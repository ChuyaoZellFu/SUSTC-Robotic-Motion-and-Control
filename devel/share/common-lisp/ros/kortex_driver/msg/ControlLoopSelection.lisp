; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude ControlLoopSelection.msg.html

(cl:defclass <ControlLoopSelection> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass ControlLoopSelection (<ControlLoopSelection>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ControlLoopSelection>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ControlLoopSelection)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<ControlLoopSelection> is deprecated: use kortex_driver-msg:ControlLoopSelection instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<ControlLoopSelection>)))
    "Constants for message type '<ControlLoopSelection>"
  '((:RESERVED . 0)
    (:JOINT_POSITION . 1)
    (:MOTOR_POSITION . 2)
    (:JOINT_VELOCITY . 4)
    (:MOTOR_VELOCITY . 8)
    (:JOINT_TORQUE . 16)
    (:MOTOR_CURRENT . 32)
    (:JOINT_TORQUE_HIGH_VELOCITY . 64))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'ControlLoopSelection)))
    "Constants for message type 'ControlLoopSelection"
  '((:RESERVED . 0)
    (:JOINT_POSITION . 1)
    (:MOTOR_POSITION . 2)
    (:JOINT_VELOCITY . 4)
    (:MOTOR_VELOCITY . 8)
    (:JOINT_TORQUE . 16)
    (:MOTOR_CURRENT . 32)
    (:JOINT_TORQUE_HIGH_VELOCITY . 64))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ControlLoopSelection>) ostream)
  "Serializes a message object of type '<ControlLoopSelection>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ControlLoopSelection>) istream)
  "Deserializes a message object of type '<ControlLoopSelection>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ControlLoopSelection>)))
  "Returns string type for a message object of type '<ControlLoopSelection>"
  "kortex_driver/ControlLoopSelection")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ControlLoopSelection)))
  "Returns string type for a message object of type 'ControlLoopSelection"
  "kortex_driver/ControlLoopSelection")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ControlLoopSelection>)))
  "Returns md5sum for a message object of type '<ControlLoopSelection>"
  "85d717df01106b4f27c7c235a5980355")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ControlLoopSelection)))
  "Returns md5sum for a message object of type 'ControlLoopSelection"
  "85d717df01106b4f27c7c235a5980355")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ControlLoopSelection>)))
  "Returns full string definition for message of type '<ControlLoopSelection>"
  (cl:format cl:nil "~%uint32 RESERVED = 0~%~%uint32 JOINT_POSITION = 1~%~%uint32 MOTOR_POSITION = 2~%~%uint32 JOINT_VELOCITY = 4~%~%uint32 MOTOR_VELOCITY = 8~%~%uint32 JOINT_TORQUE = 16~%~%uint32 MOTOR_CURRENT = 32~%~%uint32 JOINT_TORQUE_HIGH_VELOCITY = 64~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ControlLoopSelection)))
  "Returns full string definition for message of type 'ControlLoopSelection"
  (cl:format cl:nil "~%uint32 RESERVED = 0~%~%uint32 JOINT_POSITION = 1~%~%uint32 MOTOR_POSITION = 2~%~%uint32 JOINT_VELOCITY = 4~%~%uint32 MOTOR_VELOCITY = 8~%~%uint32 JOINT_TORQUE = 16~%~%uint32 MOTOR_CURRENT = 32~%~%uint32 JOINT_TORQUE_HIGH_VELOCITY = 64~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ControlLoopSelection>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ControlLoopSelection>))
  "Converts a ROS message object to a list"
  (cl:list 'ControlLoopSelection
))

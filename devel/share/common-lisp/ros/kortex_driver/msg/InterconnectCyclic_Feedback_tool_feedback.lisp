; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude InterconnectCyclic_Feedback_tool_feedback.msg.html

(cl:defclass <InterconnectCyclic_Feedback_tool_feedback> (roslisp-msg-protocol:ros-message)
  ((gripper_feedback
    :reader gripper_feedback
    :initarg :gripper_feedback
    :type (cl:vector kortex_driver-msg:GripperCyclic_Feedback)
   :initform (cl:make-array 0 :element-type 'kortex_driver-msg:GripperCyclic_Feedback :initial-element (cl:make-instance 'kortex_driver-msg:GripperCyclic_Feedback))))
)

(cl:defclass InterconnectCyclic_Feedback_tool_feedback (<InterconnectCyclic_Feedback_tool_feedback>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <InterconnectCyclic_Feedback_tool_feedback>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'InterconnectCyclic_Feedback_tool_feedback)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<InterconnectCyclic_Feedback_tool_feedback> is deprecated: use kortex_driver-msg:InterconnectCyclic_Feedback_tool_feedback instead.")))

(cl:ensure-generic-function 'gripper_feedback-val :lambda-list '(m))
(cl:defmethod gripper_feedback-val ((m <InterconnectCyclic_Feedback_tool_feedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:gripper_feedback-val is deprecated.  Use kortex_driver-msg:gripper_feedback instead.")
  (gripper_feedback m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <InterconnectCyclic_Feedback_tool_feedback>) ostream)
  "Serializes a message object of type '<InterconnectCyclic_Feedback_tool_feedback>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'gripper_feedback))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'gripper_feedback))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <InterconnectCyclic_Feedback_tool_feedback>) istream)
  "Deserializes a message object of type '<InterconnectCyclic_Feedback_tool_feedback>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'gripper_feedback) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'gripper_feedback)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'kortex_driver-msg:GripperCyclic_Feedback))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<InterconnectCyclic_Feedback_tool_feedback>)))
  "Returns string type for a message object of type '<InterconnectCyclic_Feedback_tool_feedback>"
  "kortex_driver/InterconnectCyclic_Feedback_tool_feedback")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'InterconnectCyclic_Feedback_tool_feedback)))
  "Returns string type for a message object of type 'InterconnectCyclic_Feedback_tool_feedback"
  "kortex_driver/InterconnectCyclic_Feedback_tool_feedback")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<InterconnectCyclic_Feedback_tool_feedback>)))
  "Returns md5sum for a message object of type '<InterconnectCyclic_Feedback_tool_feedback>"
  "39e2425dcc897a387ad5c442c6119c99")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'InterconnectCyclic_Feedback_tool_feedback)))
  "Returns md5sum for a message object of type 'InterconnectCyclic_Feedback_tool_feedback"
  "39e2425dcc897a387ad5c442c6119c99")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<InterconnectCyclic_Feedback_tool_feedback>)))
  "Returns full string definition for message of type '<InterconnectCyclic_Feedback_tool_feedback>"
  (cl:format cl:nil "~%GripperCyclic_Feedback[] gripper_feedback~%================================================================================~%MSG: kortex_driver/GripperCyclic_Feedback~%~%GripperCyclic_MessageId feedback_id~%uint32 status_flags~%uint32 fault_bank_a~%uint32 fault_bank_b~%uint32 warning_bank_a~%uint32 warning_bank_b~%MotorFeedback[] motor~%================================================================================~%MSG: kortex_driver/GripperCyclic_MessageId~%~%uint32 identifier~%================================================================================~%MSG: kortex_driver/MotorFeedback~%~%uint32 motor_id~%float32 position~%float32 velocity~%float32 current_motor~%float32 voltage~%float32 temperature_motor~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'InterconnectCyclic_Feedback_tool_feedback)))
  "Returns full string definition for message of type 'InterconnectCyclic_Feedback_tool_feedback"
  (cl:format cl:nil "~%GripperCyclic_Feedback[] gripper_feedback~%================================================================================~%MSG: kortex_driver/GripperCyclic_Feedback~%~%GripperCyclic_MessageId feedback_id~%uint32 status_flags~%uint32 fault_bank_a~%uint32 fault_bank_b~%uint32 warning_bank_a~%uint32 warning_bank_b~%MotorFeedback[] motor~%================================================================================~%MSG: kortex_driver/GripperCyclic_MessageId~%~%uint32 identifier~%================================================================================~%MSG: kortex_driver/MotorFeedback~%~%uint32 motor_id~%float32 position~%float32 velocity~%float32 current_motor~%float32 voltage~%float32 temperature_motor~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <InterconnectCyclic_Feedback_tool_feedback>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'gripper_feedback) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <InterconnectCyclic_Feedback_tool_feedback>))
  "Converts a ROS message object to a list"
  (cl:list 'InterconnectCyclic_Feedback_tool_feedback
    (cl:cons ':gripper_feedback (gripper_feedback msg))
))

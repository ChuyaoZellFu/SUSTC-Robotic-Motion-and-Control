; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude InterconnectCyclic_CustomData_tool_customData.msg.html

(cl:defclass <InterconnectCyclic_CustomData_tool_customData> (roslisp-msg-protocol:ros-message)
  ((gripper_custom_data
    :reader gripper_custom_data
    :initarg :gripper_custom_data
    :type (cl:vector kortex_driver-msg:GripperCyclic_CustomData)
   :initform (cl:make-array 0 :element-type 'kortex_driver-msg:GripperCyclic_CustomData :initial-element (cl:make-instance 'kortex_driver-msg:GripperCyclic_CustomData))))
)

(cl:defclass InterconnectCyclic_CustomData_tool_customData (<InterconnectCyclic_CustomData_tool_customData>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <InterconnectCyclic_CustomData_tool_customData>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'InterconnectCyclic_CustomData_tool_customData)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<InterconnectCyclic_CustomData_tool_customData> is deprecated: use kortex_driver-msg:InterconnectCyclic_CustomData_tool_customData instead.")))

(cl:ensure-generic-function 'gripper_custom_data-val :lambda-list '(m))
(cl:defmethod gripper_custom_data-val ((m <InterconnectCyclic_CustomData_tool_customData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:gripper_custom_data-val is deprecated.  Use kortex_driver-msg:gripper_custom_data instead.")
  (gripper_custom_data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <InterconnectCyclic_CustomData_tool_customData>) ostream)
  "Serializes a message object of type '<InterconnectCyclic_CustomData_tool_customData>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'gripper_custom_data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'gripper_custom_data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <InterconnectCyclic_CustomData_tool_customData>) istream)
  "Deserializes a message object of type '<InterconnectCyclic_CustomData_tool_customData>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'gripper_custom_data) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'gripper_custom_data)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'kortex_driver-msg:GripperCyclic_CustomData))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<InterconnectCyclic_CustomData_tool_customData>)))
  "Returns string type for a message object of type '<InterconnectCyclic_CustomData_tool_customData>"
  "kortex_driver/InterconnectCyclic_CustomData_tool_customData")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'InterconnectCyclic_CustomData_tool_customData)))
  "Returns string type for a message object of type 'InterconnectCyclic_CustomData_tool_customData"
  "kortex_driver/InterconnectCyclic_CustomData_tool_customData")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<InterconnectCyclic_CustomData_tool_customData>)))
  "Returns md5sum for a message object of type '<InterconnectCyclic_CustomData_tool_customData>"
  "08300f61864775cfea0ba45676ef4872")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'InterconnectCyclic_CustomData_tool_customData)))
  "Returns md5sum for a message object of type 'InterconnectCyclic_CustomData_tool_customData"
  "08300f61864775cfea0ba45676ef4872")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<InterconnectCyclic_CustomData_tool_customData>)))
  "Returns full string definition for message of type '<InterconnectCyclic_CustomData_tool_customData>"
  (cl:format cl:nil "~%GripperCyclic_CustomData[] gripper_custom_data~%================================================================================~%MSG: kortex_driver/GripperCyclic_CustomData~%~%GripperCyclic_MessageId custom_data_id~%CustomDataUnit gripper_custom_data~%CustomDataUnit[] motor_custom_data~%================================================================================~%MSG: kortex_driver/GripperCyclic_MessageId~%~%uint32 identifier~%================================================================================~%MSG: kortex_driver/CustomDataUnit~%~%uint32 custom_data_0~%uint32 custom_data_1~%uint32 custom_data_2~%uint32 custom_data_3~%uint32 custom_data_4~%uint32 custom_data_5~%uint32 custom_data_6~%uint32 custom_data_7~%uint32 custom_data_8~%uint32 custom_data_9~%uint32 custom_data_10~%uint32 custom_data_11~%uint32 custom_data_12~%uint32 custom_data_13~%uint32 custom_data_14~%uint32 custom_data_15~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'InterconnectCyclic_CustomData_tool_customData)))
  "Returns full string definition for message of type 'InterconnectCyclic_CustomData_tool_customData"
  (cl:format cl:nil "~%GripperCyclic_CustomData[] gripper_custom_data~%================================================================================~%MSG: kortex_driver/GripperCyclic_CustomData~%~%GripperCyclic_MessageId custom_data_id~%CustomDataUnit gripper_custom_data~%CustomDataUnit[] motor_custom_data~%================================================================================~%MSG: kortex_driver/GripperCyclic_MessageId~%~%uint32 identifier~%================================================================================~%MSG: kortex_driver/CustomDataUnit~%~%uint32 custom_data_0~%uint32 custom_data_1~%uint32 custom_data_2~%uint32 custom_data_3~%uint32 custom_data_4~%uint32 custom_data_5~%uint32 custom_data_6~%uint32 custom_data_7~%uint32 custom_data_8~%uint32 custom_data_9~%uint32 custom_data_10~%uint32 custom_data_11~%uint32 custom_data_12~%uint32 custom_data_13~%uint32 custom_data_14~%uint32 custom_data_15~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <InterconnectCyclic_CustomData_tool_customData>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'gripper_custom_data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <InterconnectCyclic_CustomData_tool_customData>))
  "Converts a ROS message object to a list"
  (cl:list 'InterconnectCyclic_CustomData_tool_customData
    (cl:cons ':gripper_custom_data (gripper_custom_data msg))
))

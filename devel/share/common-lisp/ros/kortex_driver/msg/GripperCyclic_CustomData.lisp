; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude GripperCyclic_CustomData.msg.html

(cl:defclass <GripperCyclic_CustomData> (roslisp-msg-protocol:ros-message)
  ((custom_data_id
    :reader custom_data_id
    :initarg :custom_data_id
    :type kortex_driver-msg:GripperCyclic_MessageId
    :initform (cl:make-instance 'kortex_driver-msg:GripperCyclic_MessageId))
   (gripper_custom_data
    :reader gripper_custom_data
    :initarg :gripper_custom_data
    :type kortex_driver-msg:CustomDataUnit
    :initform (cl:make-instance 'kortex_driver-msg:CustomDataUnit))
   (motor_custom_data
    :reader motor_custom_data
    :initarg :motor_custom_data
    :type (cl:vector kortex_driver-msg:CustomDataUnit)
   :initform (cl:make-array 0 :element-type 'kortex_driver-msg:CustomDataUnit :initial-element (cl:make-instance 'kortex_driver-msg:CustomDataUnit))))
)

(cl:defclass GripperCyclic_CustomData (<GripperCyclic_CustomData>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GripperCyclic_CustomData>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GripperCyclic_CustomData)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<GripperCyclic_CustomData> is deprecated: use kortex_driver-msg:GripperCyclic_CustomData instead.")))

(cl:ensure-generic-function 'custom_data_id-val :lambda-list '(m))
(cl:defmethod custom_data_id-val ((m <GripperCyclic_CustomData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:custom_data_id-val is deprecated.  Use kortex_driver-msg:custom_data_id instead.")
  (custom_data_id m))

(cl:ensure-generic-function 'gripper_custom_data-val :lambda-list '(m))
(cl:defmethod gripper_custom_data-val ((m <GripperCyclic_CustomData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:gripper_custom_data-val is deprecated.  Use kortex_driver-msg:gripper_custom_data instead.")
  (gripper_custom_data m))

(cl:ensure-generic-function 'motor_custom_data-val :lambda-list '(m))
(cl:defmethod motor_custom_data-val ((m <GripperCyclic_CustomData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:motor_custom_data-val is deprecated.  Use kortex_driver-msg:motor_custom_data instead.")
  (motor_custom_data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GripperCyclic_CustomData>) ostream)
  "Serializes a message object of type '<GripperCyclic_CustomData>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'custom_data_id) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'gripper_custom_data) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'motor_custom_data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'motor_custom_data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GripperCyclic_CustomData>) istream)
  "Deserializes a message object of type '<GripperCyclic_CustomData>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'custom_data_id) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'gripper_custom_data) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'motor_custom_data) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'motor_custom_data)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'kortex_driver-msg:CustomDataUnit))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GripperCyclic_CustomData>)))
  "Returns string type for a message object of type '<GripperCyclic_CustomData>"
  "kortex_driver/GripperCyclic_CustomData")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GripperCyclic_CustomData)))
  "Returns string type for a message object of type 'GripperCyclic_CustomData"
  "kortex_driver/GripperCyclic_CustomData")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GripperCyclic_CustomData>)))
  "Returns md5sum for a message object of type '<GripperCyclic_CustomData>"
  "ccb37c7f71598724d847a97b3a0c62f5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GripperCyclic_CustomData)))
  "Returns md5sum for a message object of type 'GripperCyclic_CustomData"
  "ccb37c7f71598724d847a97b3a0c62f5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GripperCyclic_CustomData>)))
  "Returns full string definition for message of type '<GripperCyclic_CustomData>"
  (cl:format cl:nil "~%GripperCyclic_MessageId custom_data_id~%CustomDataUnit gripper_custom_data~%CustomDataUnit[] motor_custom_data~%================================================================================~%MSG: kortex_driver/GripperCyclic_MessageId~%~%uint32 identifier~%================================================================================~%MSG: kortex_driver/CustomDataUnit~%~%uint32 custom_data_0~%uint32 custom_data_1~%uint32 custom_data_2~%uint32 custom_data_3~%uint32 custom_data_4~%uint32 custom_data_5~%uint32 custom_data_6~%uint32 custom_data_7~%uint32 custom_data_8~%uint32 custom_data_9~%uint32 custom_data_10~%uint32 custom_data_11~%uint32 custom_data_12~%uint32 custom_data_13~%uint32 custom_data_14~%uint32 custom_data_15~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GripperCyclic_CustomData)))
  "Returns full string definition for message of type 'GripperCyclic_CustomData"
  (cl:format cl:nil "~%GripperCyclic_MessageId custom_data_id~%CustomDataUnit gripper_custom_data~%CustomDataUnit[] motor_custom_data~%================================================================================~%MSG: kortex_driver/GripperCyclic_MessageId~%~%uint32 identifier~%================================================================================~%MSG: kortex_driver/CustomDataUnit~%~%uint32 custom_data_0~%uint32 custom_data_1~%uint32 custom_data_2~%uint32 custom_data_3~%uint32 custom_data_4~%uint32 custom_data_5~%uint32 custom_data_6~%uint32 custom_data_7~%uint32 custom_data_8~%uint32 custom_data_9~%uint32 custom_data_10~%uint32 custom_data_11~%uint32 custom_data_12~%uint32 custom_data_13~%uint32 custom_data_14~%uint32 custom_data_15~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GripperCyclic_CustomData>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'custom_data_id))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'gripper_custom_data))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'motor_custom_data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GripperCyclic_CustomData>))
  "Converts a ROS message object to a list"
  (cl:list 'GripperCyclic_CustomData
    (cl:cons ':custom_data_id (custom_data_id msg))
    (cl:cons ':gripper_custom_data (gripper_custom_data msg))
    (cl:cons ':motor_custom_data (motor_custom_data msg))
))

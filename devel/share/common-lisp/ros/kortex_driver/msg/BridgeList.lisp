; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude BridgeList.msg.html

(cl:defclass <BridgeList> (roslisp-msg-protocol:ros-message)
  ((bridgeConfig
    :reader bridgeConfig
    :initarg :bridgeConfig
    :type (cl:vector kortex_driver-msg:BridgeConfig)
   :initform (cl:make-array 0 :element-type 'kortex_driver-msg:BridgeConfig :initial-element (cl:make-instance 'kortex_driver-msg:BridgeConfig))))
)

(cl:defclass BridgeList (<BridgeList>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BridgeList>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BridgeList)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<BridgeList> is deprecated: use kortex_driver-msg:BridgeList instead.")))

(cl:ensure-generic-function 'bridgeConfig-val :lambda-list '(m))
(cl:defmethod bridgeConfig-val ((m <BridgeList>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:bridgeConfig-val is deprecated.  Use kortex_driver-msg:bridgeConfig instead.")
  (bridgeConfig m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BridgeList>) ostream)
  "Serializes a message object of type '<BridgeList>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'bridgeConfig))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'bridgeConfig))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BridgeList>) istream)
  "Deserializes a message object of type '<BridgeList>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'bridgeConfig) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'bridgeConfig)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'kortex_driver-msg:BridgeConfig))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BridgeList>)))
  "Returns string type for a message object of type '<BridgeList>"
  "kortex_driver/BridgeList")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BridgeList)))
  "Returns string type for a message object of type 'BridgeList"
  "kortex_driver/BridgeList")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BridgeList>)))
  "Returns md5sum for a message object of type '<BridgeList>"
  "67edf56b8cf474f747335e24413448f0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BridgeList)))
  "Returns md5sum for a message object of type 'BridgeList"
  "67edf56b8cf474f747335e24413448f0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BridgeList>)))
  "Returns full string definition for message of type '<BridgeList>"
  (cl:format cl:nil "~%BridgeConfig[] bridgeConfig~%================================================================================~%MSG: kortex_driver/BridgeConfig~%~%uint32 device_identifier~%uint32 bridgetype~%BridgePortConfig port_config~%BridgeIdentifier bridge_id~%================================================================================~%MSG: kortex_driver/BridgePortConfig~%~%uint32 target_port~%uint32 out_port~%================================================================================~%MSG: kortex_driver/BridgeIdentifier~%~%uint32 bridge_id~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BridgeList)))
  "Returns full string definition for message of type 'BridgeList"
  (cl:format cl:nil "~%BridgeConfig[] bridgeConfig~%================================================================================~%MSG: kortex_driver/BridgeConfig~%~%uint32 device_identifier~%uint32 bridgetype~%BridgePortConfig port_config~%BridgeIdentifier bridge_id~%================================================================================~%MSG: kortex_driver/BridgePortConfig~%~%uint32 target_port~%uint32 out_port~%================================================================================~%MSG: kortex_driver/BridgeIdentifier~%~%uint32 bridge_id~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BridgeList>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'bridgeConfig) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BridgeList>))
  "Converts a ROS message object to a list"
  (cl:list 'BridgeList
    (cl:cons ':bridgeConfig (bridgeConfig msg))
))

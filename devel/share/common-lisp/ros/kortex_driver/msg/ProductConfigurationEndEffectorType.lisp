; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude ProductConfigurationEndEffectorType.msg.html

(cl:defclass <ProductConfigurationEndEffectorType> (roslisp-msg-protocol:ros-message)
  ((end_effector_type
    :reader end_effector_type
    :initarg :end_effector_type
    :type cl:integer
    :initform 0))
)

(cl:defclass ProductConfigurationEndEffectorType (<ProductConfigurationEndEffectorType>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ProductConfigurationEndEffectorType>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ProductConfigurationEndEffectorType)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<ProductConfigurationEndEffectorType> is deprecated: use kortex_driver-msg:ProductConfigurationEndEffectorType instead.")))

(cl:ensure-generic-function 'end_effector_type-val :lambda-list '(m))
(cl:defmethod end_effector_type-val ((m <ProductConfigurationEndEffectorType>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:end_effector_type-val is deprecated.  Use kortex_driver-msg:end_effector_type instead.")
  (end_effector_type m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ProductConfigurationEndEffectorType>) ostream)
  "Serializes a message object of type '<ProductConfigurationEndEffectorType>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'end_effector_type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'end_effector_type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'end_effector_type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'end_effector_type)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ProductConfigurationEndEffectorType>) istream)
  "Deserializes a message object of type '<ProductConfigurationEndEffectorType>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'end_effector_type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'end_effector_type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'end_effector_type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'end_effector_type)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ProductConfigurationEndEffectorType>)))
  "Returns string type for a message object of type '<ProductConfigurationEndEffectorType>"
  "kortex_driver/ProductConfigurationEndEffectorType")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ProductConfigurationEndEffectorType)))
  "Returns string type for a message object of type 'ProductConfigurationEndEffectorType"
  "kortex_driver/ProductConfigurationEndEffectorType")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ProductConfigurationEndEffectorType>)))
  "Returns md5sum for a message object of type '<ProductConfigurationEndEffectorType>"
  "471e42663382672f68e1e5c83df5fb95")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ProductConfigurationEndEffectorType)))
  "Returns md5sum for a message object of type 'ProductConfigurationEndEffectorType"
  "471e42663382672f68e1e5c83df5fb95")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ProductConfigurationEndEffectorType>)))
  "Returns full string definition for message of type '<ProductConfigurationEndEffectorType>"
  (cl:format cl:nil "~%uint32 end_effector_type~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ProductConfigurationEndEffectorType)))
  "Returns full string definition for message of type 'ProductConfigurationEndEffectorType"
  (cl:format cl:nil "~%uint32 end_effector_type~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ProductConfigurationEndEffectorType>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ProductConfigurationEndEffectorType>))
  "Converts a ROS message object to a list"
  (cl:list 'ProductConfigurationEndEffectorType
    (cl:cons ':end_effector_type (end_effector_type msg))
))

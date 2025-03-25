; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude BaseType.msg.html

(cl:defclass <BaseType> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass BaseType (<BaseType>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BaseType>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BaseType)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<BaseType> is deprecated: use kortex_driver-msg:BaseType instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<BaseType>)))
    "Constants for message type '<BaseType>"
  '((:BASE_TYPE_UNSPECIFIED . 0)
    (:BASE_TYPE_L53_QUICK_CONNECT . 1)
    (:BASE_TYPE_L53_FIXED . 2)
    (:BASE_TYPE_L31_FIXED . 3)
    (:BASE_TYPE_HDK_FIXED . 4))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'BaseType)))
    "Constants for message type 'BaseType"
  '((:BASE_TYPE_UNSPECIFIED . 0)
    (:BASE_TYPE_L53_QUICK_CONNECT . 1)
    (:BASE_TYPE_L53_FIXED . 2)
    (:BASE_TYPE_L31_FIXED . 3)
    (:BASE_TYPE_HDK_FIXED . 4))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BaseType>) ostream)
  "Serializes a message object of type '<BaseType>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BaseType>) istream)
  "Deserializes a message object of type '<BaseType>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BaseType>)))
  "Returns string type for a message object of type '<BaseType>"
  "kortex_driver/BaseType")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BaseType)))
  "Returns string type for a message object of type 'BaseType"
  "kortex_driver/BaseType")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BaseType>)))
  "Returns md5sum for a message object of type '<BaseType>"
  "7e0a5914d628ae4f54d3bdfb80a2c377")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BaseType)))
  "Returns md5sum for a message object of type 'BaseType"
  "7e0a5914d628ae4f54d3bdfb80a2c377")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BaseType>)))
  "Returns full string definition for message of type '<BaseType>"
  (cl:format cl:nil "~%uint32 BASE_TYPE_UNSPECIFIED = 0~%~%uint32 BASE_TYPE_L53_QUICK_CONNECT = 1~%~%uint32 BASE_TYPE_L53_FIXED = 2~%~%uint32 BASE_TYPE_L31_FIXED = 3~%~%uint32 BASE_TYPE_HDK_FIXED = 4~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BaseType)))
  "Returns full string definition for message of type 'BaseType"
  (cl:format cl:nil "~%uint32 BASE_TYPE_UNSPECIFIED = 0~%~%uint32 BASE_TYPE_L53_QUICK_CONNECT = 1~%~%uint32 BASE_TYPE_L53_FIXED = 2~%~%uint32 BASE_TYPE_L31_FIXED = 3~%~%uint32 BASE_TYPE_HDK_FIXED = 4~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BaseType>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BaseType>))
  "Converts a ROS message object to a list"
  (cl:list 'BaseType
))

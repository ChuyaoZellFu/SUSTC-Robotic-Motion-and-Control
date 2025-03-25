; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude VisionModuleType.msg.html

(cl:defclass <VisionModuleType> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass VisionModuleType (<VisionModuleType>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <VisionModuleType>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'VisionModuleType)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<VisionModuleType> is deprecated: use kortex_driver-msg:VisionModuleType instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<VisionModuleType>)))
    "Constants for message type '<VisionModuleType>"
  '((:VISION_MODULE_TYPE_UNSPECIFIED . 0)
    (:VISION_MODULE_TYPE_NOT_INSTALLED . 1)
    (:VISION_MODULE_TYPE_L53_2D3D . 2)
    (:VISION_MODULE_TYPE_L53_2D . 3)
    (:VISION_MODULE_TYPE_EOD . 4))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'VisionModuleType)))
    "Constants for message type 'VisionModuleType"
  '((:VISION_MODULE_TYPE_UNSPECIFIED . 0)
    (:VISION_MODULE_TYPE_NOT_INSTALLED . 1)
    (:VISION_MODULE_TYPE_L53_2D3D . 2)
    (:VISION_MODULE_TYPE_L53_2D . 3)
    (:VISION_MODULE_TYPE_EOD . 4))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <VisionModuleType>) ostream)
  "Serializes a message object of type '<VisionModuleType>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <VisionModuleType>) istream)
  "Deserializes a message object of type '<VisionModuleType>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<VisionModuleType>)))
  "Returns string type for a message object of type '<VisionModuleType>"
  "kortex_driver/VisionModuleType")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'VisionModuleType)))
  "Returns string type for a message object of type 'VisionModuleType"
  "kortex_driver/VisionModuleType")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<VisionModuleType>)))
  "Returns md5sum for a message object of type '<VisionModuleType>"
  "bb31bed96ad6e07f3b30f11301afe3f6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'VisionModuleType)))
  "Returns md5sum for a message object of type 'VisionModuleType"
  "bb31bed96ad6e07f3b30f11301afe3f6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<VisionModuleType>)))
  "Returns full string definition for message of type '<VisionModuleType>"
  (cl:format cl:nil "~%uint32 VISION_MODULE_TYPE_UNSPECIFIED = 0~%~%uint32 VISION_MODULE_TYPE_NOT_INSTALLED = 1~%~%uint32 VISION_MODULE_TYPE_L53_2D3D = 2~%~%uint32 VISION_MODULE_TYPE_L53_2D = 3~%~%uint32 VISION_MODULE_TYPE_EOD = 4~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'VisionModuleType)))
  "Returns full string definition for message of type 'VisionModuleType"
  (cl:format cl:nil "~%uint32 VISION_MODULE_TYPE_UNSPECIFIED = 0~%~%uint32 VISION_MODULE_TYPE_NOT_INSTALLED = 1~%~%uint32 VISION_MODULE_TYPE_L53_2D3D = 2~%~%uint32 VISION_MODULE_TYPE_L53_2D = 3~%~%uint32 VISION_MODULE_TYPE_EOD = 4~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <VisionModuleType>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <VisionModuleType>))
  "Converts a ROS message object to a list"
  (cl:list 'VisionModuleType
))

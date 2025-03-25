; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude BaseCyclic_ServiceVersion.msg.html

(cl:defclass <BaseCyclic_ServiceVersion> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass BaseCyclic_ServiceVersion (<BaseCyclic_ServiceVersion>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BaseCyclic_ServiceVersion>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BaseCyclic_ServiceVersion)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<BaseCyclic_ServiceVersion> is deprecated: use kortex_driver-msg:BaseCyclic_ServiceVersion instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<BaseCyclic_ServiceVersion>)))
    "Constants for message type '<BaseCyclic_ServiceVersion>"
  '((:RESERVED_0 . 0)
    (:CURRENT_VERSION . 1))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'BaseCyclic_ServiceVersion)))
    "Constants for message type 'BaseCyclic_ServiceVersion"
  '((:RESERVED_0 . 0)
    (:CURRENT_VERSION . 1))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BaseCyclic_ServiceVersion>) ostream)
  "Serializes a message object of type '<BaseCyclic_ServiceVersion>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BaseCyclic_ServiceVersion>) istream)
  "Deserializes a message object of type '<BaseCyclic_ServiceVersion>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BaseCyclic_ServiceVersion>)))
  "Returns string type for a message object of type '<BaseCyclic_ServiceVersion>"
  "kortex_driver/BaseCyclic_ServiceVersion")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BaseCyclic_ServiceVersion)))
  "Returns string type for a message object of type 'BaseCyclic_ServiceVersion"
  "kortex_driver/BaseCyclic_ServiceVersion")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BaseCyclic_ServiceVersion>)))
  "Returns md5sum for a message object of type '<BaseCyclic_ServiceVersion>"
  "d8624be8e7830df48d9964b7612fa5b3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BaseCyclic_ServiceVersion)))
  "Returns md5sum for a message object of type 'BaseCyclic_ServiceVersion"
  "d8624be8e7830df48d9964b7612fa5b3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BaseCyclic_ServiceVersion>)))
  "Returns full string definition for message of type '<BaseCyclic_ServiceVersion>"
  (cl:format cl:nil "~%uint32 RESERVED_0 = 0~%~%uint32 CURRENT_VERSION = 1~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BaseCyclic_ServiceVersion)))
  "Returns full string definition for message of type 'BaseCyclic_ServiceVersion"
  (cl:format cl:nil "~%uint32 RESERVED_0 = 0~%~%uint32 CURRENT_VERSION = 1~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BaseCyclic_ServiceVersion>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BaseCyclic_ServiceVersion>))
  "Converts a ROS message object to a list"
  (cl:list 'BaseCyclic_ServiceVersion
))

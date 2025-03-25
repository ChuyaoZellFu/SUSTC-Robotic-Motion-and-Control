; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude Permission.msg.html

(cl:defclass <Permission> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass Permission (<Permission>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Permission>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Permission)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<Permission> is deprecated: use kortex_driver-msg:Permission instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<Permission>)))
    "Constants for message type '<Permission>"
  '((:NO_PERMISSION . 0)
    (:READ_PERMISSION . 1)
    (:UPDATE_PERMISSION . 2)
    (:DELETE_PERMISSION . 4))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'Permission)))
    "Constants for message type 'Permission"
  '((:NO_PERMISSION . 0)
    (:READ_PERMISSION . 1)
    (:UPDATE_PERMISSION . 2)
    (:DELETE_PERMISSION . 4))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Permission>) ostream)
  "Serializes a message object of type '<Permission>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Permission>) istream)
  "Deserializes a message object of type '<Permission>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Permission>)))
  "Returns string type for a message object of type '<Permission>"
  "kortex_driver/Permission")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Permission)))
  "Returns string type for a message object of type 'Permission"
  "kortex_driver/Permission")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Permission>)))
  "Returns md5sum for a message object of type '<Permission>"
  "2ae002b57072e72c8a5ded36bcd8b246")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Permission)))
  "Returns md5sum for a message object of type 'Permission"
  "2ae002b57072e72c8a5ded36bcd8b246")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Permission>)))
  "Returns full string definition for message of type '<Permission>"
  (cl:format cl:nil "~%uint32 NO_PERMISSION = 0~%~%uint32 READ_PERMISSION = 1~%~%uint32 UPDATE_PERMISSION = 2~%~%uint32 DELETE_PERMISSION = 4~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Permission)))
  "Returns full string definition for message of type 'Permission"
  (cl:format cl:nil "~%uint32 NO_PERMISSION = 0~%~%uint32 READ_PERMISSION = 1~%~%uint32 UPDATE_PERMISSION = 2~%~%uint32 DELETE_PERMISSION = 4~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Permission>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Permission>))
  "Converts a ROS message object to a list"
  (cl:list 'Permission
))

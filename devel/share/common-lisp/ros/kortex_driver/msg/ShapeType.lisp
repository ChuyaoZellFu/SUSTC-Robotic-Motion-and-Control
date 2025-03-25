; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude ShapeType.msg.html

(cl:defclass <ShapeType> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass ShapeType (<ShapeType>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ShapeType>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ShapeType)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<ShapeType> is deprecated: use kortex_driver-msg:ShapeType instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<ShapeType>)))
    "Constants for message type '<ShapeType>"
  '((:UNSPECIFIED_SHAPE . 0)
    (:CYLINDER . 1)
    (:SPHERE . 2)
    (:RECTANGULAR_PRISM . 3))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'ShapeType)))
    "Constants for message type 'ShapeType"
  '((:UNSPECIFIED_SHAPE . 0)
    (:CYLINDER . 1)
    (:SPHERE . 2)
    (:RECTANGULAR_PRISM . 3))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ShapeType>) ostream)
  "Serializes a message object of type '<ShapeType>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ShapeType>) istream)
  "Deserializes a message object of type '<ShapeType>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ShapeType>)))
  "Returns string type for a message object of type '<ShapeType>"
  "kortex_driver/ShapeType")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ShapeType)))
  "Returns string type for a message object of type 'ShapeType"
  "kortex_driver/ShapeType")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ShapeType>)))
  "Returns md5sum for a message object of type '<ShapeType>"
  "8cce3154d49982341a5a162e90cffa5a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ShapeType)))
  "Returns md5sum for a message object of type 'ShapeType"
  "8cce3154d49982341a5a162e90cffa5a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ShapeType>)))
  "Returns full string definition for message of type '<ShapeType>"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_SHAPE = 0~%~%uint32 CYLINDER = 1~%~%uint32 SPHERE = 2~%~%uint32 RECTANGULAR_PRISM = 3~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ShapeType)))
  "Returns full string definition for message of type 'ShapeType"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_SHAPE = 0~%~%uint32 CYLINDER = 1~%~%uint32 SPHERE = 2~%~%uint32 RECTANGULAR_PRISM = 3~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ShapeType>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ShapeType>))
  "Converts a ROS message object to a list"
  (cl:list 'ShapeType
))

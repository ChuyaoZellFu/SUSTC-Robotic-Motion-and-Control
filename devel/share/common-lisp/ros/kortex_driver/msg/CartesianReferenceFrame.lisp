; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude CartesianReferenceFrame.msg.html

(cl:defclass <CartesianReferenceFrame> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass CartesianReferenceFrame (<CartesianReferenceFrame>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CartesianReferenceFrame>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CartesianReferenceFrame)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<CartesianReferenceFrame> is deprecated: use kortex_driver-msg:CartesianReferenceFrame instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<CartesianReferenceFrame>)))
    "Constants for message type '<CartesianReferenceFrame>"
  '((:CARTESIAN_REFERENCE_FRAME_UNSPECIFIED . 0)
    (:CARTESIAN_REFERENCE_FRAME_MIXED . 1)
    (:CARTESIAN_REFERENCE_FRAME_TOOL . 2)
    (:CARTESIAN_REFERENCE_FRAME_BASE . 3))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'CartesianReferenceFrame)))
    "Constants for message type 'CartesianReferenceFrame"
  '((:CARTESIAN_REFERENCE_FRAME_UNSPECIFIED . 0)
    (:CARTESIAN_REFERENCE_FRAME_MIXED . 1)
    (:CARTESIAN_REFERENCE_FRAME_TOOL . 2)
    (:CARTESIAN_REFERENCE_FRAME_BASE . 3))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CartesianReferenceFrame>) ostream)
  "Serializes a message object of type '<CartesianReferenceFrame>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CartesianReferenceFrame>) istream)
  "Deserializes a message object of type '<CartesianReferenceFrame>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CartesianReferenceFrame>)))
  "Returns string type for a message object of type '<CartesianReferenceFrame>"
  "kortex_driver/CartesianReferenceFrame")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CartesianReferenceFrame)))
  "Returns string type for a message object of type 'CartesianReferenceFrame"
  "kortex_driver/CartesianReferenceFrame")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CartesianReferenceFrame>)))
  "Returns md5sum for a message object of type '<CartesianReferenceFrame>"
  "82b14912eebdea49eac0f3e20689bea8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CartesianReferenceFrame)))
  "Returns md5sum for a message object of type 'CartesianReferenceFrame"
  "82b14912eebdea49eac0f3e20689bea8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CartesianReferenceFrame>)))
  "Returns full string definition for message of type '<CartesianReferenceFrame>"
  (cl:format cl:nil "~%uint32 CARTESIAN_REFERENCE_FRAME_UNSPECIFIED = 0~%~%uint32 CARTESIAN_REFERENCE_FRAME_MIXED = 1~%~%uint32 CARTESIAN_REFERENCE_FRAME_TOOL = 2~%~%uint32 CARTESIAN_REFERENCE_FRAME_BASE = 3~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CartesianReferenceFrame)))
  "Returns full string definition for message of type 'CartesianReferenceFrame"
  (cl:format cl:nil "~%uint32 CARTESIAN_REFERENCE_FRAME_UNSPECIFIED = 0~%~%uint32 CARTESIAN_REFERENCE_FRAME_MIXED = 1~%~%uint32 CARTESIAN_REFERENCE_FRAME_TOOL = 2~%~%uint32 CARTESIAN_REFERENCE_FRAME_BASE = 3~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CartesianReferenceFrame>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CartesianReferenceFrame>))
  "Converts a ROS message object to a list"
  (cl:list 'CartesianReferenceFrame
))

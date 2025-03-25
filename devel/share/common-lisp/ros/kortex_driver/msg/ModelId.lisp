; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude ModelId.msg.html

(cl:defclass <ModelId> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass ModelId (<ModelId>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ModelId>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ModelId)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<ModelId> is deprecated: use kortex_driver-msg:ModelId instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<ModelId>)))
    "Constants for message type '<ModelId>"
  '((:MODEL_ID_UNSPECIFIED . 0)
    (:MODEL_ID_L53 . 1)
    (:MODEL_ID_L31 . 2)
    (:MODEL_ID_HDK . 3))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'ModelId)))
    "Constants for message type 'ModelId"
  '((:MODEL_ID_UNSPECIFIED . 0)
    (:MODEL_ID_L53 . 1)
    (:MODEL_ID_L31 . 2)
    (:MODEL_ID_HDK . 3))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ModelId>) ostream)
  "Serializes a message object of type '<ModelId>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ModelId>) istream)
  "Deserializes a message object of type '<ModelId>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ModelId>)))
  "Returns string type for a message object of type '<ModelId>"
  "kortex_driver/ModelId")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ModelId)))
  "Returns string type for a message object of type 'ModelId"
  "kortex_driver/ModelId")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ModelId>)))
  "Returns md5sum for a message object of type '<ModelId>"
  "7fb7d718b4a478fa436cc49becd077b0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ModelId)))
  "Returns md5sum for a message object of type 'ModelId"
  "7fb7d718b4a478fa436cc49becd077b0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ModelId>)))
  "Returns full string definition for message of type '<ModelId>"
  (cl:format cl:nil "~%uint32 MODEL_ID_UNSPECIFIED = 0~%~%uint32 MODEL_ID_L53 = 1~%~%uint32 MODEL_ID_L31 = 2~%~%uint32 MODEL_ID_HDK = 3~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ModelId)))
  "Returns full string definition for message of type 'ModelId"
  (cl:format cl:nil "~%uint32 MODEL_ID_UNSPECIFIED = 0~%~%uint32 MODEL_ID_L53 = 1~%~%uint32 MODEL_ID_L31 = 2~%~%uint32 MODEL_ID_HDK = 3~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ModelId>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ModelId>))
  "Converts a ROS message object to a list"
  (cl:list 'ModelId
))

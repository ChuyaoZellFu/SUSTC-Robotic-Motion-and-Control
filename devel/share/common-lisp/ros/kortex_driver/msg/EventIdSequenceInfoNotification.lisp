; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude EventIdSequenceInfoNotification.msg.html

(cl:defclass <EventIdSequenceInfoNotification> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass EventIdSequenceInfoNotification (<EventIdSequenceInfoNotification>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <EventIdSequenceInfoNotification>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'EventIdSequenceInfoNotification)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<EventIdSequenceInfoNotification> is deprecated: use kortex_driver-msg:EventIdSequenceInfoNotification instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<EventIdSequenceInfoNotification>)))
    "Constants for message type '<EventIdSequenceInfoNotification>"
  '((:UNSPECIFIED_SEQUENCE_EVENT . 0)
    (:SEQUENCE_COMPLETED . 1)
    (:SEQUENCE_ABORTED . 2)
    (:SEQUENCE_PAUSED . 3)
    (:SEQUENCE_TASK_STARTED . 4)
    (:SEQUENCE_TASK_COMPLETED . 5)
    (:SEQUENCE_STARTED . 6))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'EventIdSequenceInfoNotification)))
    "Constants for message type 'EventIdSequenceInfoNotification"
  '((:UNSPECIFIED_SEQUENCE_EVENT . 0)
    (:SEQUENCE_COMPLETED . 1)
    (:SEQUENCE_ABORTED . 2)
    (:SEQUENCE_PAUSED . 3)
    (:SEQUENCE_TASK_STARTED . 4)
    (:SEQUENCE_TASK_COMPLETED . 5)
    (:SEQUENCE_STARTED . 6))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <EventIdSequenceInfoNotification>) ostream)
  "Serializes a message object of type '<EventIdSequenceInfoNotification>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <EventIdSequenceInfoNotification>) istream)
  "Deserializes a message object of type '<EventIdSequenceInfoNotification>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<EventIdSequenceInfoNotification>)))
  "Returns string type for a message object of type '<EventIdSequenceInfoNotification>"
  "kortex_driver/EventIdSequenceInfoNotification")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EventIdSequenceInfoNotification)))
  "Returns string type for a message object of type 'EventIdSequenceInfoNotification"
  "kortex_driver/EventIdSequenceInfoNotification")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<EventIdSequenceInfoNotification>)))
  "Returns md5sum for a message object of type '<EventIdSequenceInfoNotification>"
  "68fe86a38033c2b20696485a1800dc6a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'EventIdSequenceInfoNotification)))
  "Returns md5sum for a message object of type 'EventIdSequenceInfoNotification"
  "68fe86a38033c2b20696485a1800dc6a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<EventIdSequenceInfoNotification>)))
  "Returns full string definition for message of type '<EventIdSequenceInfoNotification>"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_SEQUENCE_EVENT = 0~%~%uint32 SEQUENCE_COMPLETED = 1~%~%uint32 SEQUENCE_ABORTED = 2~%~%uint32 SEQUENCE_PAUSED = 3~%~%uint32 SEQUENCE_TASK_STARTED = 4~%~%uint32 SEQUENCE_TASK_COMPLETED = 5~%~%uint32 SEQUENCE_STARTED = 6~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'EventIdSequenceInfoNotification)))
  "Returns full string definition for message of type 'EventIdSequenceInfoNotification"
  (cl:format cl:nil "~%uint32 UNSPECIFIED_SEQUENCE_EVENT = 0~%~%uint32 SEQUENCE_COMPLETED = 1~%~%uint32 SEQUENCE_ABORTED = 2~%~%uint32 SEQUENCE_PAUSED = 3~%~%uint32 SEQUENCE_TASK_STARTED = 4~%~%uint32 SEQUENCE_TASK_COMPLETED = 5~%~%uint32 SEQUENCE_STARTED = 6~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <EventIdSequenceInfoNotification>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <EventIdSequenceInfoNotification>))
  "Converts a ROS message object to a list"
  (cl:list 'EventIdSequenceInfoNotification
))
